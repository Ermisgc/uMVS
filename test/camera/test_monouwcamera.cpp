/**
 * @file test_monouwcamera.cpp
 * @brief 水下相机模型单元测试：验证正反向投影的闭环一致性
 * 在测试中，验证了不考虑折射时的误差和考虑折射时的误差，两者相差很大
 * 
 * 代码由Gemini-3-Pro生成
 */

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "camera/monouwcamera.h"

// 使用你的命名空间
using namespace NAMESPACE_U3D::camera;

// 辅助打印函数
void printResult(const std::string& label, const cv::Point2d& orig, const cv::Point2d& proj, double error) {
    std::cout << "[" << label << "] " << std::endl;
    std::cout << "  Original Pixel : (" << orig.x << ", " << orig.y << ")" << std::endl;
    std::cout << "  Reprojected    : (" << proj.x << ", " << proj.y << ")" << std::endl;
    std::cout << "  Error          : " << error << " px" << std::endl;
    
    if (error < 0.1) {
        std::cout << "  Status         : [PASS]" << std::endl;
    } else {
        std::cout << "  Status         : [FAIL] - Error too large!" << std::endl;
    }
    std::cout << "---------------------------------------------" << std::endl;
}

int main() {
    std::cout << "=== Testing MonocularUWCamera Model ===" << std::endl;

    // ---------------------------------------------------------
    // 1. 构建针孔相机模型 (CameraModel)
    // ---------------------------------------------------------
    int width = 1280;
    int height = 720;
    
    // 内参矩阵 K (fx, fy, cx, cy)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        1000.0, 0.0,    640.0,
        0.0,    1000.0, 360.0,
        0.0,    0.0,    1.0);

    // 畸变系数 (k1, k2, p1, p2, k3) - 加一点畸变增加测试难度
    // cv::Mat D = (cv::Mat_<double>(1, 5) << -0.1, 0.01, 0.001, 0.001, 0.0);
    cv::Mat D = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // 外参 (R, t) - 设为单位阵，方便理解，即 相机系 = 世界系
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Vec3d t(0, 0, 0);

    MonoCamera pinhole(K, D, {width, height}, R, t);
    std::cout << "Camera Model initialized." << std::endl;

    // ---------------------------------------------------------
    // 2. 构建水下相机模型 (MonocularUWCamera)
    // ---------------------------------------------------------
    // 参数假设：
    double d_housing = 100.0;    // 光心到玻璃内表面 50mm
    double d_glass = 5.0;      // 玻璃厚度 10mm
    double n_air = 1.0;         // 空气
    double n_glass = 1.5;       // 玻璃
    double n_water = 1.333;     // 水
    cv::Vec3d normal(0, 0, 1);  // 玻璃法线朝前

    MonoUWCamera uw_cam(pinhole, d_housing, d_glass, n_air, n_glass, n_water, normal);
    std::cout << "Underwater Camera initialized." << std::endl;
    std::cout << "---------------------------------------------" << std::endl;

    // ---------------------------------------------------------
    // 3. 开始测试 (Loop Closure Test)
    // ---------------------------------------------------------
    
    // 测试点集合 (覆盖中心、边缘、角点)
    std::vector<cv::Point2d> test_pixels = {
        {640.0, 360.0},  // 图像中心 (理论上无折射偏转)
        {800.0, 450.0},  // 中间区域
        {100.0, 100.0},  // 左上角 (大折射角度)
        {1200.0, 700.0}  // 右下角 (大折射角度)
    };

    double target_depth = 2000.0; // 测试深度 2米 (相机Z轴方向)

    for (const auto& pixel : test_pixels) {
        // Step A: 反向投影 (Pixel -> World Point)
        auto Pw_opt = uw_cam.backwardProject(pixel, target_depth);
        
        if (!Pw_opt.has_value()) {
            std::cout << "[ERROR] Backward project failed for (" << pixel.x << "," << pixel.y << ")" << std::endl;
            continue;
        }
        cv::Vec3d Pw = Pw_opt.value();

        // Step B: 正向投影 (World Point -> Pixel)
        // 使用默认的 ITERATE 方法
        auto pixel_reproj_opt = uw_cam.forwardProject(Pw, ForwardMethod::ANALYTIC);

        if (!pixel_reproj_opt.has_value()) {
            std::cout << "[ERROR] Forward project failed for 3D Point: " << Pw << std::endl;
            continue;
        }
        cv::Point2d pixel_reproj = pixel_reproj_opt.value();

        // Step C: 计算误差
        double error = cv::norm(pixel - pixel_reproj);
        
        // 打印结果
        std::string label = (pixel.x == 640 && pixel.y == 360) ? "Center Point" : "General Point";
        printResult(label, pixel, pixel_reproj, error);

        // --- 额外验证：对比如果不考虑折射会怎样 ---
        auto pinhole_proj = pinhole.worldToPixel(Pw); 
        if(pinhole_proj) {
            cv::Vec2d pixel_vec = cv::Vec2d(pixel_reproj.x, pixel_reproj.y);
            double pinhole_error = cv::norm(pixel_vec - pinhole_proj.value());
            std::cout << "  (Compare: Pure Pinhole Error would be: " << pinhole_error << " px)" << std::endl; 
        }
        std::cout << std::endl;
    }

    return 0;
}
