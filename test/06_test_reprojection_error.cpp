/**
 * @file 06_test_reprojection_error.cpp
 * @brief 测试重投影误差
 * @version 0.1
 * @date 2026-01-25
 * @example 06_test_reprojection_error.exe -d 1000.0 -c mono_uw_0310_left.yaml -o error_map.png
 * @copyright Copyright (c) 2026
 * 
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "camera/monocular_uwcamera.h"
#include "camera/binouwcamera.h"
#include "argparse/argparse.hpp"
using namespace cv;
using namespace NAMESPACE_U3D;
using namespace NAMESPACE_U3D::optics; 
using namespace NAMESPACE_U3D::camera;

int main(int argc, char** argv) {
    argparse::ArgumentParser parser("06_test_reprojection_error");
    parser.add_argument("--depth_value", "-d").default_value(1000.0).help("Depth value of water plane").scan<'g', double>();
    parser.add_argument("--camera_path", "-c").default_value("mono_uw_0310_left.yaml").help("Path to camera calibration file");
    parser.add_argument("--output", "-o").help("Path to output error map image");
    try {
        parser.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    double depth_in_water = parser.get<double>("--depth_value");
    std::string camera_path = parser.get<std::string>("--camera_path");
    std::cout << "Depth in water: " << depth_in_water << std::endl;
    std::cout << "Camera path: " << camera_path << std::endl;

    MonocularUWCamera uw_cam;
    uw_cam.read(camera_path);
    
    int H = uw_cam.camera_model_.imageSize.height;
    int W = uw_cam.camera_model_.imageSize.width;

    // --- 统计数据 ---
    Mat error_map(H, W, CV_32F, Scalar(0));
    double max_error = 0.0;
    double sum_error = 0.0;
    double sum_sq_error = 0.0;
    int valid_count = 0;

    std::cout << "Starting full resolution test (" << W << "x" << H << ")..." << std::endl;

    // --- 循环遍历所有像素 ---
    for (int v = 0; v < H; ++v) {
        if (v % 100 == 0) std::cout << "Rows processed: " << v << " / " << H << "\r" << std::flush;
        for (int u = 0; u < W; ++u) {
            Vec2d pixel_start(u, v);
            Vec3d p_world = uw_cam.backwardProject(pixel_start, depth_in_water).value();
            auto ret = uw_cam.forwardProject(p_world, ForwardMethod::ANALYTIC);
            
            if(ret) {
                Vec2d pixel_pred = ret.value();
                double err = cv::norm(pixel_start - pixel_pred); // L2 距离
                error_map.at<float>(v, u) = (float)err;
                if (err > max_error) max_error = err;
                sum_error += err;
                sum_sq_error += err * err;
                valid_count++;
            } 
        }
    }

    std::cout << "=== Test Results ===" << std::endl;
    if (valid_count > 0) {
        double mean_error = sum_error / valid_count;
        double rmse = std::sqrt(sum_sq_error / valid_count);
        
        std::cout << "Valid Pixels: " << valid_count << std::endl;
        std::cout << "Max Error:    " << max_error << " px" << std::endl;
        std::cout << "Mean Error:   " << mean_error << " px" << std::endl;
        std::cout << "RMSE:         " << rmse << " px" << std::endl;

        // --- 判定标准 ---
        if (max_error < 0.1) {
            std::cout << "[SUCCESS] Algorithm is highly accurate!" << std::endl;
        } else if (max_error < 0.5) {
            std::cout << "[WARNING] Acceptable for visual, but maybe calibration/math drift." << std::endl;
        } else {
            std::cout << "[FAILURE] Significant error detected. Check coordinate systems." << std::endl;
        }

        // --- 可视化 ---
        // 将误差图归一化显示 (0 ~ max_error)
        Mat error_vis;
        // 为了看清微小误差，可以放大系数，或者做对数变换
        // 这里简单地做 0-1 范围的 normalize
        if (max_error > 1e-9) error_map.convertTo(error_vis, CV_8U, 255.0 / max_error);
        else error_vis = Mat::zeros(H, W, CV_8U);
        
        applyColorMap(error_vis, error_vis, COLORMAP_JET);
        
        // 在图像上写上最大误差
        // std::string msg = "Max Err: " + std::to_string(max_error) + " px";
        // putText(error_vis, msg, Point(20, 50), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);

        imwrite("reprojection_error_heatmap.png", error_vis);
        std::cout << "Saved error heatmap to 'reprojection_error_heatmap.png'" << std::endl;

        // 显示图片 (如果有GUI支持)
        imshow("Error Heatmap", error_vis);
        waitKey(0);

    } else {
        std::cout << "No valid solutions found. Check your implementation." << std::endl;
    }

    return 0;
}
