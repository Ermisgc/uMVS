/**
 * @file 00_simple_refraction.cpp
 * @author ermis
 * @brief 简单折射测试，验证折射公式的实现是否正确
 * @details 本文件的测试场景如下：
 * 1. 构造一个简单相机，位于原点，看向+Z方向
 * 2. 定义两层平面折射界面 (相机坐标系下)，一层是空气到玻璃，z = 0.10，一层是玻璃到水，z = 0.12
 * 3. 选取一些测试像素（中心和四个角），对每个像素，用CameraModel::pixelToRefractedRayCam得到其在相机坐标系中的折射射线
 * 检查与第二个界面的交点是否在z = 0.12的平面上，然后检查第一界面处是否满足Snell定律n1 sin(theta1) = n2 sin(theta2)
 * 
 * @version 0.1
 * @date 2025-12-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <iostream>
#include <vector>
#include <cmath>

#include "utils.h"
#include "optics/refraction.h"
#include "camera/camera_model.h"

int main() {
    using namespace NAMESPACE_U3D;
    using namespace NAMESPACE_U3D::refract;
    using namespace NAMESPACE_U3D::camera;

    // 1. 构造一个简单相机
    int width  = 640;
    int height = 480;
    double fx = 500.0;
    double fy = 500.0;
    double cx = (width  - 1) * 0.5;
    double cy = (height - 1) * 0.5;

    // 相机坐标系与世界坐标系重合：Pc = Pw
    Matx33d R_cw = Matx33d::eye();
    Vec3d   t_cw(0.0, 0.0, 0.0);

    CameraModel cam(fx, fy, cx, cy, width, height, R_cw, t_cw);

    // 2. 定义两层平面折射界面 (相机坐标系下)
    // 平面方程: n·x + d = 0, 取 n = (0,0,1)
    // z = z0 => (0,0,1)·(x,y,z) - z0 = 0 => d = -z0

    double z1 = 0.10;  // 空气->玻璃
    double z2 = 0.12;  // 玻璃->水

    Plane air_glass(Vec3d(0.0, 0.0, 1.0), -z1);
    Plane glass_water(Vec3d(0.0, 0.0, 1.0), -z2);

    std::vector<RefractiveInterface> interfaces;
    interfaces.emplace_back(air_glass,   1.0, 1.5);  // 空气 -> 玻璃
    interfaces.emplace_back(glass_water, 1.5, 1.33); // 玻璃 -> 水

    // 3. 选取一些测试像素
    std::vector<cv::Point2d> testPixels = {
        {cx, cy},                    // 中心
        {0.0, 0.0},                  // 左上
        {width - 1.0, 0.0},          // 右上
        {0.0, height - 1.0},         // 左下
        {width - 1.0, height - 1.0}  // 右下
    };

    std::cout << "Refraction numerical test";
    std::cout << "Camera: fx=" << fx << " fy=" << fy
              << " cx=" << cx << " cy=" << cy << "\n";
    std::cout << "Interfaces: air->glass at z=" << z1
              << ", glass->water at z=" << z2 << "\n";

    for (const auto& pix : testPixels) {
        double u = pix.x;
        double v = pix.y;

        std::cout << "Pixel (" << u << ", " << v << ")\n";

        // 3.1 原始相机射线（未折射）
        Ray camRay = cam.pixelToCamRay(u, v);
        std::cout << "  Cam ray dir = ["
                  << camRay.d[0] << ", "
                  << camRay.d[1] << ", "
                  << camRay.d[2] << "]\n";

        // 3.2 折射后的相机坐标系射线
        auto refrRayOpt = cam.pixelToRefractedRayCam(u, v, interfaces);
        if (!refrRayOpt.has_value()) {
            std::cout << "  Refraction failed (total internal reflection or no intersection)\n";
            continue;
        }

        Ray refrCamRay = refrRayOpt.value();
        std::cout << "  Refracted ray origin (cam) = ["
                  << refrCamRay.o[0] << ", "
                  << refrCamRay.o[1] << ", "
                  << refrCamRay.o[2] << "]\n";
        std::cout << "  Refracted ray dir (cam)    = ["
                  << refrCamRay.d[0] << ", "
                  << refrCamRay.d[1] << ", "
                  << refrCamRay.d[2] << "]\n";

        // 3.3 检查最后交点是否在第二个界面附近 (z ~ z2)
        double z_last = refrCamRay.o[2];
        std::cout << "  Last intersection z        = " << z_last
                  << " (expected " << z2 << "), |dz| = "
                  << std::abs(z_last - z2) << "\n";

        // 3.4 数值检查 Snell 定律在第一个界面是否成立
        //     n1 * sin(theta1) 约等于 n2 * sin(theta2)

        // 先算与界面1的交点
        Vec3d p1;
        double t1;
        if (auto ret = intersectRayPlane(camRay, air_glass, t1); !ret.has_value()) {
            std::cout << "  (Cannot intersect with first interface, skip Snell check)\n";
            continue;
        } else {
            p1 = ret.value();
        }

        Vec3d I = normalize(camRay.d);       // 入射方向
        Vec3d N = air_glass.normal;          // 法线 (0,0,1)
        double n1 = 1.0;
        double n2 = 1.5;

        Vec3d T;
        if (auto ret = refract::refract(I, N, n1, n2); !ret.has_value()) {
            std::cout << "  (Total internal reflection at first interface)\n";
            continue;
        } else {
            T = ret.value();
        }

        double cos1 = std::abs(N.dot(I));
        double cos2 = std::abs(N.dot(T));
        double sin1 = std::sqrt(std::max(0.0, 1.0 - cos1 * cos1));
        double sin2 = std::sqrt(std::max(0.0, 1.0 - cos2 * cos2));

        double lhs = n1 * sin1;
        double rhs = n2 * sin2;
        double diff = std::abs(lhs - rhs);

        std::cout << "  Snell check at interface 1:\n";
        std::cout << "    n1 * sin(theta1) = " << lhs << "\n";
        std::cout << "    n2 * sin(theta2) = " << rhs << "\n";
        std::cout << "    |diff|           = " << diff << "\n";
    }

    return 0;
}
