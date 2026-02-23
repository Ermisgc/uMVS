/**
 * @file 00_test_camera_model.cpp
 * @author ermis
 * @brief 单元测试，验证针孔相机模型的投影、反投影及序列化功能
 * @details 本文件的测试场景如下：
 * 1. 相机构造测试，构造一个虚拟的相机场景，给定一系列虚拟参数，包括相机内参矩阵、相机外参矩阵、相机畸变参数、相机传感器尺寸等。
 * 2. 正向投影测试，测试在有畸变的情况下，投影到的像素点是否符合桶形畸变模型
 * 3. 闭环测试，将正向投影测试得到的像素点，反向投影到世界坐标，然后测试点线距离是否小于阈值
 * 4. 序列化测试，测试相机模型的序列化和反序列化功能是否正确
 * @version 0.1
 * @date 2026-01-02
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cassert>
#include <cmath>

// 包含你的头文件
#include "camera/monocamera.h"
#include "optics/geometry.h"

using namespace NAMESPACE_U3D;
using namespace NAMESPACE_U3D::camera;
using namespace NAMESPACE_U3D::optics;

int main(){
    // 假设分辨率 1920x1080
    cv::Size imgSize(1920, 1080);
    
    // 内参 (fx, fy, cx, cy)
    Matx33d K(1000.0, 0.0, 960.0,
              0.0, 1000.0, 540.0,
              0.0, 0.0, 1.0);
    
    // 畸变 (k1, k2, p1, p2, k3) - 稍微加一点桶形畸变测试去畸变能力
    Vec5d D(-0.1, 0.01, 0.0, 0.0, 0.0);

    // 外参：相机位于世界坐标 (0, -0.5, -2.0)，稍微仰视看原点
    // 这里我们简单起见，设为单位阵和零平移，先测纯投影
    Matx33d R = Matx33d::eye(); 
    Vec3d t(0, 0, 0); 

    MonoCamera cam(K, D, imgSize, R, t);

    if (!cam.isValid()) {
        std::cerr << "[Error] CameraModel is not valid!" << std::endl;
        return -1;
    }
    std::cout << "[Pass] CameraModel is valid!" << std::endl;

    //正向投影测试
    std::cout << "[Test] Test forward projection ... " << std::endl;
    Vec3d P_world(0.5, 0.5, 5.0);
    auto uv_opt = cam.forwardProject(P_world);
    if(!uv_opt.has_value()) {
        std::cerr << "[Error] Forward projection failed!" << std::endl;
        return -1;
    }
    Vec2d uv = uv_opt.value();
    
    //这里估算一下大致的值：u = fx * x/z + cx = 1000 * 0.5/5.0 + 960 = 1060, 
    //v = fy * y/z + cy = 1000 * 0.5/5.0 + 540 = 640
    //如果是桶形畸变的话，投影到的像素点要内收，因此u,v要在[1000, 1060]x[540, 640]内
    if(uv[0] < 1060 && uv[0] > 1000 && uv[1] < 640 && uv[1] > 540){
        std::cout << "[Pass] Forward projection is valid! World Point = " << P_world << ", uv = " << uv << std::endl;
    } else {
        std::cerr << "[Error] Forward projection failed! Not according to the distortion model" << std::endl;
    }


    //闭环测试
    std::cout << "[Test] Test closed loop projection ... " << std::endl;
    auto ray = cam.pixelToWorldRay(uv);
    double dist = distPointToLine(P_world, ray);
    if(dist < 1e-6){
        std::cout << "[Pass] Closed loop projection is valid! World Point = " << P_world << ", dist = " << dist << std::endl;
    } else {
        std::cerr << "[Error] Closed loop projection failed! Not according to the distortion model" << std::endl;
    }

    //序列化测试
    std::string filename = "test_cam.yaml";

    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        cam.write(fs);
        fs.release();
    }

    MonoCamera cam_loaded;
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        cam_loaded.read(fs.root()); // 读取根节点
        fs.release();
    }

    double k_diff = cv::norm(cam.K - cam_loaded.K);
    double d_diff = cv::norm(cam.D - cam_loaded.D);

    if (k_diff < 1e-6 && d_diff < 1e-6) {
        std::cout << "[Pass] Serialization and deserialization are consistent!" << std::endl;
    } else {
        std::cerr << "[Fail] Serialization and deserialization are not consistent!" << std::endl;
        return -1;
    }

    std::cout << "[Pass] All tests passed!" << std::endl;
    return 0;
}