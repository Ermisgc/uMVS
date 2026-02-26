/**
 * @file test_binouwcamera_reproj.cpp
 * @brief 测试水下双目相机的重建闭环
 * @author ermis
 * @date 2026-2-26
 */

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <opencv2/opencv.hpp>
#include "camera/binouwcamera.h"
#include "argparse/argparse.hpp"

using namespace NAMESPACE_U3D::camera; 

int main(int argc, char** argv) {
    argparse::ArgumentParser parser("test_binouwcamera_reproj");
    parser.add_argument("--depth_value", "-d").default_value(1000.0).help("Depth value of water plane").scan<'g', double>();
    parser.add_argument("--camera_path", "-c").default_value("./models/binouwcam_1110.yaml").help("Path to camera calibration file");
    parser.add_argument("--output", "-o").default_value("./output/error_map.png").help("Path to output error map image");
    parser.add_argument("--random_seed", "-s").default_value(10086).help("Random seed for reproducibility").scan<'i', int>();
    parser.add_argument("--test_counts", "-n").default_value(1000).help("Number of test points to generate").scan<'i', int>();
    try {
        parser.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    double depth_in_water = parser.get<double>("--depth_value");
    std::string camera_path = parser.get<std::string>("--camera_path");
    int random_seed = parser.get<int>("--random_seed");
    int test_counts = parser.get<int>("--test_counts");
    std::string output_path = parser.get<std::string>("--output");
    std::cout << "Depth in water: " << depth_in_water << std::endl;
    std::cout << "Camera path: " << camera_path << std::endl;    
    std::cout << "Random seed: " << random_seed << std::endl;
    std::cout << "Test counts: " << test_counts << std::endl;
    std::cout << "Output path: " << output_path << std::endl << std::endl;

    std::string yaml_file = camera_path;
    BinoUWCamera bino_cam;
    bino_cam.read(yaml_file);

    // 2. 准备数据容器
    std::vector<cv::Point3d> gt_pts;        // 3D Ground Truth
    std::vector<cv::Point2f> left_pts;      // 左图 2D 像素
    std::vector<cv::Point2f> right_pts;     // 右图 2D 像素

    std::mt19937 rng(random_seed); // 固定随机种子以便复现
    std::uniform_real_distribution<double> dist_x(-750.0, 750.0);
    std::uniform_real_distribution<double> dist_y(-750.0, 750.0);

    int total_trials = 1000;
    std::cout << "[进行] 正在前向投影生成 " << test_counts << " 个随机测试点..." << std::endl;

    for (int i = 0; i < test_counts; ++i) {
        cv::Vec3d Pw(dist_x(rng), dist_y(rng), depth_in_water);

        // 调用相机的正向投影模型
        auto p_l = bino_cam.left().forwardProject(Pw);
        auto p_r = bino_cam.right().forwardProject(Pw);

        // 如果该物理点在左右相机都成实像（没有全反射或超出投影模型处理范围）
        if (p_l.has_value() && p_r.has_value()) {
            // 进一步判断：点必须落在图像分辨率内 (根据你的YAML，分辨率是 4096 x 3000)
            if (p_l->x >= 0 && p_l->x < 4096 && p_l->y >= 0 && p_l->y < 3000 &&
                p_r->x >= 0 && p_r->x < 4096 && p_r->y >= 0 && p_r->y < 3000) 
            {
                gt_pts.push_back(cv::Point3d(Pw[0], Pw[1], Pw[2]));
                left_pts.push_back(cv::Point2f(p_l->x, p_l->y));
                right_pts.push_back(cv::Point2f(p_r->x, p_r->y));
            }
        }
    }
    
    int valid_count = gt_pts.size();
    std::cout << "[结果] 成功生成落在双目公共视野内的有效匹配点对: " << valid_count << " 个" << std::endl;

    if (valid_count == 0) {
        std::cerr << "[错误] 没有生成有效点，请检查生成范围或相机的朝向！" << std::endl;
        return -1;
    }

    std::vector<cv::Point3d> reconstructed_pts;
    std::cout << "[进行] 调用 BinoUWCamera::reconstruct 进行三角化..." << std::endl;
    bino_cam.reconstruct(left_pts, right_pts, reconstructed_pts);

    // 5. 校验结果与计算误差
    double max_err = 0.0;
    double mean_err = 0.0;
    
    for (int i = 0; i < valid_count; ++i) {
        double err = cv::norm(gt_pts[i] - reconstructed_pts[i]);
        max_err = std::max(max_err, err);
        mean_err += err;
    }
    mean_err /= valid_count;

    std::cout << "====== 测试统计报告 ======" << std::endl;
    std::cout << "样本总数 : " << valid_count << " 对点\n";
    std::cout << "平均误差 : " << mean_err << " mm\n";
    std::cout << "最大误差 : " << max_err << " mm\n";
    
    if (mean_err < 1e-2) {
        std::cout << "-> 结论：测试通过" << std::endl;
    } else {
        std::cout << "-> 结论：误差较大" << std::endl;
    }

    return 0;
}
