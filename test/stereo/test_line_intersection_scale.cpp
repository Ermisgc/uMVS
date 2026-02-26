/**
 * @file test_line_intersection_scale.cpp
 * @author ermis
 * @brief 大规模测试两条射线中垂线的计算
 * @version 0.1
 * @date 2026-02-25
 * @copyright Copyright (c) 2026
 */

#include "optics/geometry.h"
#include "argparse/argparse.hpp"
#include <random>
using namespace NAMESPACE_U3D::optics;

std::pair<Ray, Ray> generateRaysWithMidPoint(const cv::Point3f& target_mid, std::mt19937& rng) {
    std::uniform_real_distribution<double> dir_dist(-1.0, 1.0);
    cv::Vec3d d1, d2, n;
    // 确保d1、d2非平行（点积绝对值<0.99）
    do {
        d1 = cv::Vec3d(dir_dist(rng), dir_dist(rng), dir_dist(rng));
        d2 = cv::Vec3d(dir_dist(rng), dir_dist(rng), dir_dist(rng));
        normalize(d1);
        normalize(d2);
        n = d1.cross(d2);
    } while (cv::norm(n) < 0.05);
    n = normalize(n);
    CV_Assert(std::abs(cv::norm(n) - 1.0) < epsilon());

    std::uniform_real_distribution<double> dist_dist(0.0, 5.0);
    std::uniform_real_distribution<double> t_dist(1.0, 10.0); // 随机t1、t2范围
    double half_dist = dist_dist(rng);

    cv::Vec3d mid_d(target_mid.x, target_mid.y, target_mid.z);
    cv::Vec3d p1 = mid_d + half_dist * n;
    cv::Vec3d p2 = mid_d - half_dist * n;

    double t1 = t_dist(rng);
    double t2 = t_dist(rng);

    Ray ray1, ray2;
    ray1.o = p1 - t1 * d1;
    ray1.d = d1;
    ray2.o = p2 - t2 * d2;
    ray2.d = d2;
    return {ray1, ray2};
}

bool verifyMidPoint(const cv::Point3f& target_mid, const std::optional<cv::Point3d>& computed_mid) {
    if (!computed_mid) {
        return false;
    }

    // 计算两点间误差
    double dx = target_mid.x - computed_mid->x;
    double dy = target_mid.y - computed_mid->y;
    double dz = target_mid.z - computed_mid->z;
    double total_error = sqrt(dx*dx + dy*dy + dz*dz);

    return total_error < 1e-4;
}

int main(int argc, char* argv[]){
    argparse::ArgumentParser parser("test_line_intersection_scale");
    parser.add_argument("--random_number", "-r").default_value(42).help("Random number for testing").scan<'i', int>();
    parser.add_argument("--test_count", "-n").default_value(100).help("Number of test cases to run").scan<'i', int>();

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    int random_number = parser.get<int>("--random_number");
    int test_count = parser.get<int>("--test_count");

    std::random_device rd;
    std::mt19937 rng(rd());
    int success_count = 0;
    for (int i = 0; i < test_count; ++i) {
        std::uniform_real_distribution<float> mid_dist(-50.0f, 50.0f);
        cv::Point3f random_target(mid_dist(rng), mid_dist(rng), mid_dist(rng));
        auto [ray1, ray2] = generateRaysWithMidPoint(random_target, rng);
        auto computed_mid = intersect(ray1, ray2);
        if (verifyMidPoint(random_target, computed_mid)) {
            success_count++;
        } 
        std::cout << "\r测试进度： " << (i + 1) << "/" << test_count << " 成功数： " << success_count << "/" << (i + 1) << std::flush;
    }

    return 0;
}