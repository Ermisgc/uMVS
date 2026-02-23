/**
 * @file 08_vertical_error_of_rectify.cpp
 * @brief 水下双目相机的垂直方向校正误差
 * @version 0.1
 * @date 2026-02-15
 * @example ./bin/Release/08_vertical_error_of_rectify.exe -m binouwcam_1110.yaml -l ./DataStereo/20251110/L -r ./DataStereo/20251110/R
 * @copyright Copyright (c) 2026
 * 
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <fstream>
#include "argparse/argparse.hpp"
#include "camera/binouwcamera.h"

using namespace NAMESPACE_U3D;
using namespace NAMESPACE_U3D::optics; 
using namespace NAMESPACE_U3D::camera;

/**
 * @brief 计算校正后左右图的垂直误差
 * @param imgL 校正后的左图
 * @param imgR 校正后的右图
 * @param out_mean 输出：当前对图像的平均垂直误差
 * @param out_max  输出：当前对图像的最大垂直误差
 * @return 匹配点对数量
 */
int computeRectificationError(const cv::Mat& imgL, const cv::Mat& imgR, double& out_mean, double& out_max, bool showMatches = true) {
    // auto createValidMask = [](const cv::Mat& img) {
    //     cv::Mat mask;
    //     // 1. 寻找非黑区域 (像素值 > 0)
    //     cv::threshold(img, mask, 1, 255, cv::THRESH_BINARY);
        
    //     // 2. 腐蚀掩模，向内收缩 15 像素，确保避开黑白交界处的剧烈梯度
    //     cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31)); 
    //     cv::erode(mask, mask, element);
    //     return mask;
    // };

    // cv::Mat maskL = createValidMask(imgL);
    // cv::Mat maskR = createValidMask(imgR);
    
    // 1. SIFT 特征提取 (保持不变)
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> kpL, kpR;
    cv::Mat desL, desR;
    sift->detectAndCompute(imgL, cv::noArray(), kpL, desL);
    sift->detectAndCompute(imgR, cv::noArray(), kpR, desR);

    if (desL.empty() || desR.empty()) return 0;

    // 2. FLANN 特征匹配 (保持不变)
    cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(5), new cv::flann::SearchParams(50));
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desL, desR, knn_matches, 2);

    // 3. 筛选匹配点并存储 DMatch 用于可视化
    std::vector<cv::DMatch> good_matches_for_show; // 用于 cv::drawMatches
    std::vector<double> v_errors;
    double max_err = 0.0;
    double sum_err = 0.0;

    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < 0.7 * knn_matches[i][1].distance) {
            
            
            const auto& ptL = kpL[knn_matches[i][0].queryIdx].pt;
            const auto& ptR = kpR[knn_matches[i][0].trainIdx].pt;

            double err = std::abs(ptL.y - ptR.y);
            if(err > 50.0) continue;  //这种情况一般是匹配到了畸变黑边的边缘位置
            v_errors.push_back(err);
            good_matches_for_show.push_back(knn_matches[i][0]);
            
            sum_err += err;
            if (err > max_err) max_err = err;
        }
    }

    if (v_errors.empty()) return 0;

    // 4. 计算统计值
    out_mean = sum_err / v_errors.size();
    out_max = max_err;

    if (showMatches) {
        cv::Mat imgMatches;
        // 绘制匹配连线
        // 使用 DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS 保证只显示匹配成功的对
        cv::drawMatches(imgL, kpL, imgR, kpR, good_matches_for_show, imgMatches, 
                        cv::Scalar(0, 255, 0), cv::Scalar::all(-1), 
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // --- 核心可视化增强：绘制水平参考线 ---
        // 为了验证 y 坐标是否一致，我们在合并图上每隔 100 像素画一条红色细线
        for (int y = 0; y < imgMatches.rows; y += 100) {
            cv::line(imgMatches, cv::Point(0, y), cv::Point(imgMatches.cols, y), cv::Scalar(0, 0, 255), 1);
        }

        // 添加文字说明
        std::string label = "Mean Error: " + std::to_string(out_mean).substr(0, 6) + " px";
        cv::putText(imgMatches, label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 255), 2);

        // 如果图片太大，缩小显示
        cv::Mat imgShow;
        double scale = 0.2; // 根据你的屏幕分辨率调整
        cv::resize(imgMatches, imgShow, cv::Size(), scale, scale);

        cv::imshow("Stereo Rectification Verification (Green: Match, Red: Epipolar Line)", imgShow);
        cv::waitKey(0); // 按任意键继续处理下一对图像
    }

    return static_cast<int>(v_errors.size());
}


int main(int argc, char** argv) {
    
    argparse::ArgumentParser program("08_vertical_error_of_rectify");
    program.add_argument("--model", "-m").required().help("Path to the binocular camera model file");
    program.add_argument("--left", "-l").required().help("Path to the left image file");
    program.add_argument("--right", "-r").required().help("Path to the right image file");
    program.add_argument("--count", "-c").default_value(55).help("Number of image pairs").scan<'d', int>();
    program.add_argument("--out", "-o").default_value("out.csv").help("Path to the output csv file");
    program.add_argument("--show", "-s").flag().default_value(false).help("Show matches");

    try{
        program.parse_args(argc, argv);
    } catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cerr << program;
        return 1;
    }

    std::string modelPath = program.get<std::string>("--model");
    std::string left_dir = program.get<std::string>("--left");
    std::string right_dir = program.get<std::string>("--right");
    int count = program.get<int>("--count");
    std::string outPath = program.get<std::string>("--out");
    bool showMatches = program.is_used("--show");
    std::cout << "Args:" << std::endl;
    std::cout << ">>> modelPath: " << modelPath << std::endl;
    std::cout << ">>> left_dir: " << left_dir << std::endl;
    std::cout << ">>> right_dir: " << right_dir << std::endl;
    std::cout << ">>> count: " << count << std::endl;
    std::cout << ">>> outPath: " << outPath << std::endl;

    std::ofstream outFile(outPath);
    if(!outFile.is_open()){
        std::cerr << "Error: Failed to open output file:" << outPath << std::endl;
        return 1;
    }
    outFile << "image_index,match_count,mean_error,max_error" << std::endl;

    // 加载双目相机模型
    BinoUWCamera camera;
    cv::FileStorage fs(modelPath, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cerr << "Error: Failed to open model file." << std::endl;
        return 1;
    }
    camera.read(fs.root());
    fs.release();
    std::cout << "Info: BinocularCamera model loaded successfully:" << std::endl << camera << std::endl;
    for(int i = 1; i <= program.get<int>("--count"); i++){
        std::string leftPath = left_dir + "/" + std::to_string(i) + ".bmp";
        std::string rightPath = right_dir + "/" + std::to_string(i) + ".bmp";
        // 加载左右图像并进行立体校正
        cv::Mat leftImg = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
        cv::Mat rightImg = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
        if(leftImg.empty() || rightImg.empty()){
            std::cerr << "Error: Failed to load images. leftPath: " << leftPath << ", rightPath: " << rightPath << std::endl;
            return 1;
        }

        auto [rectifiedLeftImg, rectifiedRightImg] = camera.rectify(leftImg, rightImg);
        if(rectifiedLeftImg.empty() || rectifiedRightImg.empty()){
            std::cerr << "Error: Failed to rectify images." << std::endl;
            return 1;
        }
        
        //转为彩色图：
        cv::Mat rectifiedLeftImgColor;
        cv::cvtColor(rectifiedLeftImg, rectifiedLeftImgColor, cv::COLOR_GRAY2BGR);

        cv::Mat rectifiedRightImgColor;
        cv::cvtColor(rectifiedRightImg, rectifiedRightImgColor, cv::COLOR_GRAY2BGR);

        double out_mean, out_max;
        int match_count = computeRectificationError(rectifiedLeftImg, rectifiedRightImg, out_mean, out_max, showMatches);
        outFile << i << "," << match_count << "," << out_mean << "," << out_max << std::endl;
        std::cout << "Info: Image " << i << " processed. Match count: " << match_count << ", Mean error: " << out_mean << ", Max error: " << out_max << std::endl;
    }

    outFile.close();

    return 0;
}

