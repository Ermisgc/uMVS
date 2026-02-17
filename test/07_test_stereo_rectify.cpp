/**
 * @file 07_test_stereo_rectify.cpp
 * @brief 水下双目相机的重投影误差
 * @version 0.1
 * @date 2026-02-15
 * @example 07_test_stereo_rectify.exe -c binouwcam_1110.yaml
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
#include "argparse/argparse.hpp"
#include "camera/binouwcamera.h"

using namespace NAMESPACE_U3D;
using namespace NAMESPACE_U3D::optics; 
using namespace NAMESPACE_U3D::camera;

int main(int argc, char** argv) {
    
    argparse::ArgumentParser program("07_test_stereo_rectify");
    program.add_argument("--model", "-m").required().help("Path to the binocular camera model file");
    program.add_argument("--left", "-l").required().help("Path to the left image file");
    program.add_argument("--right", "-r").required().help("Path to the right image file");
    program.add_argument("--show", "-s").flag().help("Show the rectified images");
    program.add_argument("--left_out", "-lo").help("Path to the left output image file");
    program.add_argument("--right_out", "-ro").help("Path to the right output image file");
    program.add_argument("--combine_out", "-co").help("Path to the combine output image file");

    try{
        program.parse_args(argc, argv);
    } catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cerr << program;
        return 1;
    }

    std::string modelPath = program.get<std::string>("--model");
    std::string leftPath = program.get<std::string>("--left");
    std::string rightPath = program.get<std::string>("--right");
    std::string leftOutPath = program.is_used("--left_out") ? program.get<std::string>("--left_out") : "";
    std::string rightOutPath = program.is_used("--right_out") ? program.get<std::string>("--right_out") : "";
    std::string combineOutPath = program.is_used("--combine_out") ? program.get<std::string>("--combine_out") : "";
    bool show = program.is_used("--show");
    std::cout << "Args:" << std::endl;
    std::cout << ">>> modelPath: " << modelPath << std::endl;
    std::cout << ">>> leftPath: " << leftPath << std::endl;
    std::cout << ">>> rightPath: " << rightPath << std::endl;
    std::cout << ">>> leftOutPath: " << leftOutPath << std::endl;
    std::cout << ">>> rightOutPath: " << rightOutPath << std::endl;
    std::cout << ">>> combineOutPath: " << combineOutPath << std::endl;
    std::cout << ">>> show: " << show << std::endl;

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

    // 加载左右图像并进行立体校正
    cv::Mat leftImg = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat rightImg = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
    if(leftImg.empty() || rightImg.empty()){
        std::cerr << "Error: Failed to load images." << std::endl;
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
    if(!leftOutPath.empty()){
        cv::imwrite(leftOutPath, rectifiedLeftImgColor);
        std::cout << "Rectified left image saved to " << leftOutPath << std::endl;
    }

    cv::Mat rectifiedRightImgColor;
    cv::cvtColor(rectifiedRightImg, rectifiedRightImgColor, cv::COLOR_GRAY2BGR);
    if(!rightOutPath.empty()){
        cv::imwrite(rightOutPath, rectifiedRightImgColor);
        std::cout << "Rectified right image saved to " << rightOutPath << std::endl;
    }

    if(show){
        //在两幅图中划线比较结果，验证正确性
        //在rectifiedLeftImg中画横线
        int cols = rectifiedLeftImg.cols;
        for(int i = 0; i < cols; i += 100){
            cv::line(rectifiedLeftImgColor, cv::Point(0, i), cv::Point(rectifiedLeftImg.cols, i), cv::Scalar(0, i * 255 / cols, 255 - i * 255 / cols), 2);
        }
        //在rectifiedRightImg中画横线
        for(int i = 0; i < rectifiedRightImg.cols; i += 100){
            cv::line(rectifiedRightImgColor, cv::Point(0, i), cv::Point(rectifiedRightImg.cols, i), cv::Scalar(0, i * 255 / cols, 255 - i * 255 / cols), 2);
        }

        //把两图并在一起
        cv::Mat combinedImg(rectifiedLeftImg.rows, rectifiedLeftImg.cols + rectifiedRightImg.cols, rectifiedLeftImgColor.type());
        rectifiedLeftImgColor.copyTo(combinedImg(cv::Rect(0, 0, rectifiedLeftImg.cols, rectifiedLeftImg.rows)));
        rectifiedRightImgColor.copyTo(combinedImg(cv::Rect(rectifiedLeftImg.cols, 0, rectifiedRightImg.cols, rectifiedRightImg.rows)));

        if(!combineOutPath.empty()){
            cv::imwrite(combineOutPath, combinedImg);
            std::cout << "Combined rectified image saved to " << combineOutPath << std::endl;
        }
        
        //缩小combinedImg
        cv::Mat combinedImgSmall;
        cv::resize(combinedImg, combinedImgSmall, cv::Size(), 0.25, 0.25);
        cv::imshow("Combined Rectified Images", combinedImgSmall);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}

