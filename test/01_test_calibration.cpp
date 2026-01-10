/**
 * @file 01_test_calibration.cpp
 * @author ermis
 * @date 2026-1-2
 * @brief 标定测试，读取本地标定板图片，然后执行相机标定
 * @example 使用示例为
 * @code
 * ./bin/Release/01_test_calibration --imgDir ./CbtCameraL --boardSize 11,8 --squareSize 15.0 --save=calibrated_params.yaml --verbose
 * @endcode
 */

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "camera/camera_model.h" 
#include "argparse/argparse.hpp"


int main(int argc, char** argv) {
    argparse::ArgumentParser parser("01_test_calibration");
    parser.add_argument("--imgDir", "-i").required().help("Path to the directory containing calibration images");
    parser.add_argument("--boardSize", "-b").required().help("Size of the checkerboard (rows,cols)");
    parser.add_argument("--squareSize", "-s").required().help("Size of each square in the checkerboard");
    parser.add_argument("--save", "-o").required().help("Path to save the calibrated parameters");
    parser.add_argument("--verbose", "-v").flag().help("Enable verbose output");
    try{
        parser.parse_args(argc, argv);
    } catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cerr << parser;
        return 1;
    }
    std::string imgDir = parser.get<std::string>("imgDir"); 
    std::string boardSizeString = parser.get<std::string>("boardSize");
    std::string squareSizeString = parser.get<std::string>("squareSize"); 
    std::string savePath = parser.get<std::string>("save");
    bool verbose = parser.get<bool>("verbose");

    cv::Size boardSize;
    auto pos = boardSizeString.find(',');
    boardSize.width = std::stoi(boardSizeString.substr(0, pos));
    boardSize.height = std::stoi(boardSizeString.substr(pos + 1));
    double squareSize = std::stod(squareSizeString);

    u3d::camera::CameraModel myCam;
    
    double rms = u3d::camera::calibratePinhole(imgDir, boardSize, squareSize, myCam, verbose);
    if (rms < 0) {
        std::cerr << "[Fail] Calibration failed! Check if the images are valid." << std::endl;
        return -1;
    }

    std::cout << "[Info] (RMS): " << rms << " pixels" << std::endl;
    
    // 评价标准
    if (rms < 0.2) std::cout << "[Info] Excellent calibration (RMS < 0.2)" << std::endl;
    else if (rms < 0.5) std::cout << "[Info] Good calibration (RMS < 0.5)" << std::endl;
    else if (rms < 1.0) std::cout << "[Info] Acceptable calibration (RMS < 1.0)" << std::endl;
    else std::cout << "[Info] Poor calibration (RMS >= 1.0), suggest to re-take images or check corner parameters." << std::endl;

    cv::FileStorage fs("calibrated_params.yaml", cv::FileStorage::WRITE);
    myCam.write(fs);
    std::cout << "[Info] Parameters have been saved to calibrated_params.yaml" << std::endl;
    return 0;
}
