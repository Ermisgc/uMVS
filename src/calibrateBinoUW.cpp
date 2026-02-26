/**
 * @file calibrateBinoUW.cpp
 * @author ermis
 * @date 2026-2-26
 * @brief 水下双目相机标定，输入左右相机同时观测到的棋盘格图像所在的目录，以及已经标定好了的空气中的双目相机模型
 * @example 使用示例为
 * @code
 * ./bin/Release/calibrateBinoUW --leftImage ./CbtCameraL --rightImage ./CbtCameraR --boardSize 11,8 --squareSize 15.0 
 * --binoModel calibrated_params.yaml --save calibrated_paramsUW.yaml --verbose
 * @endcode
 */

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "camera/binouwcamera.h" 
#include "argparse/argparse.hpp"

using namespace u3d::camera;

int main(int argc, char** argv) {
    argparse::ArgumentParser parser("calibrateBino");
    parser.add_argument("--leftImage", "-l").required().help("Path to the directory containing left calibration images");
    parser.add_argument("--rightImage", "-r").required().help("Path to the directory containing right calibration images");
    parser.add_argument("--boardSize", "-b").required().help("Size of the checkerboard (rows,cols)");
    parser.add_argument("--squareSize", "-s").required().help("Size of each square in the checkerboard");
    parser.add_argument("--save", "-o").default_value("calibrated_params.yaml").help("Path to save the calibrated parameters");
    parser.add_argument("--binoModel", "-bm").required().help("Path to the bino camera model file");
    parser.add_argument("--verbose", "-v").flag().help("Enable verbose output");
    try{
        parser.parse_args(argc, argv);
    } catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cerr << parser;
        return 1;
    }
    std::string leftImage = parser.get<std::string>("--leftImage"); 
    std::string rightImage = parser.get<std::string>("--rightImage");
    std::string boardSizeString = parser.get<std::string>("--boardSize");
    std::string squareSizeString = parser.get<std::string>("--squareSize"); 
    std::string binoModelPath = parser.get<std::string>("--binoModel");
    std::string savePath = parser.get<std::string>("--save");
    bool verbose = parser.is_used("--verbose");

    cv::Size boardSize;
    auto pos = boardSizeString.find(',');
    boardSize.width = std::stoi(boardSizeString.substr(0, pos));
    boardSize.height = std::stoi(boardSizeString.substr(pos + 1));
    double squareSize = std::stod(squareSizeString);

    std::cout << "[Info] Left image directory: " << leftImage << std::endl;
    std::cout << "[Info] Right image directory: " << rightImage << std::endl;
    std::cout << "[Info] Board size: " << boardSize.width << " x " << boardSize.height << std::endl;
    std::cout << "[Info] Square size: " << squareSize << std::endl;
    std::cout << "[Info] Save path: " << savePath << std::endl;
    std::cout << "[Info] Verbose: " << verbose << std::endl;
    std::cout << "[Info] Bino camera model file: " << binoModelPath << std::endl;

    BinoCamera binoCamera;
    binoCamera.read(binoModelPath);

    double rms;
    BinoUWCamera binoUWCam = BinoUWCamera::calibrate(leftImage, rightImage, boardSize, squareSize, binoCamera, &rms, verbose);
    std::cout << "[Info] RMS: " << rms << std::endl;
    binoUWCam.write(savePath);
    std::cout << "[Info] Calibrated parameters saved to: " << savePath << std::endl;
    return 0;
}