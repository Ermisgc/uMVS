/**
 * @file 04_test_uw_epipolar.cpp
 * @brief 用于示例的代码，作用是计算水下双目相机的极线，这里仅限于计算右图像的极线
 * @author Ermis
 * @date 2026-1-10
 * @example ./bin/Release/04_test_uw_epipolar -m uw_camera_model.yml -r right.png -s -ro right_epi.png
 */
#include "camera/binouwcamera.h"
#include "argparse/argparse.hpp"

int main(int argc, char *argv[]){
    //参数解析
    argparse::ArgumentParser program("04_test_uw_epipolar");
    program.add_argument("--model", "-m").required().help("Path to the uw binocular camera model file");
    program.add_argument("--left", "-l").required().help("Path to the left image file");
    program.add_argument("--right", "-r").required().help("Path to the right image file");
    program.add_argument("--show", "-s").flag().default_value(true).help("Show the rectified images");
    program.add_argument("--min_depth", "-min").default_value(300.0).help("Minimum depth of the image");
    program.add_argument("--max_depth", "-max").default_value(50000.0).help("Maximum depth of the image");
    program.add_argument("--steps", "-st").default_value(100).help("Number of points to sample in an epipolar line");
    program.add_argument("--output", "-o").help("Path to the output image file");
    program.add_argument("--grid_rows", "-gr").default_value(7).help("Sampling steps for the grid rows");
    program.add_argument("--grid_cols", "-gc").default_value(10).help("Sampling steps for the grid columns");

    try{
        program.parse_args(argc, argv);
    } catch(const std::exception& e){
        std::cerr << e.what() << '\n';
        std::cerr << program;
        return 1;
    }

    std::string modelPath = program.get<std::string>("--model");
    std::string rightPath = program.get<std::string>("--right");
    std::string leftPath = program.get<std::string>("--left");
    double minDepth = program.get<double>("--min_depth");
    double maxDepth = program.get<double>("--max_depth");
    int steps = program.get<int>("--steps");
    int gridRows = program.get<int>("--grid_rows");
    int gridCols = program.get<int>("--grid_cols");
    std::cout << "Args:" << std::endl;
    std::cout << ">>> modelPath: " << modelPath << std::endl;
    std::cout << ">>> rightPath: " << rightPath << std::endl;
    std::cout << ">>> leftPath: " << leftPath << std::endl;
    std::cout << ">>> minDepth: " << minDepth << std::endl;
    std::cout << ">>> maxDepth: " << maxDepth << std::endl;
    std::cout << ">>> steps: " << steps << std::endl;

    //加载相机模型
    NAMESPACE_U3D::camera::BinoUWCamera camera;
    cv::FileStorage fs(modelPath, cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cerr << "Failed to open the model file: " << modelPath << std::endl;
        return 1;
    }
    camera.read(fs.root());
    fs.release();

    //加载图像
    cv::Mat leftImg = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    if(leftImg.empty()){
        std::cerr << "Failed to load the image file: " << leftPath << std::endl;
        return 1;
    }
    cv::Mat rightImg = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
    if(rightImg.empty()){
        std::cerr << "Failed to load the image file: " << rightPath << std::endl;
        return 1;
    }

    //在左图中均匀采样100个点
    std::vector<cv::Point2d> leftPoints;
    int stepX = leftImg.cols / (gridCols - 1);
    int stepY = leftImg.rows / (gridRows - 1);
    for(int r = 1; r <= gridRows; r++){
        for(int c = 1; c <= gridCols; c++){
            leftPoints.push_back(cv::Point2d(c * stepX, r * stepY));
        }
    }

    cv::Mat canvasL = leftImg.clone();
    cv::Mat canvasR = rightImg.clone();
    cv::cvtColor(canvasL, canvasL, cv::COLOR_GRAY2BGR);
    cv::cvtColor(canvasR, canvasR, cv::COLOR_GRAY2BGR);

    int ptIdx = 0;
    //计算右图中对应的极线
    for(const auto & point: leftPoints){
        cv::Scalar color = NAMESPACE_U3D::randomColor(ptIdx++);  //随机获取一个颜色

        //左图：画点和十字标记
        cv::circle(canvasL, point, 10, color, 4);
        cv::drawMarker(canvasL, point, color, cv::MARKER_CROSS, 20, 4);

        //计算对极曲线
        std::vector<cv::Point2d> curvePts = camera.computeEpipolarCurve(point, minDepth, maxDepth, steps);

        // --- 右图：画平滑曲线 ---
        if (curvePts.size() > 1) {
            std::vector<cv::Point> pts_cv;
            pts_cv.reserve(curvePts.size());
            for (const auto& p : curvePts) {
                pts_cv.emplace_back(std::round(p.x), std::round(p.y));
            }

            const cv::Point* ppt[1] = { pts_cv.data() };
            int npt[] = { static_cast<int>(pts_cv.size()) };
            cv::polylines(canvasR, ppt, npt, 1, false, color, 4, cv::LINE_AA);
            
            // 在曲线起点画一个小点表示方向
            cv::circle(canvasR, pts_cv.front(), 3, color, -1);
        }
    }

    // 拼接左右图像
    cv::Mat convasCombined;
    cv::hconcat(canvasL, canvasR, convasCombined);
    cv::resize(convasCombined, convasCombined, cv::Size(), 0.25, 0.25);

    if(program.get<bool>("--show")){
        cv::imshow("Epipolar Lines", convasCombined);
        cv::waitKey(0);
    }
    if(program.is_used("--output")){
        cv::imwrite(program.get<std::string>("--output"), convasCombined);
        std::cout << "Save result to: " << program.get<std::string>("--output") << std::endl;
    }

    return 0;
}