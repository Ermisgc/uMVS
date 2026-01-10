#include "camera/camera_model.h"
#include <vector>
#include <filesystem>
NAMESPACE_BEGIN { namespace camera {
    Vec3d CameraModel::pixelToCamDir(const Point2d & uv) const {
        std::vector<Point2d> src = {uv};
        std::vector<Point2d> dst;

        //将像素点去畸变并转换为归一化平面(x, y, 1)
        cv::undistortPoints(src, dst, K, D);
        Vec3d dir = Vec3d(dst[0].x, dst[0].y, 1.0);
        return optics::normalize(dir);
    }

    std::vector<Vec3d> CameraModel::pixelToCamDir(const std::vector<Point2d> & uv) const {
        std::vector<Point2d> dst;
        cv::undistortPoints(uv, dst, K, D);
        std::vector<Vec3d> dirs;
        for (const auto & p : dst) {
            dirs.push_back(optics::normalize(Vec3d(p.x, p.y, 1.0)));
        }
        return dirs;
    }

    std::optional<Point2d> CameraModel::camToPixel(const Vec3d& Pc) const{
        double x = Pc[0] / Pc[2];
        double y = Pc[1] / Pc[2];
        double r2 = x * x + y * y;

        //应用畸变，这里采用的模型是Brown-Conrady畸变模型，因为它是OpenCV的默认畸变模型
        //详情参考https://en.wikipedia.org/wiki/Distortion_(optics)#Brown-Conrady_model (记得挂梯子访问)
        double k1 = D[0], k2 = D[1], p1 = D[2], p2 = D[3], k3 = D[4];
        double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;

        double x_dist = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        double y_dist = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

        //应用内参K投影到像素
        double u = K(0, 0) * x_dist + K(0, 2);
        double v = K(1, 1) * y_dist + K(1, 2);

        if(u < 0 || u >= imageSize.width || v < 0 || v >= imageSize.height) return std::nullopt; // 检查像素是否在图像范围内
        return cv::Point2d(u, v);
    }

    /**
     * @brief 把本对象序列化到文件中
     * @param fs 文件存储对象
     */
    void CameraModel::write(cv::FileStorage& fs) const {
        fs << "imageSize" << imageSize;
        fs << "K" << K;
        fs << "D" << D;
        fs << "R_w2c" << R_w2c;
        fs << "t_w2c" << t_w2c;
    }

    /**
     * @brief 从文件中读取本对象
     * @param fs 文件存储对象
     */
    void CameraModel::read(const cv::FileNode& fs) {
        fs["imageSize"] >> imageSize;
        fs["K"] >> K;
        fs["D"] >> D;
        fs["R_w2c"] >> R_w2c;
        fs["t_w2c"] >> t_w2c;
        K_inv = K.inv();
    }

    void CameraModel::print() const {
        std::cout << "imageSize: " << imageSize << std::endl;
        std::cout << "K: " << K << std::endl;
        std::cout << "D: " << D << std::endl;
        std::cout << "R_w2c: " << R_w2c << std::endl;
        std::cout << "t_w2c: " << t_w2c << std::endl;
    }

    double calibratePinhole(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, CameraModel& camera_model, bool verbose){
        //定义世界坐标系下的三维点，这些三维点可以位于世界坐标系的任何一个位置
        //只要棋盘格的参数正确即可，因为这里的标定功能仅限于对相机内参进行标定
        //这里假设所有的棋盘格角点都在世界坐标系的z=0平面上
        std::vector<cv::Point3f> ObjP;
        for(int i = 0;i < boardSize.height;i++){
            for(int j = 0;j < boardSize.width;j++){
                ObjP.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0));
            }
        }

        int successCount = 0;
        cv::Size imgSize;
        std::vector<std::vector<cv::Point2f>> imagePoints;
        std::vector<std::vector<cv::Point3f>> objectPoints;

        for(const auto & file: imageFiles) {
            cv::Mat img = cv::imread(file);
            if(img.empty()){
                std::cerr << "[Error] Cannot read image " << file << std::endl;
                continue;
            }

            if(successCount == 0){
                imgSize = img.size();
            } else if (img.size() != imgSize){
                std::cerr << "[Error]: Image " << file << " has different size than the first image." << std::endl;
                continue;
            }

            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

            //调用函数寻找棋盘格角点
            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
            if(!found){
                std::cerr << "[Error] Cannot find chessboard corners in image " << file << std::endl;
                continue;
            }

            //亚像素级精确化
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            
            //将当前图像的角点和对应的三维点添加到集合中
            imagePoints.push_back(corners);
            objectPoints.push_back(ObjP);

            successCount++;

            if(verbose){
                cv::drawChessboardCorners(img, boardSize, corners, found);
                cv::imshow("Chessboard Corners", img);
                cv::waitKey(500);
            }
        }

        if(successCount < 5){
            std::cerr << "[Error] Only " << successCount << " images with valid chessboard corners. Calibration failed." << std::endl;
            return -1.0;
        }

        std::cout << "[Info] Calculate camera model with " << successCount << " images." << std::endl;

        cv::Mat K, D;
        std::vector<cv::Mat> rvecs, tvecs;

        K = cv::initCameraMatrix2D(objectPoints, imagePoints, imgSize, 0);

        double rms = cv::calibrateCamera(
            objectPoints,
            imagePoints,
            imgSize,
            K,
            D,
            rvecs,
            tvecs,
            cv::CALIB_FIX_ASPECT_RATIO
        );

        //填充结果
        camera_model.K = K.clone();
        camera_model.D = D.clone();
        camera_model.imageSize = imgSize;

        std::cout << "[Info] Camera model calibrated with RMS error: " << rms << std::endl;
        if(verbose){
            camera_model.print();
        }
        return rms;
    }

    double calibratePinhole(const std::string & imagePath, cv::Size boardSize, double squareSize, CameraModel& camera_model, bool verbose){
        //从imagePath中读取所有的图片
        std::vector<std::string> imageFiles;
        for(const auto & entry : std::filesystem::directory_iterator(imagePath)){
            auto extension = entry.path().extension().string();
            for(auto & c: extension) c = std::tolower(c); //小写

            if(extension == ".jpg" || extension == ".png" || extension == ".jpeg" || extension == ".bmp"){
                imageFiles.push_back(entry.path().string());
            }
        }
        return calibratePinhole(imageFiles, boardSize, squareSize, camera_model, verbose);
    }

    std::ostream& operator<<(std::ostream& os, const CameraModel& camera){
        os << "CameraModel:" << std::endl;
        os << "K:" << std::endl << camera.K << std::endl;
        os << "D:" << std::endl << camera.D << std::endl;
        os << "imageSize:" << camera.imageSize << std::endl;
        return os;
    }
}}  // namespace camera
