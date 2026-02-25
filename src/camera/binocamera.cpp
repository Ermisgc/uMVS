#include "camera/binocamera.h"
NAMESPACE_BEGIN{namespace camera{
    BinoCamera::BinoCamera(const MonoCamera& left_camera, const MonoCamera& right_camera)
    : left_camera(left_camera), right_camera(right_camera){
        initRemap(left_camera, right_camera);
    }

    std::pair<cv::Mat, cv::Mat> BinoCamera::rectify(const cv::Mat& left_img, const cv::Mat& right_img){
        if(!is_init){
            throw std::runtime_error("BinoCamera::rectify: not init");
        }
        if(left_img.empty() || right_img.empty()){
            throw std::invalid_argument("BinoCamera::rectify: image is empty");
        }
        if(left_img.size() != imgSize || right_img.size() != imgSize){
            throw std::invalid_argument("BinoCamera::rectify: image size not match");
        }

        cv::Mat rectified_left_img, rectified_right_img;
        cv::remap(left_img, rectified_left_img, mapl1, mapl2, cv::INTER_LINEAR);
        cv::remap(right_img, rectified_right_img, mapr1, mapr2, cv::INTER_LINEAR);

        return std::make_pair(rectified_left_img, rectified_right_img);
    }

    void BinoCamera::write(cv::FileStorage& fs) const {
        fs << "left_camera" << "{";
        left_camera.write(fs);
        fs << "}";
        fs << "right_camera" << "{";
        right_camera.write(fs);
        fs << "}";
    }

    void BinoCamera::write(const std::string& filename) const {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if(!fs.isOpened()){
            throw std::runtime_error(filename + " file not opened or existed");
        }
        write(fs);
        fs.release();
    }

    void BinoCamera::read(const cv::FileNode& fs) {
        fs["left_camera"] >> left_camera;
        fs["right_camera"] >> right_camera;
        initRemap(left_camera, right_camera);
    }

    void BinoCamera::read(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if(!fs.isOpened()){
            throw std::runtime_error(filename + " file not opened or exist");
        }
        read(fs.root());
        fs.release();
    }

    std::ostream& operator<<(std::ostream& os, const BinoCamera& camera){
        os << "BinoCamera:" << std::endl;
        os << "left_camera:" << std::endl << camera.left_camera << std::endl;
        os << "right_camera:" << std::endl << camera.right_camera << std::endl;
        return os;
    }

    void BinoCamera::initRemap(const MonoCamera& left_camera, const MonoCamera& right_camera){
        this->imgSize = left_camera.imageSize;
        //根据已经标定好的双目相机模型，求校正时remap，采用Bongeut方法
        Matx33d K_l = left_camera.K, K_r = right_camera.K;
        Vec5d D_l = left_camera.D, D_r = right_camera.D;
        cv::Size imgSize = left_camera.imageSize;
        Matx33d R_l = left_camera.R_w2c, R_r = right_camera.R_w2c;
        Vec3d t_l = left_camera.t_w2c, t_r = right_camera.t_w2c;
        
        //计算相对位姿
        this->R_l2r = R_l * R_r.t();
        this->t_l2r = t_l - this->R_l2r * t_r;

        //使用Bongeut方法求左图的rectify矩阵R1，P1，右图的rectify矩阵R2，P2
        cv::Mat R1, R2, P1, P2;
        cv::stereoRectify(K_l, D_l, K_r, D_r, imgSize, R_l2r, t_l2r, R1, R2, P1, P2, cv::noArray(), cv::CALIB_ZERO_DISPARITY);
        
        cv::Mat mapl1, mapl2, mapr1, mapr2;
        cv::initUndistortRectifyMap(K_l, D_l, R1, P1, imgSize, CV_16SC2, mapl1, mapl2);
        cv::initUndistortRectifyMap(K_r, D_r, R2, P2, imgSize, CV_16SC2, mapr1, mapr2);

        this->mapl1 = mapl1;
        this->mapl2 = mapl2;
        this->mapr1 = mapr1;
        this->mapr2 = mapr2;

        this->is_init = true;
    }

    BinoCamera BinoCamera::calibrate(const std::string& left_imagePath,
                                const std::string& right_imagePath,
                                cv::Size boardSize, 
                                double squareSize, 
                                const MonoCamera * left_camera, 
                                const MonoCamera * right_camera, 
                                double * out_rms,
                                bool verbose)
    {
        std::vector<cv::String> left_images, right_images;
        if(left_imagePath.empty() || right_imagePath.empty()) throw std::invalid_argument("BinoCamera::calibrate: image path is empty");
        if(left_imagePath[left_imagePath.size() - 1] != '/') cv::glob(left_imagePath + "/*.bmp", left_images);
        else cv::glob(left_imagePath + "*.bmp", left_images);
        if(right_imagePath[right_imagePath.size() - 1] != '/') cv::glob(right_imagePath + "/*.bmp", right_images);
        else cv::glob(right_imagePath + "*.bmp", right_images);

        //排序，避免因为某些操作系统的glob实现不同导致左右图顺序不同
        std::sort(left_images.begin(), left_images.end());
        std::sort(right_images.begin(), right_images.end());

        if(left_images.size() != right_images.size() || left_images.empty()) {
            throw std::invalid_argument("BinoCamera::calibrate: left image size not match right image size or empty");
        }
        
        std::vector<std::vector<cv::Point3f>> objectPoints; 
        std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;     

        std::vector<std::vector<cv::Point2f>> imgPointsLeftAll, imgPointsRightAll;  //为可能存在的单目标定需求准备的  
        std::vector<std::vector<cv::Point3f>> objPointsLeftAll, objPointsRightAll;
  

        std::vector<cv::Point3f> objP;
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                objP.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
            }
        }      
        
        cv::Size currentImgSize;
        if (verbose) std::cout << "开始提取棋盘格角点，共 " << left_images.size() << " 对..." << std::endl;

        int findFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
        cv::TermCriteria subPixCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001);
        int successCount = 0;
        //角点提取
        for(size_t i = 0;i < left_images.size();++i){
            cv::Mat left_img = cv::imread(left_images[i], cv::IMREAD_GRAYSCALE);
            cv::Mat right_img = cv::imread(right_images[i], cv::IMREAD_GRAYSCALE);
            if(left_img.empty() || right_img.empty()){
                if(verbose) std::cerr << "[Error] Failed to read image pair " << i << std::endl;
                continue;
            }

            if(i == 0) currentImgSize = left_img.size();
            std::vector<cv::Point2f> cornersL, cornersR;
            bool foundL = cv::findChessboardCorners(left_img, boardSize, cornersL, findFlags);
            bool foundR = cv::findChessboardCorners(right_img, boardSize, cornersR, findFlags);

            if(foundL){
                cv::cornerSubPix(left_img, cornersL, cv::Size(11, 11), cv::Size(-1, -1), subPixCriteria);
                imgPointsLeftAll.push_back(cornersL);
                objPointsLeftAll.push_back(objP);
            }

            if(foundR){
                cv::cornerSubPix(right_img, cornersR, cv::Size(11, 11), cv::Size(-1, -1), subPixCriteria);
                imgPointsRightAll.push_back(cornersR);
                objPointsRightAll.push_back(objP);
            }

            if(foundL && foundR){
                imagePointsLeft.push_back(cornersL);
                imagePointsRight.push_back(cornersR);
                objectPoints.push_back(objP);
                successCount++;
            } else {
                if(verbose) std::cerr << "[Warning] Failed to find chessboard corners in image pair " << i << std::endl;
            }
        }
        if(verbose) std::cout << "成功提取棋盘格角点 " << successCount << " 对" << std::endl;

        //进行单目判断，如果左右相机都没有标定，那么这里的双目标定就需利用上述标定板提前标定出来单目相机
        cv::Mat K1, D1, K2, D2;
        MonoCamera final_left_cam, final_right_cam;

        if (left_camera != nullptr) {
            final_left_cam = *left_camera;
            K1 = cv::Mat(final_left_cam.K);   
            D1 = cv::Mat(final_left_cam.D);
            if (verbose) std::cout << "左相机：使用外部提供的单目模型" << std::endl;
        } else {
            if (verbose) std::cout << "左相机：未提供模型，正在根据图像执行内部单目标定..." << std::endl;
            std::vector<cv::Mat> rvecs, tvecs;
            double rms_L = cv::calibrateCamera(objPointsLeftAll, imgPointsLeftAll, currentImgSize, K1, D1, rvecs, tvecs);
            final_left_cam.K = K1.clone();
            final_left_cam.D = D1.clone();
            final_left_cam.imageSize = currentImgSize;
            if (verbose) std::cout << "  左单目标定 RMS: " << rms_L << std::endl;
        }

        if (right_camera != nullptr) {
            final_right_cam = *right_camera;
            K2 = cv::Mat(final_right_cam.K);
            D2 = cv::Mat(final_right_cam.D);
            if (verbose) std::cout << "右相机：使用外部提供的单目模型" << std::endl;
        } else {
            if (verbose) std::cout << "右相机：未提供模型，正在根据图像执行内部单目标定..." << std::endl;
            std::vector<cv::Mat> rvecs, tvecs;
            double rms_R = cv::calibrateCamera(objPointsRightAll, imgPointsRightAll, currentImgSize, K2, D2, rvecs, tvecs);
            final_right_cam.K = K2.clone();
            final_right_cam.D = D2.clone();
            final_right_cam.imageSize = currentImgSize;
            if (verbose) std::cout << "  右单目标定 RMS: " << rms_R << std::endl;
        }

        if(verbose) std::cout << "进行双目立体校正联合优化..." << std::endl;
        cv::Mat R, T, E, F;
        int calibFlags = cv::CALIB_FIX_INTRINSIC;
        cv::TermCriteria calibCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-5);

        double rms = cv::stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
                                         K1, D1, K2, D2, currentImgSize,
                                         R, T, E, F,
                                         calibFlags, calibCriteria);

        if(verbose) std::cout << "  双目标定完成，重投影误差 RMS: " << rms << std::endl;
        if(out_rms != nullptr) *out_rms = rms;

        final_left_cam.R_w2c = cv::Matx33d::eye();
        final_left_cam.t_w2c = {0, 0, 0};
        final_right_cam.R_w2c = R.clone();
        final_right_cam.t_w2c = T.clone();
        
        BinoCamera bino_cam(final_left_cam, final_right_cam);
        return bino_cam;
    }
}}