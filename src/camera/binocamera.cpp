#include "camera/binocamera.h"
NAMESPACE_BEGIN{namespace camera{
    BinocularCamera::BinocularCamera(const CameraModel& left_camera, const CameraModel& right_camera)
    : left_camera(left_camera), right_camera(right_camera){
        initRemap(left_camera, right_camera);
    }

    std::pair<cv::Mat, cv::Mat> BinocularCamera::rectify(const cv::Mat& left_img, const cv::Mat& right_img){
        if(!is_init){
            throw std::runtime_error("BinocularCamera::rectify: not init");
        }
        if(left_img.empty() || right_img.empty()){
            throw std::invalid_argument("BinocularCamera::rectify: image is empty");
        }
        if(left_img.size() != imgSize || right_img.size() != imgSize){
            throw std::invalid_argument("BinocularCamera::rectify: image size not match");
        }

        cv::Mat rectified_left_img, rectified_right_img;
        cv::remap(left_img, rectified_left_img, mapl1, mapl2, cv::INTER_LINEAR);
        cv::remap(right_img, rectified_right_img, mapr1, mapr2, cv::INTER_LINEAR);

        return std::make_pair(rectified_left_img, rectified_right_img);
    }

    void BinocularCamera::write(cv::FileStorage& fs) const {
        fs << "left_camera" << "{";
        left_camera.write(fs);
        fs << "}";
        fs << "right_camera" << "{";
        right_camera.write(fs);
        fs << "}";
    }

    void BinocularCamera::read(const cv::FileNode& fs) {
        fs["left_camera"] >> left_camera;
        fs["right_camera"] >> right_camera;
        initRemap(left_camera, right_camera);
    }

    std::ostream& operator<<(std::ostream& os, const BinocularCamera& camera){
        os << "BinocularCamera:" << std::endl;
        os << "left_camera:" << std::endl << camera.left_camera << std::endl;
        os << "right_camera:" << std::endl << camera.right_camera << std::endl;
        return os;
    }

    void BinocularCamera::initRemap(const CameraModel& left_camera, const CameraModel& right_camera){
        this->imgSize = left_camera.imageSize;
        //根据已经标定好的双目相机模型，求校正时remap，采用Bongeut方法
        Matx33d K_l = left_camera.K, K_r = right_camera.K;
        Vec5d D_l = left_camera.D, D_r = right_camera.D;
        Size imgSize = left_camera.imageSize;
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
}}