#include "camera/binouwcamera.h"
NAMESPACE_BEGIN { namespace camera{
    std::vector<Point2d> BinoUWCamera::computeEpipolarCurve(const Vec2d& left_pixel, const std::vector<double>& depths) const {
        std::vector<Point2d> curve_points;
        curve_points.reserve(depths.size());

        for (double z : depths) {
            auto world_point_opt = left_camera_.backwardProject(left_pixel, z);

            if (!world_point_opt.has_value()) {
                continue; // 该深度下无法反投影（可能超出物理限制）
            }

            auto right_pixel_opt = right_camera_.forwardProject(world_point_opt.value(), ForwardMethod::ITERATE);

            if (right_pixel_opt.has_value()) {
                curve_points.push_back(right_pixel_opt.value());
            }
        }
        return curve_points;
    }

    std::vector<Point2d> BinoUWCamera::computeEpipolarCurve(const Vec2d& left_pixel, double min_depth, double max_depth, int steps) const {
        if (min_depth <= 1e-6 || max_depth <= min_depth || steps <= 0) {
            return {};
        }

        std::vector<double> depths;
        depths.reserve(steps);

        // 采用逆深度均匀采样： 1/d 线性分布
        // 这样近处的点会密一些，远处的点稀疏一些，符合透视投影的特性，画出的曲线更平滑
        double inv_min = 1.0 / max_depth;
        double inv_max = 1.0 / min_depth;
        double step_size = (inv_max - inv_min) / static_cast<double>(steps - 1);

        for (int i = 0; i < steps; ++i) {
            double inv_d = inv_max - i * step_size;
            if (inv_d > 1e-9) {
                depths.push_back(1.0 / inv_d);
            }
        }

        return computeEpipolarCurve(left_pixel, depths);
    }

    double BinoUWCamera::computeVerticalDisparity(const Vec2d& left_pixel_rect, double depth) const {
        // 注意：此函数假设传入的 left_pixel_rect 已经是通过某种单应性变换校正过的坐标
        // 或者是假设如果完美校正，left_pixel_rect.y 应该等于 right_pixel.y
        // 如果是在原始未校正图像上操作，这个函数意义不大，需要结合 StereoRectifier 使用。
        
        // 这里仅提供基础逻辑：计算投影点的 y 坐标差
        // 实际使用需确保 left_camera_ 和 right_camera_ 已经被“旋转”到了校正姿态
        
        auto world_pt = left_camera_.backwardProject(left_pixel_rect, depth);
        if(world_pt) {
            auto right_pt = right_camera_.forwardProject(*world_pt, ForwardMethod::ITERATE);
            if(right_pt) {
                return right_pt->y - left_pixel_rect[1];
            }
        }
        return 0.0;
    }

    void BinoUWCamera::write(cv::FileStorage& fs) const {
        fs << "left_camera" << "{";
        left_camera_.write(fs);
        fs << "}";
        fs << "right_camera" << "{";
        right_camera_.write(fs);
        fs << "}";
    }

    void BinoUWCamera::write(const std::string& filename) const {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        write(fs);
    }

    void BinoUWCamera::read(const cv::FileNode& fs) {
        fs["left_camera"] >> left_camera_;
        fs["right_camera"] >> right_camera_;
        initRemap(left_camera_, right_camera_);
    }

    void BinoUWCamera::read(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        read(fs.root());
    }

    std::pair<cv::Mat, cv::Mat> BinoUWCamera::rectify(const cv::Mat& left_img, const cv::Mat& right_img) const {
        if(left_img.empty() || right_img.empty()){
            throw std::invalid_argument("BinocularCamera::rectify: image is empty");
        }
        if(left_img.size() != imgSize || right_img.size() != imgSize){
            throw std::invalid_argument("BinocularCamera::rectify: image size not match");
        }

        cv::Mat left_rectified, right_rectified;
        cv::remap(left_img, left_rectified, left_map_x_, left_map_y_, cv::INTER_LINEAR);
        cv::remap(right_img, right_rectified, right_map_x_, right_map_y_, cv::INTER_LINEAR);
        return {left_rectified, right_rectified};
    }

    void BinoUWCamera::initRemap(const MonocularUWCamera& left_camera, const MonocularUWCamera& right_camera){
        using namespace NAMESPACE_U3D::optics;
        auto left_pinhole = left_camera.pinhole();
        auto right_pinhole = right_camera.pinhole();

        //初始化相机参数
        this->imgSize = left_pinhole.imageSize;
        int width = left_pinhole.imageSize.width;
        int height = left_pinhole.imageSize.height;

        Matx33d K_l = left_pinhole.K, K_r = right_pinhole.K;
        Vec5d D_l = left_pinhole.D, D_r = right_pinhole.D;
        Size imgSize = left_pinhole.imageSize;
        Matx33d R_l = left_pinhole.R_w2c, R_r = right_pinhole.R_w2c;
        Vec3d t_l = left_pinhole.t_w2c, t_r = right_pinhole.t_w2c;
        //计算相对位姿
        // this->R_l2r = R_l * R_r.t();
        // this->t_l2r = R_l * t_r - t_l;
        this->R_l2r = R_r * R_l.t();
        this->t_l2r = t_r - this->R_l2r * t_l;

        //准备数据
        Vec3d glass_normal_left_ = left_camera.glass_normal_;
        Matx33d R_norm_L = alignAxisToZ(glass_normal_left_);
        Matx33d R_norm_R = alignAxisToZ(right_camera.glass_normal_);
        Matx33d R_rel_norm = R_norm_R * this->R_l2r * R_norm_L.t();
        Vec3d T_rel_norm = R_norm_R * this->t_l2r;

        double scale_l = left_camera_.n_water_  / left_camera_.n_air_;
        Matx33d k_l_water = K_l;
        k_l_water(0, 0) *= scale_l;
        k_l_water(1, 1) *= scale_l;

        double scale_r = right_camera_.n_water_  / right_camera_.n_air_;
        Matx33d k_r_water = K_r;
        k_r_water(0, 0) *= scale_r;
        k_r_water(1, 1) *= scale_r;

        cv::Mat R_rect_L, R_rect_R, P_L, P_R, Q;
        cv::stereoRectify(k_l_water, cv::Mat::zeros(5, 1, CV_64F), 
                          k_r_water, cv::Mat::zeros(5, 1, CV_64F), 
                          imgSize, 
                          R_rel_norm, T_rel_norm, 
                          R_rect_L, R_rect_R, 
                          P_L, P_R, 
                          Q, cv::CALIB_ZERO_DISPARITY, 0, imgSize);

        double f_v = P_L.at<double>(0, 0);
        double cx_v = P_L.at<double>(0, 2);
        double cy_v = P_L.at<double>(1, 2);
        double baseline = -P_R.at<double>(0, 3) / f_v; 

        //这里面的map，对应的是采样LUT表，对于新图中的任一像素u，v，寻找它在原图中的位置
        cv::Mat map_x_l(height, width, CV_32FC1);
        cv::Mat map_y_l(height, width, CV_32FC1);
        cv::Mat map_x_r(height, width, CV_32FC1);
        cv::Mat map_y_r(height, width, CV_32FC1);

        for (int v_f = 0; v_f < height; ++v_f) {
            for (int u_f = 0; u_f < width; ++u_f) {
                // 此时该向量代表在“水中”的射线方向，因为 RAL 定义在水下角度空间
                double dx_f = (u_f - cx_v) / f_v;
                double dy_f = (v_f - cy_v) / f_v;
                double dz_f = 1.0;
                
                Vec3d d_rect(dx_f, dy_f, dz_f);
                double norm = cv::norm(d_rect);
                if(norm > 1e-9){
                    d_rect /= norm;
                }

                // 逆向旋转：从校正后的空间转回“折射轴归一化”后的相机空间
                // 此时 Z 轴依然平行于玻璃法线
                Vec3d d_norm_w = cv::Matx33d(R_rect_L).t() * d_rect;

                // 逆向折射计算 (从水到空气)
                // n_w * sin(theta_w) = n_a * sin(theta_a)
                double nw = left_camera.n_water_;
                double na = left_camera.n_air_;
                
                // 在归一化平面，sin(theta) 与 xy 分量成正比
                double nx = d_norm_w[0];
                double ny = d_norm_w[1];
                
                double scale = nw / na;
                double dx_a = nx * scale;
                double dy_a = ny * scale;
                
                // 检查全反射
                double sin2_a = dx_a*dx_a + dy_a*dy_a;
                if (sin2_a >= 1.0) {
                    map_x_l.at<float>(v_f, u_f) = -1; // 标记无效
                    continue;
                }
                double dz_a = std::sqrt(1.0 - sin2_a);
                Vec3d d_norm_a(dx_a, dy_a, dz_a);

                // 逆向轴归一化：转回原始相机物理坐标系
                Vec3d d_phys_a = R_norm_L.t() * d_norm_a;

                // 投影到原始传感器：包含镜头畸变
                auto pixel_raw = left_camera.pinhole().camToPixel(d_phys_a); 
                if(!pixel_raw.has_value()){
                    map_x_l.at<float>(v_f, u_f) = -1; // 标记无效
                    continue;
                }
                
                map_x_l.at<float>(v_f, u_f) = pixel_raw.value().x;
                map_y_l.at<float>(v_f, u_f) = pixel_raw.value().y;
            }
        }        

        for (int v_f = 0; v_f < height; ++v_f) {
            for (int u_f = 0; u_f < width; ++u_f) {
                // 此时该向量代表在“水中”的射线方向，因为 RAL 定义在水下角度空间
                double dx_f = (u_f - cx_v) / f_v;
                double dy_f = (v_f - cy_v) / f_v;
                double dz_f = 1.0;
                
                Vec3d d_rect(dx_f, dy_f, dz_f);
                double norm = cv::norm(d_rect);
                if(norm > 1e-9){
                    d_rect /= norm;
                }

                // 逆向旋转：从校正后的空间转回“折射轴归一化”后的相机空间
                // 此时 Z 轴依然平行于玻璃法线
                Vec3d d_norm_w = cv::Matx33d(R_rect_R).t() * d_rect;

                // 逆向折射计算 (从水到空气)
                // n_w * sin(theta_w) = n_a * sin(theta_a)
                double nw = right_camera.n_water_;
                double na = right_camera.n_air_;
                
                // 在归一化平面，sin(theta) 与 xy 分量成正比
                double nx = d_norm_w[0];
                double ny = d_norm_w[1];
                
                double scale = nw / na;
                double dx_a = nx * scale;
                double dy_a = ny * scale;
                
                // 检查全反射
                double sin2_a = dx_a*dx_a + dy_a*dy_a;
                if (sin2_a >= 1.0) {
                    map_x_r.at<float>(v_f, u_f) = -1; // 标记无效
                    continue;
                }
                double dz_a = std::sqrt(1.0 - sin2_a);
                Vec3d d_norm_a(dx_a, dy_a, dz_a);

                // 逆向轴归一化：转回原始相机物理坐标系
                Vec3d d_phys_a = R_norm_R.t() * d_norm_a;
                // 投影到原始传感器：包含镜头畸变
                auto pixel_raw = right_camera.pinhole().camToPixel(d_phys_a); 
                if(!pixel_raw.has_value()){
                    map_x_r.at<float>(v_f, u_f) = -1; // 标记无效
                    continue;
                }
                
                map_x_r.at<float>(v_f, u_f) = pixel_raw.value().x;
                map_y_r.at<float>(v_f, u_f) = pixel_raw.value().y;
            }
        }    
        
        this->left_map_x_ = map_x_l;
        this->left_map_y_ = map_y_l;
        this->right_map_x_ = map_x_r;
        this->right_map_y_ = map_y_r;
    }

    std::ostream& operator<<(std::ostream& os, const BinoUWCamera& camera){
        os << "BinoUWCamera:" << std::endl;
        os << "left_camera:" << std::endl << camera.left_camera_ << std::endl;
        os << "right_camera:" << std::endl << camera.right_camera_ << std::endl;
        return os;
    }
}}