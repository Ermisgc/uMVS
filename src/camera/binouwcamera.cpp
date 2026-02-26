#include "camera/binouwcamera.h"
#include "optics/nsga2.h"
#include "optics/geometry.h"
#include <fstream>
#include <iomanip>
NAMESPACE_BEGIN { namespace camera{
    BinoUWCamera::BinoUWCamera(const MonoUWCamera& left, const MonoUWCamera& right, bool init) 
        : left_camera_(left), right_camera_(right) 
    {
        if(init) initRemap(left, right);
        else{
            const auto & left_pinhole = left.pinhole();
            const auto & right_pinhole = right.pinhole();
            Matx33d R_l = left_pinhole.R_w2c, R_r = right_pinhole.R_w2c;
            Vec3d t_l = left_pinhole.t_w2c, t_r = right_pinhole.t_w2c;
            this->R_l2r = R_r * R_l.t();
            this->t_l2r = t_r - this->R_l2r * t_l;            
        }
    }

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
        if(!is_init){
            throw std::runtime_error("BinocularCamera::rectify: camera not initialized");
        }

        cv::Mat left_rectified, right_rectified;
        cv::remap(left_img, left_rectified, left_map_x_, left_map_y_, cv::INTER_LINEAR);
        cv::remap(right_img, right_rectified, right_map_x_, right_map_y_, cv::INTER_LINEAR);
        return {left_rectified, right_rectified};
    }

    void BinoUWCamera::initRemap(const MonoUWCamera& left_camera, const MonoUWCamera& right_camera){
        using namespace NAMESPACE_U3D::optics;
        const auto & left_pinhole = left_camera.pinhole();
        const auto & right_pinhole = right_camera.pinhole();

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
        is_init = true;
    }

    std::ostream& operator<<(std::ostream& os, const BinoUWCamera& camera){
        os << "BinoUWCamera:" << std::endl;
        os << "left_camera:" << std::endl << camera.left_camera_ << std::endl;
        os << "right_camera:" << std::endl << camera.right_camera_ << std::endl;
        return os;
    }

    void BinoUWCamera::reconstruct(const std::vector<cv::Point2f>& left_pts, 
                    const std::vector<cv::Point2f>& right_pts, 
                    std::vector<cv::Point3d>& reconstructed_pts) const
    {
        CV_Assert(left_pts.size() == right_pts.size());
        using namespace optics;
        reconstructed_pts.clear();
        reconstructed_pts.reserve(left_pts.size());
        const auto & left_camera = this->left_camera_;
        const auto & right_camera = this->right_camera_;

        int n = left_pts.size();
        for(int i = 0; i < n; ++i){
            auto left_ray = left_camera.backwardProject(left_pts[i]);
            auto right_ray = right_camera.backwardProject(right_pts[i]); //TODO:检查是否是世界坐标系下的
            if(!left_ray.has_value() || !right_ray.has_value()){
                reconstructed_pts.push_back(cv::Point3d(0.0, 0.0, 0.0));
                continue;
            }
            auto mid = intersect(left_ray.value(), right_ray.value());
            if(!mid.has_value()){
                reconstructed_pts.push_back(cv::Point3d(0.0, 0.0, 0.0));
                continue;
            }
            reconstructed_pts.push_back(mid.value());
        }
    }

    using optics::NSGAOptimizer;
    using optics::IMultiObjectiveProblem;

    /**
     * @brief 内部类，仅针对6个折射参数的标定问题
     */
    class RefractiveCalibrationProblem: public IMultiObjectiveProblem{
    private:
        std::vector<std::vector<cv::Point2f>> left_pts_;  //左图上观测的像素点，大小为k * N
        std::vector<std::vector<cv::Point2f>> right_pts_; //右图上观测的像素点，大小为k * N
        cv::Size m_boardSize_; //棋盘栅格大小
        double m_squareSize_; //棋盘栅格边长尺寸
        BinoCamera camera_;
        double d0_designed_;
        double d1_designed_;
        double n0_;
        double n1_;
        double n2_;
        const int m_numVariables = 6;  //六个参数，d0_l, d0_r, θ_l, θ_r, φ_l, φ_r
        const int m_numObjectives = 3;
        double max_tolerable_reproj = std::numeric_limits<double>::max();  //TODO: 应该根据实际情况调整
    public:
        RefractiveCalibrationProblem(const std::vector<std::vector<cv::Point2f>>& left_pts,
                                     const std::vector<std::vector<cv::Point2f>>& right_pts,
                                     cv::Size boardSize, double squareSize, const BinoCamera& camera, 
                                     double d0_designed = 30.0, double d1_designed = 5.0, double n0 = 1.0,
                                     double n1 = 1.5, double n2 = 1.333)
        {
            left_pts_ = left_pts;
            right_pts_ = right_pts;
            CV_Assert(left_pts_.size() == right_pts_.size()); //左右图的图片数量应该是一致的
            m_boardSize_ = boardSize;
            m_squareSize_ = squareSize;
            camera_ = camera;
            d0_designed_ = d0_designed;
            d1_designed_ = d1_designed;
            n0_ = n0;
            n1_ = n1;
            n2_ = n2;
        }
        
        ~RefractiveCalibrationProblem() override = default;

        int getVariableCount() const override { return m_numVariables; }

        int getObjectiveCount() const override { return m_numObjectives; }

        #ifndef M_PI
        #define M_PI 3.14159265358979323846
        #endif
        void getBounds(cv::Mat& lowerBounds, cv::Mat& upperBounds) const override{
            // 每个参数的范围：
            // d0_l, d0_r: [0, d0_designed]，
            // θ_l, θ_r: [-π/2, π/2]，这里假设倾斜角和方位角都在[-π/2, π/2]范围内
            // φ_l, φ_r: [-π/2, π/2]，这里假设倾斜角和方位角都在[-π/2, π/2]范围内
            lowerBounds = (cv::Mat_<double>(1, m_numVariables) << 0.6 * d0_designed_, 0.6 * d0_designed_, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0);
            upperBounds = (cv::Mat_<double>(1, m_numVariables) << 1.5 * d0_designed_, 1.5 * d0_designed_, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0, M_PI / 2.0);
        }

        void evaluate(cv::InputArray populationVars, cv::OutputArray out_populationObjs) const override{
            auto vars = populationVars.getMat();
            int N = vars.rows;
            int frames = left_pts_.size();
            cv::Mat objs(N, m_numObjectives, CV_64FC1);

            #pragma omp parallel for
            for(int i = 0;i < N;++i){
                const double * x = vars.ptr<double>(i);
                double * f = objs.ptr<double>(i);
                BinoUWCamera bino_camera = componentCamera(x);
                double E_rep = 0.0, E_scale = 0.0, E_planar = 0.0;  //三个误差损失函数

                //遍历棋盘格上所有角点
                size_t valid_count_of_proj = 0;
                size_t valid_count_of_scale = 0;
                size_t valid_count_of_planar = 0;
                for(int k = 0; k < frames; ++k){
                    const auto & pts_l = left_pts_[k];
                    const auto & pts_r = right_pts_[k];
                    if(pts_l.size() != pts_r.size()) continue;

                    std::vector<cv::Point3d> reconstructed_pts;
                    bino_camera.reconstruct(pts_l, pts_r, reconstructed_pts);

                    //重投影误差E_reproj的计算
                    for(size_t p = 0;p < reconstructed_pts.size();++p){
                        const auto & Pw = reconstructed_pts[p];
                        if(cv::norm(Pw) < optics::epsilon()) continue;
                        const auto & p_l = pts_l[p];
                        const auto & p_r = pts_r[p];
                        
                        auto proj_l = bino_camera.left().forwardProject(Pw, ForwardMethod::ITERATE_HELLAY);
                        if(!proj_l.has_value()) continue;
                        auto proj_r = bino_camera.right().forwardProject(Pw, ForwardMethod::ITERATE_HELLAY);
                        if(!proj_r.has_value()) continue;
                        
                        double err_l = cv::norm(p_l - (cv::Point2f)proj_l.value());
                        double err_r = cv::norm(p_r - (cv::Point2f)proj_r.value());
                        E_rep += err_l + err_r;
                        valid_count_of_proj += 2;
                    }
                    
                    //E_scale的计算
                    int board_width = m_boardSize_.width, board_height = m_boardSize_.height;
                    static const double sqrt_2 = std::sqrt(2.0);
                    double diagSize = m_squareSize_ * sqrt_2;
                    for(int r = 0; r < board_height - 1; ++r){
                        for(int c = 0; c < board_width - 1; ++c){
                            int idx = r * board_width + c;  //当前在1D vector中的索引
                            const cv::Point3d & p_current = reconstructed_pts[idx];
                            const cv::Point3d & p_right = reconstructed_pts[idx + 1];  //当前点的右边点
                            const cv::Point3d & P_rb = reconstructed_pts[idx + board_width + 1];  //右下角的点
                            if(cv::norm(p_current) < optics::epsilon()) continue;
                            if(cv::norm(p_right) < optics::epsilon()) continue;
                            if(cv::norm(P_rb) < optics::epsilon()) continue;
                            
                            double err_r = std::abs(cv::norm(p_right - p_current) - m_squareSize_);
                            double err_diag = std::abs(cv::norm(P_rb - p_current) - diagSize);

                            E_scale += err_r + err_diag / sqrt_2;
                            valid_count_of_scale++;
                        }
                    }
                    
                    //E_planar的计算
                    if(reconstructed_pts.size() < 3) continue; //拟合不出平面来
                    cv::Vec3d centroid(0, 0, 0);
                    int pts_size = reconstructed_pts.size();
                    int valid_pts_size = 0;
                    for(int j = 0;j < pts_size; ++j){
                        if(cv::norm(reconstructed_pts[j]) < optics::epsilon()) continue;
                        centroid += cv::Vec3d(reconstructed_pts[j].x, reconstructed_pts[j].y, reconstructed_pts[j].z);
                        valid_pts_size++;
                    } 
                    centroid /= (double)valid_pts_size;

                    cv::Matx33d cov(0, 0, 0, 0, 0, 0, 0, 0, 0);
                    for(int j = 0;j < pts_size; ++j){
                        if(cv::norm(reconstructed_pts[j]) < optics::epsilon()) continue;
                        cv::Vec3d diff = cv::Vec3d(reconstructed_pts[j].x, reconstructed_pts[j].y, reconstructed_pts[j].z) - centroid;
                        cov(0, 0) += diff[0] * diff[0]; cov(0, 1) += diff[0] * diff[1]; cov(0, 2) += diff[0] * diff[2];
                        cov(1, 0) += diff[1] * diff[0]; cov(1, 1) += diff[1] * diff[1]; cov(1, 2) += diff[1] * diff[2];
                        cov(2, 0) += diff[2] * diff[0]; cov(2, 1) += diff[2] * diff[1]; cov(2, 2) += diff[2] * diff[2];
                    }

                    cv::Matx31d evals;
                    cv::Matx33d evecs;
                    cv::eigen(cov, evals, evecs);  //求特征值，并将特征值按从大到小排序
                    cv::Vec3d normal(evecs(2, 0), evecs(2, 1), evecs(2, 2));  //取特征向量对应的最后一列，即特征值最小的特征向量，对应于平面的法向量
                    for(int j = 0;j < pts_size; ++j){
                        if(cv::norm(reconstructed_pts[j]) < optics::epsilon()) continue;
                        cv::Vec3d diff = cv::Vec3d(reconstructed_pts[j].x, reconstructed_pts[j].y, reconstructed_pts[j].z) - centroid;
                        E_planar += std::abs(diff.dot(normal));
                        valid_count_of_planar++;
                    }
                }

                f[0] = valid_count_of_proj > 0 ? E_rep / valid_count_of_proj : std::numeric_limits<double>::max();
                f[1] = valid_count_of_scale > 0 ? E_scale / valid_count_of_scale : std::numeric_limits<double>::max();
                f[2] = valid_count_of_planar > 0 ? E_planar / valid_count_of_planar : std::numeric_limits<double>::max();

                if(f[0] > max_tolerable_reproj) {
                    f[1] = std::numeric_limits<double>::max();
                    f[2] = std::numeric_limits<double>::max();
                }
            }

            if(out_populationObjs.needed()){
                objs.copyTo(out_populationObjs);
            }
        }    

        BinoUWCamera componentCamera(const double * cv_rows) const{
            double d0_l = cv_rows[0], d0_r = cv_rows[1];
            double theta_l = cv_rows[2], theta_r = cv_rows[3];
            double phi_l = cv_rows[4], phi_r = cv_rows[5];

            cv::Vec3d glass_normal_l(cos(phi_l) * sin(theta_l), sin(phi_l) * sin(theta_l), cos(theta_l));
            cv::Vec3d glass_normal_r(cos(phi_r) * sin(theta_r), sin(phi_r) * sin(theta_r), cos(theta_r));

            const MonoCamera & left_camera = camera_.left();
            const MonoCamera & right_camera = camera_.right();

            //TODO:做check
            MonoUWCamera left_camera_uw(left_camera, d0_l, d1_designed_, n0_, n1_, n2_, glass_normal_l);
            MonoUWCamera right_camera_uw(right_camera, d0_r, d1_designed_, n0_, n1_, n2_, glass_normal_r);
            return BinoUWCamera(left_camera_uw, right_camera_uw, false);
        }
    private:
        double huberLoss(double x, double delta = 1.0) const{
            double abs_x = std::abs(x);
            if(abs_x <= delta) return 0.5 * x * x;
            else return delta * abs_x - 0.5 * delta * delta;
        }
    };

    BinoUWCamera BinoUWCamera::calibrate(const std::string& left_imagePath, const std::string& right_imagePath,
                                         cv::Size boardSize, double squareSize, const BinoCamera& calibrated_camera,
                                         double * out_rms, bool verbose, double d0_designed, double d1_designed, 
                                         double n0, double n1, double n2, const std::string& log_file)
    {
        std::vector<cv::String> left_files, right_files;
        cv::glob(left_imagePath, left_files, false);
        cv::glob(right_imagePath, right_files, false);
        std::sort(left_files.begin(), left_files.end());
        std::sort(right_files.begin(), right_files.end());

        if(left_files.size() != right_files.size() || left_files.size() == 0){
            throw std::runtime_error("Calibration failed: Left and right image files must have the same number and be non-empty.");
        }
        std::vector<std::vector<cv::Point2f>> left_pts_all;
        std::vector<std::vector<cv::Point2f>> right_pts_all;

        for(int i = 0;i < left_files.size(); ++i){
            cv::Mat imgL = cv::imread(left_files[i], cv::IMREAD_GRAYSCALE);
            cv::Mat imgR = cv::imread(right_files[i], cv::IMREAD_GRAYSCALE);
            if(imgL.empty() || imgR.empty()) continue;

            std::vector<cv::Point2f> cornersL, cornersR;
            bool success_left = cv::findChessboardCorners(imgL, boardSize, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            bool success_right = cv::findChessboardCorners(imgR, boardSize, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if(!success_left || !success_right){
                if(verbose) std::cout << "Warning: Failed to find chessboard corners in image pair " << i << std::endl;
                continue;
            }

            cv::cornerSubPix(imgL, cornersL, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001));
            cv::cornerSubPix(imgR, cornersR, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001));

            left_pts_all.push_back(cornersL);
            right_pts_all.push_back(cornersR);
        }

        if(left_pts_all.empty()){
            throw std::runtime_error("Calibration failed: No valid chessboard corners found.");
        }

        auto problem = std::make_shared<RefractiveCalibrationProblem>(left_pts_all, right_pts_all, boardSize, squareSize, calibrated_camera, d0_designed, d1_designed, n0, n1, n2);
        if(verbose) std::cout << "Starting NSGA-II Refractive Calibration Optimization..." << std::endl;

        //优化器参数
        optics::OptimizerConfig config;
        config.populationSize = 1000;
        config.maxGenerations = 200;
        config.crossoverProb = 0.9;
        config.mutationProb = 0.1 / problem->getVariableCount();
        config.crossoverDistribution = 5.0;
        config.mutationDistribution = 5.0;
        std::ofstream log_file_stream(log_file);
        if(!log_file_stream.is_open()){
            throw std::runtime_error("Calibration failed: Failed to open log file.");
        }

        // 写入表头
        log_file_stream << "Iteration,Min_E_rep,Min_E_scale,Min_E_planar,Max_E_rep,Max_E_scale,Max_E_planar" << std::endl;

        //设置优化器的回调函数，这里用折线图来展示每一代的最优解的重投影误差、尺度误差和平面度误差
        NSGAOptimizer::IterationCallback callback = [&log_file_stream](int gen, const cv::Mat & popObjs, const cv::Mat & popVars) -> void {
            double min_rep = 1e9, min_scale = 1e9, min_planar = 1e9;
            double max_rep = -1e9, max_scale = -1e9, max_planar = -1e9;
            for(int i = 0; i < popObjs.rows; ++i){
                const double * f = popObjs.ptr<double>(i);
                min_rep = std::min(min_rep, f[0]);
                min_scale = std::min(min_scale, f[1]);
                min_planar = std::min(min_planar, f[2]);
                max_rep = std::max(max_rep, f[0]);
                max_scale = std::max(max_scale, f[1]);
                max_planar = std::max(max_planar, f[2]);
            }
            log_file_stream << gen + 1 << "," << min_rep << "," << min_scale << "," << min_planar << "," << max_rep << "," << max_scale << "," << max_planar << std::endl;
            std::cout << "\rGen: " << gen + 1 << " Min Rep: " << min_rep << " Max Rep: " << max_rep << 
                        " Min Scale: " << min_scale << " Max Scale: " << max_scale << 
                        " Min Planar: " << min_planar << " Max Planar: " << max_planar << std::flush;
        };

        optics::NSGAOptimizer optimizer(problem, config);
        optimizer.setIterationCallback(callback);
        cv::Mat paretoObjs, paretoVars;
        paretoVars = optimizer.optimize(paretoObjs);

        if(paretoVars.empty()){
            throw std::runtime_error("Calibration failed: NSGA-II optimization failed to find a solution.");
        }

        //将paretoObjs存入csv文件
        std::ofstream paretoObjs_stream("result.csv");
        if(!paretoObjs_stream.is_open()){
            throw std::runtime_error("Calibration failed: Failed to open paretoObjs file.");
        }
        paretoObjs_stream << "index,obj1_min,obj2_min,obj3_min" << std::endl;
        for(int i = 0; i < paretoObjs.rows; ++i){
            const double * f = paretoObjs.ptr<double>(i);
            paretoObjs_stream << i << "," << f[0] << "," << f[1] << "," << f[2] << std::endl;
        }
        paretoObjs_stream.close();
        log_file_stream.close();

        double ideal[3];
        cv::minMaxLoc(paretoObjs.col(0), &ideal[0], nullptr);
        cv::minMaxLoc(paretoObjs.col(1), &ideal[1], nullptr);
        cv::minMaxLoc(paretoObjs.col(2), &ideal[2], nullptr);

        // 寻找切比雪夫距离最小的解
        int best_idx = 0;
        double best_chebyshev = std::max({paretoObjs.at<double>(0,0) - ideal[0],
                                        paretoObjs.at<double>(0,1) - ideal[1],
                                        paretoObjs.at<double>(0,2) - ideal[2]});
        for(int i = 1; i < paretoObjs.rows; ++i){
            double cheb = std::max({paretoObjs.at<double>(i,0) - ideal[0],
                                    paretoObjs.at<double>(i,1) - ideal[1],
                                    paretoObjs.at<double>(i,2) - ideal[2]});
            if(cheb < best_chebyshev){
                best_chebyshev = cheb;
                best_idx = i;
            }
        }
        double best_reproj_err = paretoObjs.at<double>(best_idx, 0); // 选中解的obj1值
        cv::Mat best_vars = paretoVars.row(best_idx);
        BinoUWCamera bino_camera = problem->componentCamera(best_vars.ptr<double>(0));
        if(out_rms != nullptr) *out_rms = best_reproj_err;
        std::cout << std::endl;
        return bino_camera;
    }
}}