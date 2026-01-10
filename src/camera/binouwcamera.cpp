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

    void BinoUWCamera::read(const cv::FileNode& fs) {
        fs["left_camera"] >> left_camera_;
        fs["right_camera"] >> right_camera_;
    }
}}