/**
 * @file binouwcamera.h
 * @author Ermis
 * @brief 定义了水下双目相机模型
 * 包含左右两个水下单目相机，提供了计算广义对极曲线等双目几何功能
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef U3D_BINO_UWCAMERA_H
#define U3D_BINO_UWCAMERA_H

#include <vector>
#include <algorithm>
#include "utils.h"
#include "monocular_uwcamera.h"

NAMESPACE_BEGIN { namespace camera {

    /**
     * @brief 水下双目相机模型
     * 
     * 该类封装了左右两个 MonocularUWCamera 对象。
     * 重要假设：
     * 1. 两个单目相机的外参（Pose）必须定义在同一个世界坐标系下。
     *    通常习惯将左相机坐标系设为世界坐标系（即左相机 R=I, t=0），
     *    右相机的 R, t 则是相对于左相机的变换。
     * 2. 两个相机的介质参数（水、玻璃折射率）应当一致，但允许 housing 几何参数不同。
     */
    class BinoUWCamera {
    private:
        MonocularUWCamera left_camera_;     ///< 左侧水下单目相机
        MonocularUWCamera right_camera_;    ///< 右侧水下单目相机

    public:
        BinoUWCamera() = default;

        /**
         * @brief 构造函数
         * @param left 左相机模型
         * @param right 右相机模型
         */
        BinoUWCamera(const MonocularUWCamera& left, const MonocularUWCamera& right) 
            : left_camera_(left), right_camera_(right) {}

        /**
         * @brief 获取左相机引用
         */
        inline const MonocularUWCamera& left() const { return left_camera_; }

        /**
         * @brief 获取右相机引用
         */
        inline const MonocularUWCamera& right() const { return right_camera_; }

        /**
         * @brief 计算广义对极曲线（Generalized Epipolar Curve）
         * 
         * 给定左图的一个像素点和一系列深度值，计算这些点在右图中的投影坐标。
         * 这些投影点的连线即构成了该像素对应的广义对极曲线。
         * 
         * @param left_pixel 左图中的像素坐标 (u, v)
         * @param depths 深度采样列表 (z值)，单位与标定单位一致(通常为mm或m)。
         *               注意：这里的depth指的是相机坐标系下的Z轴深度。
         * @return std::vector<Point2d> 右图中对应的曲线点集。若某深度不可见，则该点会被跳过。
         */
        std::vector<Point2d> computeEpipolarCurve(const Vec2d& left_pixel, const std::vector<double>& depths) const;
        
        /**
         * @brief 计算广义对极曲线（自动采样）
         * 
         * 这是一个辅助重载函数。它会在 [min_depth, max_depth] 之间自动生成采样点。
         * 为了使曲线在图像平面上的点分布更均匀，内部采用“逆深度(Inverse Depth)”均匀采样。
         * 
         * @param left_pixel 左图中的像素坐标
         * @param min_depth 最小深度 (e.g. 500mm)
         * @param max_depth 最大深度 (e.g. 10000mm)
         * @param steps 采样点数量，默认为 50 个
         * @return std::vector<Point2d> 右图中对应的曲线点集
         */
        std::vector<Point2d> computeEpipolarCurve(const Vec2d& left_pixel, double min_depth, double max_depth, int steps = 50) const;

        /**
         * @brief 验证立体校正效果（计算垂直视差）
         * 
         * 给定左图像素和其对应的真实深度，计算该点投影到右图后，
         * 与"理想水平对极线"（即校正后右图的同一行 v_left）的垂直偏差。
         * 
         * @param left_pixel_rect 校正后左图的像素 (u_l, v_l)
         * @param depth 真实深度
         * @return double 垂直误差 (v_right - v_left)，单位：像素
         */
        double computeVerticalDisparity(const Vec2d& left_pixel_rect, double depth) const;

        /**
         * @brief 把本对象序列化到文件中
         * @param fs 文件存储对象
         */
        void write(cv::FileStorage& fs) const;

        /**
         * @brief 从文件中读取本对象
         * @param fs 文件存储对象
         */
        void read(const cv::FileNode& fs);
    };

}}

#endif // U3D_BINO_UWCAMERA_H