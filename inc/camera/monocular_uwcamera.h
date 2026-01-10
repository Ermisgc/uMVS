/**
 * @file monocular_uwcamera.h
 * @author Ermis
 * @brief 定义了水下单目相机模型，该相机由一个普通针孔相机、两个相平行的折射平面组成
 * 实现了该相机的正向投影和反向投影计算方法
 * @version 0.1
 * @date 2025-12-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef U3D_MONO_UWCAMERA_H
#define U3D_MONO_UWCAMERA_H
#include "utils.h"
#include "camera/camera_model.h"
NAMESPACE_BEGIN { namespace camera {
    /**
     * @brief 水下单目相机模型的正向投影解法
     * 这里主要实现了水下单目相机的几种正向投影方法
     * 迭代法：采用Gauss-Newton迭代法求解，初始值为针孔相机的投影中心
     * 解析法：直接求解折射界面的四次方程
     * 几何法：这里采用我的方法，一个假想的与玻璃法线重合的虚拟相机坐标系，然后基于该虚拟相机坐标系求初值解
     */
    enum class ForwardMethod{
        ITERATE = 0,    ///< 迭代法
        ANALYTIC = 1,   ///< 解析法
        GEOMETRIC = 2,  ///< 几何法
    };

    /**
     * @brief 水下单目相机模型
     * 该相机由一个普通针孔相机、两个相平行的折射平面组成
     * 这里约定相机坐标系为针孔相机的坐标系
     * 世界坐标系与相机坐标系的变换矩阵即是针孔相机内部的投影矩阵
     * @param camera_model_ 针孔相机模型
     * @param refraction_planes_ 折射平面集合，理论上有一个玻璃，对应两个折射平面
     */
    class MonocularUWCamera {
        CameraModel camera_model_;                                       ///< 针孔相机模型
        std::vector<optics::RefractiveInterface> refraction_planes_;     ///< 折射界面集合，这里按从近到远的顺序存储

        // 物理参数的缓存
        double n_air_;      ///< 空气的折射率
        double n_glass_;    ///< 玻璃的折射率
        double n_water_;    ///< 水的折射率
        double d_housing_;  ///<  housing的厚度
        double d_glass_;    ///< 玻璃的厚度
        Vec3d glass_normal_;  ///< 玻璃的法线
    public:
        MonocularUWCamera() = default;
        /**
         * @brief 从已经构造好的针孔相机模型、两个折射平面来构造水下单目相机模型
         * @param camera_model 针孔相机模型
         * @param p1 第一个折射界面
         * @param p2 第二个折射界面
         * @details RefractiveInterface的方程为n · x + d = 0，这里的x应该看作是相机坐标系下的坐标
         */
        MonocularUWCamera(const CameraModel& camera_model, const optics::RefractiveInterface& p1, const optics::RefractiveInterface& p2);

        /**
         * @brief 从已经构造好的针孔相机模型、标定好的光心到玻璃的距离、玻璃厚度、折射率等参数来构造水下单目相机模型
         * @param camera_model 针孔相机模型
         * @param d1 光心到玻璃的距离
         * @param d2 玻璃的厚度
         * @param n0 空气的折射率
         * @param n1 玻璃的折射率
         * @param n2 水的折射率
         * @param glass_normal 玻璃的法线，这里要注意玻璃的法线方向由光心指向相机外，如果不是这样的话，函数内部会自动取反
         * @details 构造函数内部将自己构造和装配折射平面
         */
        MonocularUWCamera(const CameraModel& camera_model, double d1, double d2, double n0, double n1, double n2, const Vec3d& glass_normal);

        inline const CameraModel & pinhole() const { return camera_model_; }
        /**
         * @brief 水下单目相机的正向投影方法，计算世界坐标系下的点的像素投影
         * @param world_point 世界坐标系下的点
         * @param method 正向投影方法，默认采用迭代法
         * @return std::optional<Point2d> 成像坐标系下的点，若点在相机的成像平面外，则返回空
         */
        std::optional<Point2d> forwardProject(const Vec3d& world_point, ForwardMethod method = ForwardMethod::ITERATE) const;

        /**
         * @brief 水下单目相机的反向投影方法，计算像素点的反向投影射线在特定深度下的世界坐标
         * @param camera_point 像素点坐标
         * @param depth 像素点的深度，即相机坐标系下的z坐标
         * @return std::optional<Vec3d> 世界坐标系下的点，若点在相机的成像平面外，则返回空
         */
        std::optional<Vec3d> backwardProject(const Vec2d& pixel, double depth) const;

        /**
         * @brief 水下单目相机的反向投影方法，计算像素点的折射射线
         * @param pixel 像素点坐标
         * @return std::optional<NAMESPACE_U3D::refract::Ray> 折射射线，若点在相机的成像平面外，则返回空
         */
        inline std::optional<optics::Ray> backwardProject(const Vec2d& pixel) const {
            return pixelToWorldRay(pixel[0], pixel[1]);
        }


        /**
         * @brief 将像素坐标(u, v)转换为相机坐标系下的折射射线
         * 先将像素坐标转换为相机坐标系下的单位方向向量d_cam，调用pixelToCamDir(u, v)
         * 然后调用多层折射函数traceThroughInterfaces，得到折射射线的方向向量
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @param n 折射界面的法线向量
         * @param n1 入射介质的折射率
         * @param n2 出射介质的折射率
         * @param refractedRay 输出折射射线
         * @return true 折射成功
         * @return false 折射失败
         */
        inline std::optional<optics::Ray> pixelToRefractedRayCam(double u, double v) const {
            optics::Ray camRay = camera_model_.pixelToCamRay(u, v);
            return optics::traceThroughInterfaces(camRay, refraction_planes_);
        }

        /**
         * @brief 将像素坐标(u, v)转换为世界坐标系下的折射射线
         * 先将像素坐标转换为相机坐标系下的折射射线，调用pixelToRefractedRayCam(u, v)
         * 再将相机坐标系下的折射射线转换到世界坐标系下，调用camToWorld(Pc)和camDirToWorld(dc)
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @param n 折射界面的法线向量
         * @param n1 入射介质的折射率
         * @param n2 出射介质的折射率
         * @param refractedRay 输出折射射线
         * @return true 折射成功
         * @return false 折射失败
         */
        inline std::optional<optics::Ray> pixelToWorldRay(double u, double v) const {
            if(auto ret = pixelToRefractedRayCam(u, v); ret.has_value()){
                return optics::Ray(camera_model_.camToWorld(ret.value().o), camera_model_.camDirToWorld(ret.value().d));
            } else return std::nullopt;
        }

        inline std::optional<optics::Ray> pixelToWorldRay(const Vec2d& pixel) const {
            return pixelToWorldRay(pixel[0], pixel[1]);
        }

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
    private:
        /**
         * @brief 水下单目相机的正向投影方法，采用不动点迭代法求解
         * 
         * 策略：
         * 1. 初值：使用简单的针孔投影，得到一个初始值uv_0
         * 2. 循环：
         *     - 反向光线追踪得到射线 R
         *     - 计算 R 在目标深度pc.z的虚拟点P_virtual
         *     - 计算残差：E = Pc.xy - P_virtual.xy （有符号）
         *     - 利用焦距将误差映射回像素域：delta_uv = E * f / Z
         *     - 更新迭代值：uv = uv_0 + delta_uv
         * 3. 若迭代收敛（例如，两次迭代结果差异小于阈值），则返回 uv 作为投影点
         * @param pc 相机坐标系下的点
         * @return std::optional<Point2d> 成像坐标系下的点，若点在相机的成像平面外，则返回空
         */
        std::optional<Point2d> solveIterative(const Vec3d& pc) const;

        /**
         * @brief 水下单目相机的正向投影方法，解析法求解
         * 
         * 这里参考了Agrawal et al. CVPR 2012的论文
         * 经过他的补充材料的推导，在单层折射时，解析解将变为求界面径向距离r的四次方程
         * 在双层折射时，解析解将变为求解界面径向距离r的四次或十二次方程
         * 对于四次以上的方程，其求解非常不稳定，因此这里的解析解法会把模型简化为单层折射
         * @param pc 相机坐标系下的点
         * @return std::optional<Point2d> 成像坐标系下的点，若点在相机的成像平面外，则返回空
         */
        std::optional<Point2d> solveAnalytic(const Vec3d& pc) const;

        /**
         * @brief 水下单目相机的正向投影方法，几何法求解
         * 
         * 策略：TODO:待实现，转化为轴向相机求解
         * @param pc 相机坐标系下的点
         * @return std::optional<Point2d> 成像坐标系下的点，若点在相机的成像平面外，则返回空
         */
        std::optional<Point2d> solveGeometric(const Vec3d& pc) const;
    };

    /**
     * @brief 水下单目相机的标定方法
     * 
     * @param imageFiles 输入图片路径
     * @param boardSize 棋盘格角点数(cols, rows)，例如cv::Size(9, 6)表示9列6行
     * @param squareSize 棋盘格每个格子的实际边长，单位为mm
     * @param camera_model 已经标定好的针孔相机模型，包含内参K、畸变系数D、旋转矩阵R_w2c、平移向量t_w2c
     * @param uw_camera 输出的水下单目相机模型，以引用形式传递，标定完成后会被更新
     * @param verbose 是否打印和显示详细信息，默认值为false
     * @return double 标定误差，单位为mm
     */
    double calibrate(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, const CameraModel& camera_model, MonocularUWCamera& uw_camera, bool verbose = false);

    double calibrate(const std::string & imagePath, cv::Size boardSize, double squareSize, const CameraModel& camera_model, MonocularUWCamera& uw_camera, bool verbose = false);

    static inline void write(cv::FileStorage& fs, const std::string& name, const MonocularUWCamera& camera) {
        camera.write(fs);
    }

    static inline void read(const cv::FileNode& fs, MonocularUWCamera & camera,const MonocularUWCamera& default_value = MonocularUWCamera()) {
        if(fs.empty()) camera = default_value;
        else camera.read(fs);
    }
}}

#endif  // U3D_MONO_UWCAMERA_H

