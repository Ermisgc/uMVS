#ifndef U3D_CAMERA_MODEL_H
#define U3D_CAMERA_MODEL_H
#include "optics/refraction.h"
NAMESPACE_BEGIN { namespace camera {
    /**
     * @brief 简单针孔相机模型
     * 目前暂时假设相机没有畸变，仅考虑相机内参和旋转平移变换
     * 定义像素坐标、相机坐标系和世界坐标系三个系的转换方法
     */
    struct CameraModel{
        double fx, fy, cx, cy;  ///< 相机内参
        int width, height;      ///< 相机图像的宽度和高度
        Matx33d R_cw;           ///< 相机旋转矩阵，将相机坐标系转换到世界坐标系
        Vec3d t_cw;             ///< 相机平移向量，将相机坐标系转换到世界坐标系
        Vec5d distCoeffs;       ///< TODO:相机畸变系数，用于畸变模型

        CameraModel():fx(0.0), fy(0.0), cx(0.0), cy(0.0), width(0), height(0), R_cw(Matx33d::eye()), t_cw(0.0, 0.0, 0.0){}
        CameraModel(double fx, double fy, double cx, double cy, int width, int height, const Matx33d& R_cw, const Vec3d& t_cw):
            fx(fx), fy(fy), cx(cx), cy(cy), width(width), height(height), R_cw(R_cw), t_cw(t_cw){}

        /**
         * @brief 获取3×3的内参矩阵K 对于针孔相机模型：
         * K = [fx, 0, cx
         *      0, fy, cy
         *      0, 0,  1 ]
         */
        inline cv::Matx33d K() const{
            return Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
        }

        /**
         * @brief 将像素坐标(u, v)转换为相机坐标系下的单位方向向量
         * 简单针孔模型，假设相机没有畸变：
         * x = (u - cx) / fx
         * y = (v - cy) / fy
         * z = 1.0
         * d_cam = normalize([x, y, z])
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @return Vec3d 相机坐标系下的方向向量
         */
        inline Vec3d pixelToCamDir(double u, double v) const {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            Vec3d dir = Vec3d(x, y, 1.0);
            return refract::normalize(dir);
        }

        /**
         * @brief 将像素坐标(u, v)转换为相机坐标系下的射线
         * 简单针孔模型，假设相机没有畸变：
         * 在相机坐标系下，光心坐标为(0, 0, 0)
         * 方向向量为 pixelToCamDir(u, v)
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @return refract::Ray 相机坐标系下的射线
         */
        inline refract::Ray pixelToCamRay(double u, double v) const {
            Vec3d dir = pixelToCamDir(u, v);
            return refract::Ray(Vec3d(0.0, 0.0, 0.0), dir);
        }

        /**
         * @brief 将相机坐标系下的点Pc转换到世界坐标系下
         * 简单针孔模型，假设相机没有畸变：
         * P_cam = R_cw · P_world + t_cw
         * P_world = R_cw^T · (P_cam - t_cw)
         * @param Pc 相机坐标系下的点
         * @return Vec3d 世界坐标系下的点
         */
        inline Vec3d camToWorld(const Vec3d & Pc) const {
            return R_cw.t() * (Pc - t_cw);
        }

        /**
         * @brief 将相机坐标系下的方向向量dc转换到世界坐标系下
         * d_world = R_cw^T · d_cam，方向不受平移影响
         * @param dc 相机坐标系下的方向向量
         * @return Vec3d 世界坐标系下的方向向量
         */
        inline Vec3d camDirToWorld(const Vec3d & dc) const {
            return R_cw.t() * dc;
        }

        /**
         * @brief 获取相机坐标系在世界坐标系下的中心坐标
         * 相机坐标系中心在世界坐标系下的坐标为：
         * P_camera_center_world = R_cw^T · (-t_cw)
         * @return Vec3d 相机坐标系中心在世界坐标系下的坐标
         */
        inline Vec3d cameraCenterWorld() const {
            return R_cw.t() * (-t_cw);
        }

        /**
         * @brief 将像素坐标(u, v)转换为世界坐标系下的射线
         * 先将像素坐标转换为相机坐标系下的单位方向向量d_cam，调用pixelToCamDir(u, v)
         * 再将d_cam转换到世界坐标系下，调用camDirToWorld(d_cam)
         * 最后将相机坐标系中心作为射线的起点，d_cam作为射线的方向向量。
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @return refract::Ray 世界坐标系下的射线
         */
        inline refract::Ray pixelToWorldRay(double u, double v) const {
            Vec3d dir = pixelToCamDir(u, v);
            dir = camDirToWorld(dir);
            Vec3d o = cameraCenterWorld();
            return refract::Ray(o, dir);
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
        inline std::optional<refract::Ray> pixelToRefractedRayCam(double u, double v, const std::vector<refract::RefractiveInterface>& interfaces) const {
            refract::Ray camRay = pixelToCamRay(u, v);
            if(auto ret = refract::traceThroughInterfaces(camRay, interfaces); ret.has_value()){
                return ret.value();
            } else return std::nullopt;
        }

        /**
         * @brief 将像素坐标(u, v)转换为世界坐标系下的折射射线
         * 先将像素坐标转换为相机坐标系下的折射射线，调用pixelToRefractedRayCam(u, v, n, n1, n2)
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
        inline std::optional<refract::Ray> pixelToRefractedRayWorld(double u, double v, const std::vector<refract::RefractiveInterface>& interfaces) const {
            if(auto ret = pixelToRefractedRayCam(u, v, interfaces); ret.has_value()){
                return refract::Ray(camToWorld(ret.value().o), camDirToWorld(ret.value().d));
            } else return std::nullopt;
        }

        /**
         * @brief 不考虑折射的情况下，将世界坐标系下的点Pw转换为像素坐标(u, v)
         * 简单针孔模型，假设相机没有畸变：
         * P_cam = R_cw · P_world + t_cw
         * 再利用简单针孔模型，将Pc转换为像素坐标：
         * uv = camToPixel(P_cam.xy() / P_cam.z())
         * @param Pw 世界坐标系下的点
         * @return std::optional<Vec2d> 像素坐标(u, v)，如果点在相机后则返回空
         */
        inline std::optional<Vec2d> worldToPixel(const Vec3d& Pw) const {
            Vec3d Pc = R_cw * Pw + t_cw;
            if(Pc[2] <= 0.0) return std::nullopt;  // 点在相机后
            Vec2d uv = {fx * Pc[0] / Pc[2] + cx, fy * Pc[1] / Pc[2] + cy};
            return uv;
        }
    };
}}  // namespace camera


#endif  // U3D_CAMERA_MODEL_H
