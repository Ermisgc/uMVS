/**
 * @file monocamera.h
 * @brief 定义了针孔相机模型，以及与该针孔相机所匹配的正反向投影计算方法
 * @author Ermis
 * @date 2025-12-14
 */

#ifndef U3D_CAMERA_MODEL_H
#define U3D_CAMERA_MODEL_H
#include "optics/refraction.h"
#include <vector>
NAMESPACE_BEGIN { namespace camera {
    /**
     * @brief 针孔相机模型
     * 包含：内参、畸变系数、外参
     * 负责像素坐标系、相机坐标系、世界坐标系三者之间的转换
     */
    class MonoCamera{
    public:
        Size imageSize;         ///< 图像分辨率
        Matx33d K;              ///< 相机内参矩阵
        Matx33d K_inv;          ///< 相机内参矩阵的逆矩阵
        Vec5d D;                ///< 相机畸变系数，用于畸变模型
        Matx33d R_w2c;          ///< 外参，相机旋转矩阵
        Vec3d t_w2c;            ///< 外参，相机平移向量

        /**
         * @brief 默认构造函数
         */
        MonoCamera():imageSize(0, 0), K(Matx33d::eye()), K_inv(Matx33d::eye()), D(0, 0, 0, 0, 0), 
                      R_w2c(Matx33d::eye()), t_w2c(0.0, 0.0, 0.0){}

        /**
         * @brief 构造函数
         * @param _K 相机内参矩阵
         * @param _D 相机畸变系数
         * @param _size 图像分辨率
         * @param _R_w2c 外参，相机旋转矩阵，默认值为单位矩阵
         * @param _t_w2c 外参，相机平移向量，默认值为(0, 0, 0)
         */
        MonoCamera(const Matx33d & _K, const Vec5d & _D, const cv::Size & _size,
                    const Matx33d & _R_w2c = Matx33d::eye(), const Vec3d & _t_w2c = Vec3d(0.0, 0.0, 0.0)):
            imageSize(_size), K(_K), K_inv(_K.inv()), D(_D), R_w2c(_R_w2c), t_w2c(_t_w2c){}

        /**
         * @brief 检查相机模型是否有效
         * 检查图像分辨率是否大于0，内参矩阵是否为正定矩阵
         * @return true 相机模型有效
         * @return false 相机模型无效
         */
        inline bool isValid() const {
            return imageSize.area() > 0 && K(0, 0) > 0 && K(1, 1) > 0;
        }

        /**
         * @brief 将像素坐标(u, v)去畸变，并反向投影为相机坐标系下的单位方向向量
         * 步骤：
         * 1. 去畸变 -> 归一化平面坐标
         * 2. 归一化坐标 -> 相机坐标系下的单位方向向量
         * d_cam = normalize([x, y, z])
         * @param uv 像素坐标，(u, v)
         * @return Vec3d 相机坐标系下的方向向量
         * @warning 本方法仅仅只用于单点处理，如果要处理大量的点，建议使用批量化方法的重载
         */
        Vec3d pixelToCamDir(const Point2d & uv) const;

        inline Vec3d pixelToCamDir(double u, double v) const {
            return pixelToCamDir(Point2d(u, v));
        }

        std::vector<Vec3d> pixelToCamDir(const std::vector<Point2d> & uv) const;

        /**
         * @brief 将像素坐标(u, v)转换为相机坐标系下的射线
         * 简单针孔模型，假设相机没有畸变：
         * 在相机坐标系下，光心坐标为(0, 0, 0)
         * 方向向量为 pixelToCamDir(u, v)
         * @param u 像素坐标的x方向
         * @param v 像素坐标的y方向
         * @return optics::Ray 相机坐标系下的射线
         */
        inline optics::Ray pixelToCamRay(double u, double v) const {
            Vec3d dir = pixelToCamDir(u, v);
            return optics::Ray(Vec3d(0.0, 0.0, 0.0), dir);
        }

        inline optics::Ray pixelToCamRay(const Point2d & uv) const {
            Vec3d dir = pixelToCamDir(uv);
            return optics::Ray(Vec3d(0.0, 0.0, 0.0), dir);
        }
        
        /**
         * @brief 反向投影将像素坐标(u, v)转换为世界坐标系下的射线
         * 先将像素坐标转换为相机坐标系下的单位方向向量d_cam，调用pixelToCamDir(u, v)
         * 再将d_cam转换到世界坐标系下，调用camDirToWorld(d_cam)
         * 最后将相机坐标系中心作为射线的起点，d_cam作为射线的方向向量。
         * @param uv 像素坐标，(u, v)
         * @return optics::Ray 世界坐标系下的射线
         */
        inline optics::Ray pixelToWorldRay(const Point2d & uv) const {
            Vec3d dir = pixelToCamDir(uv);
            dir = camDirToWorld(dir);
            Vec3d o = cameraCenterWorld();
            return optics::Ray(o, dir);
        }

        inline optics::Ray pixelToWorldRay(double u, double v) const {
            return pixelToWorldRay(Point2d(u, v));
        }

        /**
         * @brief 反向投影，将像素坐标(u, v)转换为世界坐标系下的射线
         * 先将像素坐标转换为相机坐标系下的单位方向向量d_cam，调用pixelToCamDir(u, v)
         * 再将d_cam转换到世界坐标系下，调用camDirToWorld(d_cam)
         * 最后将相机坐标系中心作为射线的起点，d_cam作为射线的方向向量。
         * @param pixel 像素坐标，(u, v)
         * @return optics::Ray 世界坐标系下的射线
         */
        inline optics::Ray backwardProject(const Point2d & pixel) const {
            return pixelToWorldRay(pixel);
        }

        /**
         * @brief 将相机坐标系下的点Pc转换到世界坐标系下
         * 简单针孔模型，假设相机没有畸变：
         * P_cam = R_c2w · P_world + t_c2w
         * P_world = R_c2w^T · (P_cam - t_c2w)
         * @param Pc 相机坐标系下的点
         * @return Vec3d 世界坐标系下的点
         */
        inline Vec3d camToWorld(const Vec3d & Pc) const {
            return R_w2c.t() * (Pc - t_w2c);
        }

        /**
         * @brief 将相机坐标系下的方向向量dc转换到世界坐标系下
         * d_world = R_w2c^T · d_cam，方向不受平移影响
         * @param dc 相机坐标系下的方向向量
         * @return Vec3d 世界坐标系下的方向向量
         */
        inline Vec3d camDirToWorld(const Vec3d & dc) const {
            return R_w2c.t() * dc;
        }

        /**
         * @brief 获取相机坐标系在世界坐标系下的中心坐标
         * 相机坐标系中心在世界坐标系下的坐标为：
         * P_camera_center_world = R_w2c^T · (-t_w2c)
         * @return Vec3d 相机坐标系中心在世界坐标系下的坐标
         */
        inline Vec3d cameraCenterWorld() const {
            return R_w2c.t() * (-t_w2c);
        }

        /**
         * @brief 正向投影，将世界坐标系下的点Pw转换为像素坐标(u, v)
         * 简单针孔模型
         * P_cam = R_w2c · P_world + t_w2c
         * 再利用简单针孔模型，将Pc转换为像素坐标：
         * uv = camToPixel(P_cam.xy() / P_cam.z())
         * @param Pw 世界坐标系下的点
         * @return std::optional<Vec2d> 像素坐标(u, v)，如果点在相机后则返回空
         */
        inline std::optional<Vec2d> worldToPixel(const Vec3d& Pw) const {
            Vec3d Pc = worldToCam(Pw);
            if(Pc[2] <= 1e-6) return std::nullopt;  // Pc[2]相当于深度，如果是负的，说明点在相机后
            return camToPixel(Pc);
        }

        /**
         * @brief 将世界坐标系下的点Pw转换为相机坐标系下的点Pc
         * P_cam = R_w2c · P_world + t_w2c
         * @param Pw 世界坐标系下的点
         * @return Vec3d 相机坐标系下的点
         */
        inline Vec3d worldToCam(const Vec3d& Pw) const {
            return R_w2c * Pw + t_w2c;
        }

        /**
         * @brief 将相机坐标系下的点Pc转换为像素坐标(u, v)
         * 简单针孔模型
         * uv = camToPixel(Pc.xy() / Pc.z())
         * @param Pc 相机坐标系下的点
         * @return Point2d 像素坐标(u, v)
         */
        std::optional<Point2d> camToPixel(const Vec3d& Pc) const;

        inline std::optional<Vec2d> forwardProject(const Vec3d& Pw) const {
            return worldToPixel(Pw);
        }

        /**
         * @brief 把本对象序列化到文件中
         * @param fs 文件存储对象
         */
        void write(cv::FileStorage& fs) const;

        /**
         * @brief 把本对象序列化到文件中
         * @param filename 文件名
         */
        void write(const std::string& filename) const;

        /**
         * @brief 从文件中读取本对象
         * @param fs 文件存储对象
         */
        void read(const cv::FileNode& fs);

        /**
         * @brief 从文件中读取本对象
         * @param filename 文件名
         */
        void read(const std::string& filename);

        /**
         * @brief 打印本对象的信息
         */
        void print() const;

        /**
         * @brief 打印本对象的信息
         * @param os 输出流对象
         * @param camera 相机模型对象
         * @return std::ostream& 输出流对象
         */
        friend std::ostream& operator<<(std::ostream& os, const MonoCamera& camera);

        /**
         * @brief 从一组图像标定针孔相机
         */
        static MonoCamera calibrate(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, bool verbose = false);

        /**
         * @brief 从图片路径标定针孔相机
         */
        static MonoCamera calibrate(const std::string& imagePath, cv::Size boardSize, double squareSize, bool verbose = false);
    };

    /**
     * @brief 针孔相机模型的标定方法
     * 
     * @param imageFiles 输入图片路径
     * @param boardSize 棋盘格角点数(cols, rows)，例如cv::Size(9, 6)表示9列6行
     * @param squareSize 棋盘格每个格子的实际边长，单位为mm
     * @param camera_model 输出的相机模型，以引用形式传递，标定完成后会被更新
     * @param verbose 是否打印和显示详细信息，默认值为false
     * @return double 标定误差，单位为mm
     */
    double calibratePinhole(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, MonoCamera& camera_model, bool verbose = false);

    /**
     * @brief 针孔相机模型的标定方法
     * 这里改为了输入标定图片所处的文件夹，然后对该文件夹下的所有图片进行标定
     * @param imagePath 输入标定图片所处的文件夹路径
     * @param boardSize 棋盘格角点数(cols, rows)，例如cv::Size(9, 6)表示9列6行
     * @param squareSize 棋盘格每个格子的实际边长，单位为mm
     * @param camera_model 输出的相机模型，以引用形式传递，标定完成后会被更新
     * @param verbose 是否打印和显示详细信息，默认值为false
     * @return double 标定误差，单位为mm
     */
    double calibratePinhole(const std::string & imagePath, cv::Size boardSize, double squareSize, MonoCamera& camera_model, bool verbose = false);

    static inline void write(cv::FileStorage& fs, const std::string& name, const MonoCamera& camera) {
        camera.write(fs);
    }

    static inline void read(const cv::FileNode& fs, MonoCamera & camera,const MonoCamera& default_value = MonoCamera()) {
        if(fs.empty()) camera = default_value;
        else camera.read(fs);
    }
}}  // namespace camera

#endif  // U3D_CAMERA_MODEL_H
