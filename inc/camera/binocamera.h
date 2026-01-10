#ifndef BINOCAMERA_H
#define BINOCAMERA_H
#include "utils.h"
#include "camera/camera_model.h"

NAMESPACE_BEGIN{ namespace camera{
    /**
     * @brief 双目相机模型
     * 
     */
    class BinocularCamera{
    public:
        CameraModel left_camera;        ///< 左相机模型
        CameraModel right_camera;       ///< 右相机模型
    private:
        Matx33d R_l2r;                  ///< 左相机到右相机的旋转矩阵
        Vec3d t_l2r;                    ///< 左相机到右相机的平移向量
        Size imgSize;                   ///< 图像尺寸
        cv::Mat mapl1, mapl2, mapr1, mapr2;  ///< remap的函数
        bool is_init = false;           ///< 是否初始化
    public:
        BinocularCamera() = default;
        BinocularCamera(const CameraModel& left_camera, const CameraModel& right_camera);
        ~BinocularCamera() = default;

        /**
         * @brief 立体校正，空气中的立体校正采用Bonguet方法，最大化共视图面积
         * @param left_img 左相机图像
         * @param right_img 右相机图像
         * @return std::pair<cv::Mat, cv::Mat> 校正后的左右相机图像
         */
        std::pair<cv::Mat, cv::Mat> rectify(const cv::Mat& left_img, const cv::Mat& right_img);

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

        /**
         * @brief 打印本对象的信息
         * @param os 输出流对象
         * @param camera 双目相机模型对象
         * @return std::ostream& 输出流对象
         */
        friend std::ostream& operator<<(std::ostream& os, const BinocularCamera& camera);
    private:
        /**
         * @brief 初始化remap函数
         * @param left_camera 左相机模型
         * @param right_camera 右相机模型
         */
        void initRemap(const CameraModel& left_camera, const CameraModel& right_camera);
    };

}}

#endif