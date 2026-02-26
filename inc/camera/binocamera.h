#ifndef BINOCAMERA_H
#define BINOCAMERA_H
#include "utils.h"
#include "camera/monocamera.h"

NAMESPACE_BEGIN{ namespace camera{
    /**
     * @brief 双目相机模型
     * 
     */
    class BinoCamera{
    public:
        MonoCamera left_camera;        ///< 左相机模型
        MonoCamera right_camera;       ///< 右相机模型
    private:
        Matx33d R_l2r;                  ///< 左相机到右相机的旋转矩阵
        Vec3d t_l2r;                    ///< 左相机到右相机的平移向量
        Size imgSize;                   ///< 图像尺寸
        cv::Mat mapl1, mapl2, mapr1, mapr2;  ///< remap的函数
        bool is_init = false;           ///< 是否初始化
    public:
        BinoCamera() = default;
        BinoCamera(const MonoCamera& left_camera, const MonoCamera& right_camera);
        ~BinoCamera() = default;

        /**
         * @brief 立体校正，空气中的立体校正采用Bonguet方法，最大化共视图面积
         * @param left_img 左相机图像
         * @param right_img 右相机图像
         * @return std::pair<cv::Mat, cv::Mat> 校正后的左右相机图像
         */
        std::pair<cv::Mat, cv::Mat> rectify(const cv::Mat& left_img, const cv::Mat& right_img);

        const MonoCamera& left() const { return left_camera; }
        const MonoCamera& right() const { return right_camera; }

        /**
         * @brief 把本对象序列化到文件中
         * @param fs 文件存储对象
         */
        void write(cv::FileStorage& fs) const;

        /**
         * @brief 把本对象序列化到文件中
         * @param filename 文件路径
         */
        void write(const std::string& filename) const;

        /**
         * @brief 从文件中读取本对象
         * @param fs 文件存储对象
         */
        void read(const cv::FileNode& fs);

        /**
         * @brief 从文件中读取本对象
         * @param filename 文件路径
         */
        void read(const std::string& filename);

        /**
         * @brief 打印本对象的信息
         * @param os 输出流对象
         * @param camera 双目相机模型对象
         * @return std::ostream& 输出流对象
         */
        friend std::ostream& operator<<(std::ostream& os, const BinoCamera& camera);

        /**
         * @brief 从图片路径标定双目相机
         * @param left_imagePath 左相机图片路径
         * @param right_imagePath 右相机图片路径，这里的左右图片路径应该是对应的，即对于同一编号，左右两图拍摄同一场景
         * @param boardSize 棋盘格尺寸
         * @param squareSize 棋盘格格子尺寸
         * @param left_camera 左相机模型，可以空缺，如果空缺，则采用left_imagePath中的图片标定
         * @param right_camera 右相机模型，可以空缺，如果空缺，则采用right_imagePath中的图片标定
         * @param out_rms 输出均方根误差，可以空缺，如果空缺，则不输出均方根误差
         * @param verbose 是否打印详细信息，默认值为false
         * @return BinoCamera 标定后的双目相机模型
         */
        static BinoCamera calibrate(const std::string& left_imagePath,
                                    const std::string& right_imagePath,
                                    cv::Size boardSize, 
                                    double squareSize, 
                                    const MonoCamera * left_camera = nullptr, 
                                    const MonoCamera * right_camera = nullptr, 
                                    double * out_rms = nullptr,
                                    bool verbose = false);
    private:
        /**
         * @brief 初始化remap函数
         * @param left_camera 左相机模型
         * @param right_camera 右相机模型
         */
        void initRemap(const MonoCamera& left_camera, const MonoCamera& right_camera);
    };

}}

#endif