#ifndef U3D_UTILS_H
#define U3D_UTILS_H

#define NAMESPACE_U3D u3d
#define USING_U3D using namespace NAMESPACE_U3D;

#define NAMESPACE_BEGIN namespace NAMESPACE_U3D 
#define NAMESPACE_END 

#include <opencv2/opencv.hpp>
#include <random>

NAMESPACE_BEGIN{
    //这里做一些常用的类型定义，一方面是可以在命令空间内强制使用这些类型
    //另一方面，是为了给VSCode的智能提示功能提供一些中文的注释

    //三维向量，用于表示三维空间下的点、方向向量等
    using Vec3d = cv::Vec3d;

    //3x3矩阵，用于表示相机内参矩阵、旋转矩阵等
    using Matx33d = cv::Matx33d;

    //二维点，用于表示像素坐标等
    using Point2d = cv::Point2d;

    using Vec2d = cv::Vec2d;
    using Mat2d = cv::Matx22d;
    using Vec4d = cv::Vec<double, 4>;
    using Vec5d = cv::Vec<double, 5>;
    
    //尺寸，由两个整数表示，一般情况下表示cols和rows，即宽度和高度
    using Size = cv::Size;

    inline cv::Scalar randomColor(int seed){
        std::mt19937 gen(seed);
        std::uniform_int_distribution<int> dis(50, 255);
        return cv::Scalar(dis(gen), dis(gen), dis(gen));
    }

    /**
     * @brief 作用域定时器，用于测量代码块的执行时间，注意只能在单线程环境下使用
     * @param name 定时器名称，用于在输出中标识不同的代码块
     */
    struct ScopedTimer {
        std::string name;
        std::chrono::high_resolution_clock::time_point start;

        ScopedTimer(const std::string& n) : name(n), start(std::chrono::high_resolution_clock::now()) {}
        ~ScopedTimer() {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            std::cout << "[Profiler] " << name << " took: " << duration / 1000.0 << " ms\n";
        }
    };
}

#endif