#include "optics/geometry.h"
NAMESPACE_BEGIN { namespace optics{
    /**
     * @brief 计算两个光线的最短中垂线的中点，ray1为p1 = o1 + t1 * d1，ray2为p2 = o2 + t2 * d2，
     * 假设中垂线与ray1与ray2的交点分别为p1和p2，那么中垂线的中点为(p1 + p2) / 2，
     * 对于向量v = p1 - p2，由于它是中垂线，因此v · d1 = 0，v · d2 = 0，
     * 假设w0 = o1 - o2 = p1 - p2 - t1 * d1 + t2 * d2 = v - t1 * d1 + t2 * d2，
     * 左右两侧点乘d1：w0 · d1 = - t1 * d1 · d1 + t2 * d1 · d2，
     * 左右两侧点乘d2：w0 · d2 = - t1 * d1 · d2 + t2 * d2 · d2，
     * 由于d1与d2都限制为单位向量，假设d1 · d2 = m，因此：
     * - t1 + t2 * m = w0 · d1，
     * - t1 * m + t2 = w0 · d2
     * 解得：t1 = (m * d2 · w0 - d1 · w0) / (1 - m^2), 
     * t2 = (d2 · w0 - m * d1 · w0) / (1 - m^2)
     * 
     * @param ray1 光线1
     * @param ray2 光线2
     * @return Point3f 最短中垂线的中点
     */
    std::optional<cv::Point3d> intersect(const Ray& ray1, const Ray& ray2){
        cv::Vec3d ray1_copy_d = normalize(ray1.d);
        cv::Vec3d ray2_copy_d = normalize(ray2.d);
        //首先将方向归一化
        cv::Vec3d w0 = ray1.o - ray2.o;
        double m = ray1_copy_d.dot(ray2_copy_d);
        double delta = 1 - m * m;
        if(delta < epsilon()) return (ray1.o + ray2.o) / 2.0;
        double w1 = w0.dot(ray1_copy_d);
        double w2 = w0.dot(ray2_copy_d);
        double t1 = (m * w2 - w1) / delta;
        double t2 = (w2 - m * w1) / delta;
        if(t1 < epsilon() && t2 < epsilon()) return std::nullopt; 
        if(t1 < epsilon()){
            t1 = 0.0;
            t2 = w2;
            if(t2 < epsilon()) t2 = 0.0;
        } else if(t2 < epsilon()){
            t2 = 0.0;
            t1 = -w1;
            if(t1 < epsilon()) t1 = 0.0;
        }
        return (ray1.o + t1 * ray1_copy_d + ray2.o + t2 * ray2_copy_d) / 2.0;
    }
}}