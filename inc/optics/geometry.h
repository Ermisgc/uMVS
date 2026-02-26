#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "refraction.h"

NAMESPACE_BEGIN{ namespace optics{
     /**
     * @brief 计算点线距，计算向量v = p - o,n是线的方向向量，那么|v×n| = |v||n|sinθ，其中θ是点p到线的角度，
     * 而由关系得，如果n为单位向量，那么|v×n| = |v|sinθ，即点p到线的距离
     * 所以点p到线的距离为|v|sinθ。
     * 示意图为：
     *            p
     *           /|
     *          / |
     *         /  |
     *        /   | d = |v|sinθ
     *       /    |
     *      /     |
     *     /      |
     *    /θ      |
     *   o--------n-----> line (direction n)
     *      v
     * @param p 点
     * @param ray 线
     * @return double 点到线的距离
     */
    inline double distPointToLine(const Vec3d& p, const Ray& ray){
        Vec3d v = p - ray.o;
        return cv::norm(v.cross(ray.d)) / cv::norm(ray.d);
    }

    inline double distPointToLine(const Vec3d& p, const Vec3d& o, const Vec3d& n){
        return distPointToLine(p, Ray{o, n});
    }

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
    std::optional<cv::Point3d> intersect(const Ray& ray1, const Ray& ray2);
}}
#endif