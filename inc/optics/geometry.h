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

}}
#endif