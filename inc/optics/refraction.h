#ifndef U3D_OPTICS_REFRACTION_H
#define U3D_OPTICS_REFRACTION_H
#include "utils.h"
#include <algorithm>
#include <optional>

NAMESPACE_BEGIN{ namespace refract {
    /**
     * @brief 小值
     * 用于数值的稳定性，避免除0错误
     */
    inline double epsilon() {return 1e-9;}
    
    /**
     * @brief 归一化向量
     * @param v 输入向量
     * @return Vec3d 归一化后的向量
     */
    inline Vec3d normalize(const Vec3d& v){
        double norm = cv::norm(v);
        if(norm < epsilon()) return v;
        return v / norm;
    } 

    /**
     * @brief 平面
     * 平面方程为n * x + d = 0，这里约定n为单位向量
     */
    struct Plane{
        Vec3d normal;   ///< 平面的法线方向
        double d;       ///< 平面方程的d项
        Plane(): normal(0.0, 0.0, 1.0), d(0.0){}
        Plane(const Vec3d& n, double d): normal(normalize(n)), d(d){}
    };

    /**
     * @brief 射线
     * 射线方程为p = o + t * d，这里约定d为单位向量
     */
    struct Ray{
        Vec3d o;        ///< 射线的起点
        Vec3d d;        ///< 射线的方向向量
        Ray():o(0.0, 0.0, 0.0),d(0.0, 0.0, 1.0){}
        Ray(const Vec3d& o, const Vec3d& d):o(o),d(normalize(d)){}
    };

    /**
     * @brief 射线与平面的交点
     * 平面方程为n · x + d = 0，
     * 射线方程为x = o + t * dir，
     * 设交点为p，则n · (o + t * dir) + d = 0，
     * 解得t = - (n · o + d) / (n · dir)
     * 
     * @param ray 输入射线
     * @param plane 输入平面
     * @param t 输出交点到射线起点的距离
     * @return std::optional<Vec3d> 射线与平面的交点坐标，如果不存在交点则返回std::nullopt
     */
    inline std::optional<Vec3d> intersectRayPlane(const Ray& ray, const Plane& plane, double& t){
        double denom = plane.normal.dot(ray.d);
        if(abs(denom) < epsilon()) return std::nullopt;  // 射线与平面近似平行，无交点
        t = -(plane.normal.dot(ray.o) + plane.d) / denom;
        if(t < 0.0) return std::nullopt;  // 交点在射线起点之前
        return ray.o + t * ray.d;
    }

    /**
     * @brief 折射计算，Snell定理，参考PBRT的实现
     * @param I 输入入射向量，指向前方
     * @param N 输入法线向量，指向出射介质，背离入射介质，这样的话，入射向量、出射向量与法线的夹角均为锐角
     * @param n1 输入入射介质的折射率, 必须大于等于1.0
     * @param n2 输入折射介质的折射率, 必须大于等于1.0
     * @return std::optional<Vec3d> 折射向量，如果发生全反射则返回std::nullopt
     */
    inline std::optional<Vec3d> refract(const Vec3d & I, const Vec3d & N, double n1, double n2){
        Vec3d T;  //输出折射向量
        assert(n1 >= 1.0 && n2 >= 1.0);  // 折射率必须大于1.0
        Vec3d Iu = normalize(I);
        Vec3d Nu = normalize(N);

        //斯涅尔定律：n1 * sin(theta1) = n2 * sin(theta2)
        double cosThetaI = Nu.dot(Iu); 

        if(cosThetaI < 0.0) {  // 入射向量与法线的夹角为钝角时，视作光线反向折射
            cosThetaI = -cosThetaI;
            Nu = -Nu;
            std::swap(n1, n2);
        }

        double sin2ThetaI = std::max(0.0, 1.0 - cosThetaI * cosThetaI);
        double eta = n1 / n2;
    
        double sin2ThetaT = eta * eta * sin2ThetaI;
        if(sin2ThetaT >= 1.0) return std::nullopt;  // 出射角的sin值大于1.0，全反射

        double cosThetaT = std::sqrt(std::max(0.0, 1.0 - sin2ThetaT));
        T = eta * Iu + (cosThetaT - eta * cosThetaI) * Nu;  // 分解为垂直于法线和平行于发线的分量
        return T;
    }

    /**
     * @brief 折射界面，描述为一个平面和入射出射介质的折射率
     * @attention plane的方向向量在设定中，应该由介质1指向介质2，如果弄反了，会导致计算错误
     */
    struct RefractiveInterface{
        Plane plane;    ///< 折射界面的平面
        double n1;      ///< 入射介质的折射率
        double n2;      ///< 出射介质的折射率
        RefractiveInterface():plane(), n1(1.0),n2(1.0){}
        RefractiveInterface(const Plane& p, double n1, double n2):plane(p),n1(n1),n2(n2){}
    };

    /**
     * @brief 射线追踪，穿过多个折射界面
     * @param ray 输入射线
     * @param interfaces 输入折射界面
     * @return std::optional<Ray> 折射后的射线，如果发生全反射则返回std::nullopt
     */
    inline std::optional<Ray> traceThroughInterfaces(const Ray& ray, const std::vector<RefractiveInterface>& interfaces){
        Ray result = ray;
        for(const auto& interface : interfaces){
            double t;
            if(auto ret = intersectRayPlane(result, interface.plane, t); ret.has_value()){
                result.o = ret.value();  //有相交点，更新射线的出射点
            } else continue;  // 射线与平面不相交,直接跳过
            
            //下面求出射方向，需要具备：出射光线的方向向量，入射介质的折射率，出射介质的折射率，法线向量
            const Vec3d & normal = interface.plane.normal;

            if(auto ret = refract(result.d, normal, interface.n1, interface.n2); ret.has_value()){
                result.d = ret.value();  //有折射向量，更新射线的出射方向
            } else return std::nullopt;  // 全反射
        }
        return result;
    }
}}

#endif  // U3D_OPTICS_REFRACTION_H