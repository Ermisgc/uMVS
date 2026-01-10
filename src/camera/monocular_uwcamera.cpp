#include "camera/monocular_uwcamera.h"
NAMESPACE_BEGIN { namespace camera {
    MonocularUWCamera::MonocularUWCamera(const CameraModel& camera_model,const optics::RefractiveInterface& p1, const optics::RefractiveInterface& p2){
        auto p1_copy = p1;
        auto p2_copy = p2;

        if(p1.plane.normal[2] < 0){
            p1_copy.plane.normal = -p1_copy.plane.normal;
            p1_copy.plane.d = -p1_copy.plane.d;
            //交换法线前后折射率
            std::swap(p1_copy.n1, p1_copy.n2);
        }
        if(p2.plane.normal[2] < 0){
            p2_copy.plane.normal = -p2_copy.plane.normal;
            p2_copy.plane.d = -p2_copy.plane.d;
            //交换法线前后折射率
            std::swap(p2_copy.n1, p2_copy.n2);
        }
        assert(p1_copy.n2 == p2_copy.n1, ">>> Not match the refractive model, the n2 of p1 is not equal to n1 of p2");
        refraction_planes_= {std::move(p1_copy), std::move(p2_copy)};
        camera_model_ = camera_model;
        n_air_ = refraction_planes_[0].n1;
        n_glass_ = refraction_planes_[1].n1;
        n_water_ = refraction_planes_[1].n2;
        d_housing_ = -refraction_planes_[0].plane.d;
        d_glass_ = (-refraction_planes_[1].plane.d) - d_housing_;
        glass_normal_ = refraction_planes_[1].plane.normal;
    }

    MonocularUWCamera::MonocularUWCamera(const CameraModel& camera_model, double d1, double d2, double n0, double n1, double n2, const Vec3d& glass_normal):
        camera_model_(camera_model), n_air_(n0), n_glass_(n1), n_water_(n2), d_housing_(d1), d_glass_(d2), glass_normal_(glass_normal) 
    {
        camera_model_ = camera_model;
        Vec3d glass_normal_copy = glass_normal[2] < 0 ? -glass_normal : glass_normal; //确保玻璃平面法线指向外部
        glass_normal_copy = optics::normalize(glass_normal_copy);
        
        //组装界面，首先计算光心到玻璃外表面的距离
        double d = d1 + d2;

        //两个界面：
        auto p1 = optics::RefractiveInterface{{glass_normal_copy, -d1}, n0, n1};  // 第一个平面n · x - d1 = 0
        auto p2 = optics::RefractiveInterface{{glass_normal_copy, -d}, n1, n2};   // 第二个平面n · x - d = 0
        
        refraction_planes_ = {std::move(p1), std::move(p2)};
    }

    std::optional<Point2d> MonocularUWCamera::forwardProject(const Vec3d& pw, ForwardMethod method) const{
        //首先将点从世界坐标系变换到相机坐标系
        Vec3d pc = camera_model_.worldToCam(pw);

        //深度检查：
        if(pc[2] <= d_housing_ + d_glass_ + 1e-3) return std::nullopt;  //点在单目水下相机内部或者太过接近，视为无效三维点

        switch (method){
            case ForwardMethod::ITERATE:
                return this->solveIterative(pc);
            case ForwardMethod::ANALYTIC:
                return this->solveAnalytic(pc);
            case ForwardMethod::GEOMETRIC:
                return this->solveGeometric(pc);
            default:
                return this->solveIterative(pc);
        }
        return std::nullopt;
    }

    std::optional<Vec3d> MonocularUWCamera::backwardProject(const Vec2d& uv, double depth) const{
        //求像素点通过两个平面折射后发出的射线
        auto ret = this->backwardProject(uv);
        if(!ret.has_value()){
            return std::nullopt;
        }

        //求射线上深度距离相机光心为depth的点
        //P = o + t * d, P.z = depth
        //因此o.z + t * d.z = depth
        //因此t = (depth - o.z) / d.z
        auto ray = ret.value();
        if(std::abs(ray.d[2]) < 1e-6) {  // 射线无限平行于xy平面，成像点将无限远，这里简单让它返回空
            return std::nullopt;
        }

        double t = (depth - ray.o[2]) / ray.d[2];
        if(t < 0) return std::nullopt;  //深度在反方向，所求点在玻璃内部
        Vec3d P = ray.o + t * ray.d;
        return P;
    }

    void MonocularUWCamera::write(cv::FileStorage& fs) const {
        fs << "camera_model" << "{";
        camera_model_.write(fs);
        fs << "}";

        fs << "n_air" << n_air_;
        fs << "n_glass" << n_glass_;
        fs << "n_water" << n_water_;
        fs << "d_housing" << d_housing_;
        fs << "d_glass" << d_glass_;
        fs << "glass_normal" << glass_normal_;
    }

    void MonocularUWCamera::read(const cv::FileNode& fs) {
        fs["camera_model"] >> camera_model_;
        fs["n_air"] >> n_air_;
        fs["n_glass"] >> n_glass_;
        fs["n_water"] >> n_water_;
        fs["d_housing"] >> d_housing_;
        fs["d_glass"] >> d_glass_;
        fs["glass_normal"] >> glass_normal_;
        glass_normal_ = optics::normalize(glass_normal_);
        if(glass_normal_[2] < 0) glass_normal_ = -glass_normal_;
        // 组装界面
        auto p1 = optics::RefractiveInterface{{glass_normal_, -d_housing_}, n_air_, n_glass_};  // 第一个平面n · x - d1 = 0
        auto p2 = optics::RefractiveInterface{{glass_normal_, -d_housing_ - d_glass_}, n_glass_, n_water_};   // 第二个平面n · x - d = 0
        refraction_planes_ = {std::move(p1), std::move(p2)};
    }

    std::optional<Point2d> MonocularUWCamera::solveIterative(const Vec3d& pc) const{
        //策略：
        //1. 初值：使用简单的针孔投影，得到一个初始值uv_0
        //2. 循环：
        //    - 反向光线追踪得到射线 R
        //    - 计算 R 在目标深度pc.z的虚拟点P_virtual
        //    - 计算残差：E = Pc.xy - P_virtual.xy
        //    - 利用焦距将误差映射回像素域：delta_uv = E * f / Z
        //    - 更新迭代值：uv = uv_0 + delta_uv
        //3. 若迭代收敛（例如，两次迭代结果差异小于阈值），则返回 uv 作为投影点
        
        //1. 初始猜测：使用简单的针孔投影，得到一个初始值uv_0
        Point2d uv;
        if(auto ret = camera_model_.camToPixel(pc); !ret.has_value()){
            return std::nullopt;
        } else uv = ret.value();  // 初始值uv_0

        const int MAX_ITER = 20;
        const double TOLERANCE = 1e-4;  //迭代收敛阈值

        //2. 迭代循环：
        for(int i = 0;i < MAX_ITER; ++i){
            //先做一个反向投影，得到一个初始值uv_0
            auto ray_opt = this->pixelToRefractedRayCam(uv.x, uv.y);
            if(!ray_opt.has_value()){
                return std::nullopt;
            }

            auto ray = ray_opt.value();  //反向追踪得到的射线（相机坐标系下）
            //计算 R 在目标深度pc.z的虚拟点P_virtual
            //Pc = o + t * d，只看z方向, 得到t = (pc.z - o.z) / d.z
            if(std::abs(ray.d[2]) < 1e-6) return std::nullopt;  // 射线无限平行于xy平面，成像点将无限远，这里简单让它返回空

            double t = (pc[2] - ray.o[2]) / ray.d[2];
            if(t < 0) return std::nullopt;  //深度在反方向，所求点在玻璃内部
            Vec3d P_virtual = ray.o + t * ray.d;  //虚拟点P_virtual

            //计算残差
            Vec2d error_xy(pc[0] - P_virtual[0], pc[1] - P_virtual[1]);
            if(cv::norm(error_xy) < TOLERANCE){
                return uv;  //迭代收敛
            }
            
            //计算更新量：使用简化的Jacobian近似：d(pi) / d(uv) = [fx / Z, 0; 0, fy / Z]
            double Z = pc[2];
            double fx = camera_model_.K(0,0);
            double fy = camera_model_.K(1,1);
            Mat2d J;
            J << fx / Z, 0, 0, fy / Z;  // 简化的Jacobian近似
            Vec2d delta_uv = J * error_xy;
            
            //更新迭代值：uv = uv_0 + delta_uv
            uv += cv::Point2d(delta_uv[0], delta_uv[1]);

            if(uv.x < -1000 || uv.x > camera_model_.imageSize.width + 1000 || uv.y < -1000 || uv.y > camera_model_.imageSize.height + 1000){
                return std::nullopt;  // 迭代结果超出图像边界，视为无效
            }
        }
        return uv;  // 返回最后一次迭代结果
    }

    std::optional<Point2d> MonocularUWCamera::solveAnalytic(const Vec3d& pc) const{
        //TODO:实现四次方程求解器
        return std::nullopt;
    }

    static inline Matx33d alignAxisToZ(Vec3d glass_normal){
        //罗德里格斯公式求解
        //计算旋转轴：v = z × n = [-n2 n1 0]，这里假设n = [n1 n2 n3]^T
        //计算旋转角度：cos(θ) = n · z = n3
        //sin(θ) = ||v|| = sqrt(n1^2 + n2^2)
        //罗德里格斯公式为：
        //R = I + sin(θ) * K + (1 - cos(θ)) * K^2
        //其中K是法矢的叉积矩阵，对应于v归一化后的反对称矩阵，那么罗得里格斯公式可以转化为：
        //R = I + K + (1 - cos(θ)) * K^2 / sin(θ)^2
        //这里K = [0 0 n1, 0 0 n2, -n1 -n2 0]
        Vec3d n = optics::normalize(glass_normal);
        cv::Vec3d z_axis(0, 0, 1);
        if(n.dot(z_axis) < 0) n = -n;

        //计算旋转角度：cos(θ) = n · z = n3
        //sin(θ) = ||v|| = sqrt(n1^2 + n2^2)
        double n1 = n[0], n2 = n[1], n3 = n[2];
        double sintheta = std::sqrt(n1 * n1 + n2 * n2);  //sin(θ) = ||v|| = sqrt(n1^2 + n2^2)
        double costheta = n3;  //cos(θ) = n3

        if(sintheta < 1e-6){
            if(costheta > 0) return Matx33d::eye();  //法矢与z轴几乎重合，无需旋转
            else return Matx33d::diag(Vec3d(1, -1, -1));
        }

        Matx33d K = Matx33d(0, 0, n1, 
                            0, 0, n2, 
                            -n1, -n2, 0);

        Matx33d R_align = Matx33d::eye() + K + (1 - costheta) * K * K / (sintheta * sintheta);
        return R_align.t();
    }

    std::optional<Point2d> MonocularUWCamera::solveGeometric(const Vec3d& pc) const {
        //TODO:几何算法，目前不存在

        return std::nullopt;
    }

    double calibrate(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, 
        const CameraModel& camera_model, MonocularUWCamera& uw_camera, bool verbose)
    {
        //这里采用Agrawal的方法
        //Step1. 求解玻璃的法矢A

        //Step2. 求解
        return .0;
    }
}}