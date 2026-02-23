#include "camera/monouwcamera.h"
#include "optics/quartic_solver.h"
#include "optics/geometry.h"
#include <Eigen/Core>
#include <Eigen/Eigen>

NAMESPACE_BEGIN { namespace camera {
    MonoUWCamera::MonoUWCamera(const MonoCamera& camera_model,const optics::RefractiveInterface& p1, const optics::RefractiveInterface& p2){
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

    MonoUWCamera::MonoUWCamera(const MonoCamera& camera_model, double d1, double d2, double n0, double n1, double n2, const Vec3d& glass_normal):
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

    std::optional<Point2d> MonoUWCamera::forwardProject(const Vec3d& pw, ForwardMethod method) const{
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
            case ForwardMethod::ANALYTIC_2:
                return this->solveAnalytic2(pc);
            case ForwardMethod::ITERATE_HELLAY:
                return this->solveIterateHellay(pc);
            case ForwardMethod::ANALYTIC_ACCURATE:
                return this->solveAnalyticAccurate(pc);
            default:
                return this->solveIterative(pc);
        }
        return std::nullopt;
    }

    std::optional<Vec3d> MonoUWCamera::backwardProject(const Vec2d& uv, double depth) const{
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

    void MonoUWCamera::write(cv::FileStorage& fs) const {
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

    void MonoUWCamera::write(const std::string& filename) const {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        write(fs);
    }

    void MonoUWCamera::read(const cv::FileNode& fs) {
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

    void MonoUWCamera::read(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        read(fs.root());
    }

    std::optional<Point2d> MonoUWCamera::solveIterative(const Vec3d& pc) const{
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

    Matx33d alignAxisToZ(Vec3d glass_normal){
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

    std::optional<Point2d> MonoUWCamera::solveAnalytic(const Vec3d& pc) const{
        // 将双层折射等效为单层折射，然后调用四次方程求解器求解
        // 共分为四步：
        // 1. 先将点pc转换到玻璃垂直坐标系
        // 2. 用等效厚度法计算四次方程的系数
        // 3. 调用四次方程求解器求解
        // 4. 把求出的三维折射点转换回原来的相机坐标系
        // 坐标系示意图如下：
        //真实相机坐标系 (C)           虚拟/对齐坐标系 (V)
        //     ^ Y_c                        ^ Y_v
        //     |  / 倾斜玻璃                |  / 垂直玻璃 (Z_v = d)L
        //     | /                          | /
        // (0,0) O ------> Z_c          (0,0) O ------> Z_v (法线方向)
        //     \                            \
        //      \ 光线                       \ 光线
        //       \                            \
        //       P_c (目标点)                P_v (变换后的点)

        Matx33d R_align = alignAxisToZ(glass_normal_);
        Vec3d pc_virtual = R_align * pc;  // 将点pc转换到玻璃坐标系

        double n1 = refraction_planes_[0].n1;
        double n2 = refraction_planes_[0].n2;
        double n3 = refraction_planes_[1].n2;

        double dx = d_housing_ + d_glass_ * n1 * (n3 - n2) / (n2 * (n3 - n1));
        using optics::QuarticSolver;

        double Rc = std::sqrt(pc_virtual[0] * pc_virtual[0] + pc_virtual[1] * pc_virtual[1]);
        // std::cout << "Rc: " << Rc << std::endl;
        double no_fractive_Rc = Rc * dx / pc_virtual[2];  //用于筛选最后的生成根
        if(Rc < 1e-6){  //特殊情况，如果半径是0，可以直接知道位置了，这里给定一个与dx相关的比例系数，避免在中心时除以Rc会得到极端值
            Vec3d ps = R_align.t() * Vec3d(0, 0, dx);
            return camera_model_.camToPixel(ps);
        }

        // 计算四次方程的系数
        double k = n3 * n3 / (n1 * n1), a = Rc, b = pc_virtual[2] - dx;
        double A = 1 - k, B = 2 * a * (k - 1), C = a * a + b * b - k * (a * a + dx * dx);
        double D = 2 * a * dx * dx * k, E = -k * a * a * dx * dx;

        std::vector<double> roots = QuarticSolver::solve(A, B, C, D, E);
        double r_best = INT_MAX;
        for(double r : roots){
            if(r > 1e-6 && r > no_fractive_Rc && r < no_fractive_Rc * 2 && r < r_best){
                r_best = r;
            }
        }

        if(r_best == INT_MAX) return std::nullopt; //没有有效解
        double scale = r_best / Rc;
        Vec3d ps(pc_virtual[0] * scale, pc_virtual[1] * scale, dx);
        ps = R_align.t() * ps;  // 转换回相机坐标系
        return camera_model_.camToPixel(ps);
    }

    std::optional<Point2d> MonoUWCamera::solveIterateHellay(const Vec3d& pc) const {
        //坐标系对齐：转换到玻璃垂直坐标系 (与解析法一致)
        Matx33d R_align = alignAxisToZ(glass_normal_);
        Vec3d pc_v = R_align * pc; 

        double Rw = std::sqrt(pc_v[0] * pc_v[0] + pc_v[1] * pc_v[1]);
        double Zw = pc_v[2];

        // 特殊情况处理：点在光轴上
        if (Rw < 1e-6) {
            return camera_model_.camToPixel(R_align.t() * Vec3d(0, 0, d_housing_));
        }

        double r = (Rw * d_housing_) / Zw; // 这是一个极简初值，建议使用解析法算出的 r_best
        
        // --- 迭代准备 ---
        double n1 = refraction_planes_[0].n1; // 空气
        double n2 = refraction_planes_[0].n2; // 玻璃
        double n3 = refraction_planes_[1].n2; // 水
        double da = d_housing_;
        double dg = d_glass_;
        double dw = Zw - da - dg; // 水层深度

        double mu_g = n1 / n2;
        double mu_w = n1 / n3;
        double da2 = da * da;

        // Halley 迭代主循环
        for (int i = 0; i < 5; ++i) { // 通常 2-3 次即可收敛
            // 中间变量 S = sqrt((1-mu^2)r^2 + da^2)
            double r2 = r * r;
            double Sg2 = (1 - mu_g * mu_g) * r2 + da2;
            double Sw2 = (1 - mu_w * mu_w) * r2 + da2;
            if (Sg2 < 0 || Sw2 < 0) break;

            double Sg = std::sqrt(Sg2);
            double Sw = std::sqrt(Sw2);

            // F(r) = r + dg*tan(theta_g) + dw*tan(theta_w) - Rw
            // 其中 tan(theta_i) = mu_i * r / S_i
            double f_g = dg * mu_g * r / Sg;
            double f_w = dw * mu_w * r / Sw;
            double F = r + f_g + f_w - Rw;

            if (std::abs(F) < 1e-9) break;

            // 一阶导数 F'(r) = 1 + sum( d_i * mu_i * da^2 / S_i^3 )
            double dF = 1.0 + (dg * mu_g * da2) / (Sg2 * Sg) 
                            + (dw * mu_w * da2) / (Sw2 * Sw);

            // 二阶导数 F''(r) = -sum( 3 * d_i * mu_i * da^2 * (1-mu_i^2) * r / S_i^5 )
            double ddF = -(3.0 * dg * mu_g * da2 * (1 - mu_g * mu_g) * r) / (Sg2 * Sg2 * Sg)
                        -(3.0 * dw * mu_w * da2 * (1 - mu_w * mu_w) * r) / (Sw2 * Sw2 * Sw);

            // Halley 更新公式
            double delta = (2.0 * F * dF) / (2.0 * dF * dF - F * ddF);
            r = r - delta;

            if (std::abs(delta) < 1e-3 * r) break;
        }

        // 将迭代后的径向距离 r 转换回相机坐标系
        double scale = r / Rw;
        Vec3d ps_v(pc_v[0] * scale, pc_v[1] * scale, da);
        Vec3d ps_c = R_align.t() * ps_v;

        return camera_model_.camToPixel(ps_c);
    }

    std::optional<Point2d> MonoUWCamera::solveAnalytic2(const Vec3d& pc) const{
        Matx33d R_align = alignAxisToZ(glass_normal_);
        Vec3d pc_virtual = R_align * pc;  // 将点pc转换到玻璃坐标系

        double n1 = refraction_planes_[0].n1;
        double n2 = refraction_planes_[0].n2;
        double n3 = refraction_planes_[1].n2;

        double dx = d_housing_ + d_glass_ * n1 * (n3 - n2) / (n2 * (n3 - n1));
        using optics::QuarticSolver;

        double Rc = std::sqrt(pc_virtual[0] * pc_virtual[0] + pc_virtual[1] * pc_virtual[1]);
        // std::cout << "Rc: " << Rc << std::endl;
        double no_fractive_Rc = Rc * dx / pc_virtual[2];  //用于筛选最后的生成根
        if(Rc < 1e-6){  //特殊情况，如果半径是0，可以直接知道位置了，这里给定一个与dx相关的比例系数，避免在中心时除以Rc会得到极端值
            Vec3d ps = R_align.t() * Vec3d(0, 0, dx);
            return camera_model_.camToPixel(ps);
        }

        // 计算四次方程的系数
        double k = n3 * n3 / (n1 * n1), a = Rc, b = pc_virtual[2] - dx;
        double A = 1 - k, B = 2 * a * (k - 1), C = a * a + b * b - k * (a * a + dx * dx);
        double D = 2 * a * dx * dx * k, E = -k * a * a * dx * dx;

        std::vector<double> coeffs = {E, D, C, B, A};
        std::vector<cv::Vec2d> roots;
        cv::solvePoly(coeffs, roots);
        std::vector<double> readRoots;
        for(const auto & r : roots){
            if(std::abs(r[1]) < 1e-6) readRoots.push_back(r[0]);
        }
        double r_best = INT_MAX;
        for(double r : readRoots){
            if(r > 1e-6 && r > no_fractive_Rc && r < no_fractive_Rc * 1.5 && r < r_best){
                r_best = r;
            }
        }

        if(r_best == INT_MAX) return std::nullopt; //没有有效解
        double scale = r_best / Rc;
        Vec3d ps(pc_virtual[0] * scale, pc_virtual[1] * scale, dx);
        ps = R_align.t() * ps;  // 转换回相机坐标系
        return camera_model_.camToPixel(ps);
    }

    /**
     * @brief 编译单元内部函数，求解12次多项式
     * @param coeffs 
     * @return std::vector<std::complex<double>> 
     */
    static std::vector<std::complex<double>> solvePolynomialTwelve(const Eigen::VectorXd& coeffs){
        int n = coeffs.size() - 1;
        int start = 0;
        while (start < n && abs(coeffs(start)) < 1e-15) start++;

        if (start >= n) return std::vector<std::complex<double>>();  // 所有系数都是0

        int degree = n - start;  // 多项式的次数
        if (degree == 0) return std::vector<std::complex<double>>();  // 伴随矩阵的秩为0

        Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
        double leadCoeff = coeffs(start);

        for (int i = 0; i < degree; i++) {
            companion(0, i) = -coeffs(start + i + 1) / leadCoeff;
        }

        for (int i = 1; i < degree; i++) {
            companion(i, i - 1) = 1.0;
        }

        Eigen::EigenSolver<Eigen::MatrixXd> solver(companion);
        auto eigenvalues = solver.eigenvalues();

        std::vector<std::complex<double>> roots;
        for (int i = 0; i < eigenvalues.size(); i++) {
            roots.push_back(eigenvalues(i));
        }

        return roots;
    }

    std::optional<Point2d> MonoUWCamera::solveAnalyticAccurate(const Vec3d& pc) const{
        assert(n_air_ > 1e-6, "invalid n_air_");
        double d = d_housing_ + d_glass_;
        double d2 = d_housing_;
        Eigen::Vector3d z1 = {glass_normal_[0], glass_normal_[1], glass_normal_[2]};  //玻璃法向方向，也是POR中的z1方向
        if(z1[2] < 0) z1 = -z1;
        z1.normalize();

        Eigen::Vector3d p(pc(0), pc(1), pc(2));
        Eigen::Vector3d POR = z1.cross(p);
        Eigen::Vector3d z2 = POR.cross(z1);
        z2.normalize();
        
        if(POR.norm() < 1e-10) {
            //此时直接取法玻璃法矢与玻璃的成像夹角在成像平面的投影
            double scale = d_housing_ / z1(2);
            return camera_model_.camToPixel({z1(0) * scale, z1(1) * scale, z1(2) * scale});
        }

        Eigen::Vector2d mu = Eigen::Vector2d(n_water_ / n_air_, n_glass_ / n_air_);

        // 将3D点投影到POR上
        double v = p.dot(z1);
        double u = std::abs(p.dot(z2));

        // 计算多项式系数
        double mu1_2 = mu(0) * mu(0);
        double mu2_2 = mu(1) * mu(1);
        double d_2 = d * d;
        double d_4 = d_2 * d_2;
        double d_8 = d_4 * d_4;
        double u_2 = u * u;
        double u_3 = u_2 * u;
        double u_4 = u_2 * u_2;

        double term1 = mu1_2 - 1.0;
        double term2 = mu2_2 - 1.0;
        double term1_2 = term1 * term1;
        double term2_2 = term2 * term2;

        double dd = d - d2;
        double dd_2 = dd * dd;
        double dv = d2 - v;
        double dv_2 = dv * dv;

        // 计算中间项 A, B, C
        double A = term2 * (u_2 * term1 + d_2 * mu1_2) -
                term2 * dd_2 -
                term1 * dv_2 +
                d_2 * mu2_2 * term1;

        double B = d_2 * mu2_2 * (u_2 * term1 + d_2 * mu1_2) -
                d_2 * mu2_2 * dd_2 -
                d_2 * mu1_2 * dv_2 +
                d_2 * mu1_2 * u_2 * term2;

        double C = 2.0 * d_2 * mu1_2 * u * term2 +
                2.0 * d_2 * mu2_2 * u * term1;

        Eigen::VectorXd coeffs(13);

        coeffs(0) = term1_2 * term2_2;
        coeffs(1) = -4.0 * u * term1_2 * term2_2;
        coeffs(2) = 4.0 * u_2 * term1_2 * term2_2 + 2.0 * term1 * term2 * A;
        coeffs(3) = -2.0 * term1 * term2 * C - 4.0 * u * term1 * term2 * A;
        coeffs(4) = A * A + 2.0 * term1 * term2 * B - 4.0 * term1 * term2 * dd_2 * dv_2 + 4.0 * u * term1 * term2 * C;
        coeffs(5) = -2.0 * C * A - 4.0 * u * term1 * term2 * B - 4.0 * d_4 * mu1_2 * mu2_2 * u * term1 * term2;
        coeffs(6) = 2.0 * A * B + C * C - 4.0 * dd_2 * (d_2 * mu1_2 * term2 + d_2 * mu2_2 * term1) * dv_2 + 10.0 * d_4 * mu1_2 * mu2_2 * u_2 * term1 * term2;
        coeffs(7) = -2.0 * C * B - 4.0 * d_4 * mu1_2 * mu2_2 * u * A - 4.0 * d_4 * mu1_2 * mu2_2 * u_3 * term1 * term2;
        coeffs(8) = B * B + 2.0 * d_4 * mu1_2 * mu2_2 * u_2 * A - 4.0 * d_4 * mu1_2 * mu2_2 * dd_2 * dv_2 + 4.0 * d_4 * mu1_2 * mu2_2 * u * C;
        coeffs(9) = -2.0 * d_4 * mu1_2 * mu2_2 * u_2 * C - 4.0 * d_4 * mu1_2 * mu2_2 * u * B;
        coeffs(10) = 4.0 * d_8 * mu1_2 * mu1_2 * mu2_2 * mu2_2 * u_2 + 2.0 * d_4 * mu1_2 * mu2_2 * u_2 * B;
        coeffs(11) = -4.0 * d_8 * mu1_2 * mu1_2 * mu2_2 * mu2_2 * u_3;
        coeffs(12) = d_8 * mu1_2 * mu1_2 * mu2_2 * mu2_2 * u_4;

        auto roots = solvePolynomialTwelve(coeffs); 

        double best_dist = std::numeric_limits<double>::max();  //设定的初始距离，如果解太离谱，也视作没找到
        Point2d best_pixel;
        bool found = false;

        for (const auto& root : roots) {
            if (std::abs(root.imag()) >= 1e-7) continue;
            double r = root.real(); 
            Eigen::Vector3d q_air = d2 * z1 + r * z2;
            Eigen::Vector3d v_air = q_air.normalized();
            optics::Ray ray_air(Vec3d(0, 0, 0), Vec3d(v_air(0), v_air(1), v_air(2)));
            auto ray = optics::traceThroughInterfaces(ray_air, this->refraction_planes_);  //水中的折射射线
            //然后比较点到折射射线的距离：
            if(!ray.has_value()) continue;
            optics::Ray ray_water = ray.value();
            double dist = optics::distPointToLine(pc, ray_water);
            if(dist < best_dist){
                auto pixel = camera_model_.camToPixel({q_air(0), q_air(1), q_air(2)});
                if(!pixel.has_value()) continue;
                best_pixel = pixel.value();
                best_dist = dist;
                found = true;
            }
        }

        if (found) return best_pixel;
        return std::nullopt;
    }

    std::optional<Point2d> MonoUWCamera::solveGeometric(const Vec3d& pc) const {
        //TODO:几何算法，目前不存在

        return std::nullopt;
    }

    double calibrate(const std::vector<std::string>& imageFiles, cv::Size boardSize, double squareSize, 
        const MonoCamera& camera_model, MonoUWCamera& uw_camera, bool verbose)
    {
        //这里采用Agrawal的方法
        //Step1. 求解玻璃的法矢A

        //Step2. 求解
        return .0;
    }

    std::ostream& operator<<(std::ostream& os, const MonoUWCamera& camera){
        os << "MonoUWCamera:" << std::endl;
        os << "camera_model:" << std::endl << camera.camera_model_ << std::endl;
        os << "n_air_:" << camera.n_air_ << std::endl;
        os << "n_glass_:" << camera.n_glass_ << std::endl;
        os << "n_water_:" << camera.n_water_ << std::endl;
        os << "d_housing_:" << camera.d_housing_ << std::endl;
        os << "d_glass_:" << camera.d_glass_ << std::endl;
        os << "glass_normal_:" << camera.glass_normal_ << std::endl;
        return os;
    }
}}