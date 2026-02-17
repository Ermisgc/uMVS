#include "camera/monocular_uwcamera.h"
#include "optics/quartic_solver.h"

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
            case ForwardMethod::ANALYTIC_2:
                return this->solveAnalytic2(pc);
            case ForwardMethod::ITERATE_HELLAY:
                return this->solveIterateHellay(pc);
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

    void MonocularUWCamera::write(const std::string& filename) const {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        write(fs);
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

    void MonocularUWCamera::read(const std::string& filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open camera calibration file");
        }
        read(fs.root());
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

    std::optional<Point2d> MonocularUWCamera::solveAnalytic(const Vec3d& pc) const{
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
            return camera_model_.forwardProject(ps);
        }

        // 计算四次方程的系数
        double k = n3 * n3 / (n1 * n1), a = Rc, b = pc_virtual[2] - dx;
        double A = 1 - k, B = 2 * a * (k - 1), C = a * a + b * b - k * (a * a + dx * dx);
        double D = 2 * a * dx * dx * k, E = -k * a * a * dx * dx;

        std::vector<double> roots = QuarticSolver::solve(A, B, C, D, E);
        double r_best = INT_MAX;
        for(double r : roots){
            if(r > 1e-6 && r > no_fractive_Rc && r < no_fractive_Rc * 1.5 && r < r_best){
                r_best = r;
            }
        }

        if(r_best == INT_MAX) return std::nullopt; //没有有效解
        double scale = r_best / Rc;
        Vec3d ps(pc_virtual[0] * scale, pc_virtual[1] * scale, dx);
        ps = R_align.t() * ps;  // 转换回相机坐标系
        return camera_model_.forwardProject(ps);
    }

    std::optional<Point2d> MonocularUWCamera::solveIterateHellay(const Vec3d& pc) const {
        //坐标系对齐：转换到玻璃垂直坐标系 (与解析法一致)
        Matx33d R_align = alignAxisToZ(glass_normal_);
        Vec3d pc_v = R_align * pc; 

        double Rw = std::sqrt(pc_v[0] * pc_v[0] + pc_v[1] * pc_v[1]);
        double Zw = pc_v[2];

        // 特殊情况处理：点在光轴上
        if (Rw < 1e-6) {
            return camera_model_.forwardProject(R_align.t() * Vec3d(0, 0, d_housing_));
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

        return camera_model_.forwardProject(ps_c);
    }

    std::optional<Point2d> MonocularUWCamera::solveAnalytic2(const Vec3d& pc) const{
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
            return camera_model_.forwardProject(ps);
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
        return camera_model_.forwardProject(ps);
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

    /**
     * @brief 生成水下相机的立体校正映射表
     * 
     * @param real_cam 真实的物理水下相机模型 (Source)
     * @param virtual_cam 理想的虚拟针孔相机模型 (Target, 也就是校正后的样子)
     * @param R_rect 校正旋转矩阵 (R_l 或 R_r，来自 stereoRectify)
     * @param P_rect 校正投影矩阵 (P_l 或 P_r，来自 stereoRectify)
     * @param size 图像尺寸
     * @param z_ref 参考深度 (mm)，建议设为拍摄主体的平均距离
     * @param map1 输出 map_x
     * @param map2 输出 map_y
     */
    void initUWRectifyMap(const NAMESPACE_U3D::camera::MonocularUWCamera& real_cam,
                        const cv::Size& size,
                        const cv::Mat& R_rect,
                        const cv::Mat& P_rect,
                        double z_ref,
                        cv::Mat& map1,
                        cv::Mat& map2) 
    {
        // 初始化映射表
        map1 = cv::Mat(size, CV_32FC1);
        map2 = cv::Mat(size, CV_32FC1);

        // 预先计算 P_rect 的逆或者相关参数，用于从像素恢复归一化坐标
        // P_rect = [f_x, 0, c_x, t_x; 0, f_y, c_y, 0; 0, 0, 1, 0]
        double fx = P_rect.at<double>(0, 0);
        double fy = P_rect.at<double>(1, 1);
        double cx = P_rect.at<double>(0, 2);
        double cy = P_rect.at<double>(1, 2);
        // 注意：stereoRectify 出来的 P 矩阵通常包含平移 t，但在生成 map 时，
        // 我们主要关注旋转后的方向。如果是左图，t通常为0；右图有 t。
        // 更通用的做法是：Pixel -> Norm -> Rotate(R_rect.inv()) -> World
        
        cv::Mat R_inv;
        cv::invert(R_rect, R_inv);

        //左图的映射表
        for (int v = 0; v < size.height; ++v) {
            for (int u = 0; u < size.width; ++u) {
                // 1. 【逆向过程】从校正后的虚拟图像出发
                // 将像素 (u, v) 转换为归一化平面坐标
                double x_norm = (u - cx) / fx;
                double y_norm = (v - cy) / fy;
                
                // 2. 恢复到相机坐标系下的射线方向 (假设 Z=1)
                // 此时的点 P_rect_cam = [x_norm, y_norm, 1]^T 是在"校正后"坐标系下的
                cv::Mat P_rect_cam = (cv::Mat_<double>(3, 1) << x_norm, y_norm, 1.0);
                
                // 3. 旋转回"原始"相机坐标系 (但仍是虚拟针孔的姿态，未考虑折射)
                // P_virt = R_rect^T * P_rect_cam
                cv::Mat P_virt_mat = R_inv * P_rect_cam;
                NAMESPACE_U3D::Vec3d P_virt(P_virt_mat.at<double>(0), 
                                            P_virt_mat.at<double>(1), 
                                            P_virt_mat.at<double>(2));
                
                // 4. 【关键】确定 3D 参考点
                // 我们需要把这个方向延展到参考深度 z_ref
                // 比例 scale = z_ref / P_virt.z
                double scale = z_ref / P_virt[2];
                NAMESPACE_U3D::Vec3d P_world = P_virt * scale;

                // 5. 【正向过程】利用物理水下模型投影回原始图像
                // 这一步会精确计算折射，找到这个 3D 点在原始畸变图像上的位置
                // 使用 ITERATE 方法保证最高精度
                auto pixel_raw_opt = real_cam.forwardProject(P_world, NAMESPACE_U3D::camera::ForwardMethod::ITERATE);

                if (pixel_raw_opt.has_value()) {
                    map1.at<float>(v, u) = static_cast<float>(pixel_raw_opt.value().x);
                    map2.at<float>(v, u) = static_cast<float>(pixel_raw_opt.value().y);
                } else {
                    // 如果投影失败（超视场），填入非法值或边缘值
                    map1.at<float>(v, u) = -1;
                    map2.at<float>(v, u) = -1;
                }
            }
        }


    }

    std::ostream& operator<<(std::ostream& os, const MonocularUWCamera& camera){
        os << "MonocularUWCamera:" << std::endl;
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