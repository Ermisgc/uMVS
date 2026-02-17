#include "optics/quartic_solver.h"
NAMESPACE_BEGIN{ namespace optics{
    inline double polishRoot(double x, double A, double B, double C, double D, double E) {
        for (int i = 0; i < 2; ++i) { // 最后精修的2次迭代足以让精度接近 double 极限
            double f = (((A * x + B) * x + C) * x + D) * x + E;
            double df = ((4.0 * A * x + 3.0 * B) * x + 2.0 * C) * x + D;
            if (std::abs(df) > 1e-2 * std::abs(x)) {  // 若导数绝对值小于 x 的 1%，则认为已收敛
                x -= f / df;
            } else break;
        }
        return x;
    }

    void QuarticSolver::solveQuadratic(Complex a, Complex b, Complex c, std::vector<Complex>& roots) {
        if (abs(a) < 1e-9) { // 变成一次方程 bx + c = 0
            if (abs(b) > 1e-9) roots.push_back(-c / b);
            return;
        }
        Complex delta = b * b - 4.0 * a * c;
        
        // 如果判别式的实部非常小，且虚部也很小，视为重根（delta = 0）
        if (abs(delta.imag()) < 1e-10 && delta.real() < 0 && delta.real() > -1e-9) {
            delta = 0.0; 
        }

        roots.push_back((-b + sqrt(delta)) / (2.0 * a));
        roots.push_back((-b - sqrt(delta)) / (2.0 * a));
    }

    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    void QuarticSolver::solveCubic(double a, double b, double c, std::vector<Complex>& roots) {
        double a2 = a * a;
        double Q = (a2 - 3.0 * b) / 9.0;
        double R = (2.0 * a2 * a - 9.0 * a * b + 27.0 * c) / 54.0;

        double R2 = R * R;
        double Q3 = Q * Q * Q;

        if (R2 < Q3) {
            // 三个实根的情况 (使用三角函数解法避免复数开方的不确定性)
            double ratio = R/sqrt(Q3);
            if(ratio > 1.0) ratio = 1.0;
            else if(ratio < -1.0) ratio = -1.0;

            double theta = acos(ratio);
            double sqrt_Q = -2.0 * sqrt(Q);
            roots.push_back(sqrt_Q * cos(theta / 3.0) - a / 3.0);
            roots.push_back(sqrt_Q * cos((theta + 2.0 * M_PI) / 3.0) - a / 3.0);
            roots.push_back(sqrt_Q * cos((theta - 2.0 * M_PI) / 3.0) - a / 3.0);
        } else {
            // 一个实根，两个共轭复根
            Complex A = -pow(abs(R) + sqrt(R2 - Q3), 1.0 / 3.0);
            if (R < 0) A = -A;
            Complex B = (abs(A) < 1e-9) ? 0.0 : Q / A;

            static const Complex i_unit(0, 1);
            static const double sqrt3 = sqrt(3.0);
            Complex x1 = (A + B) - a / 3.0;
            Complex x2 = -(A + B) / 2.0 - a / 3.0 + i_unit * sqrt3 / 2.0 * (A - B);
            Complex x3 = -(A + B) / 2.0 - a / 3.0 - i_unit * sqrt3 / 2.0 * (A - B);
            
            roots.push_back(x1);
            roots.push_back(x2);
            roots.push_back(x3);
        }
    }

    std::vector<double> QuarticSolver::solve(double A, double B, double C, double D, double E) {
        std::vector<double> realRoots;
        double max_coeff = std::max({std::abs(A), std::abs(B), std::abs(C), std::abs(D), std::abs(E)});
        //考虑降次的特殊情况
        if (abs(A) < 1e-12 * max_coeff) { //降为三次方程
            if(abs(B) < 1e-12 * max_coeff){ //降为二次方程
                std::vector<Complex> temp;
                solveQuadratic(Complex(C, 0), Complex(D, 0), Complex(E, 0), temp);
                for (const auto& root : temp) {
                    if(abs(root.imag()) < 1e-6) realRoots.push_back(root.real());
                }
                return realRoots;
            }
            std::vector<Complex> temp;
            solveCubic(C/B, D/B, E/B, temp);
            for (const auto& root : temp) {
                if(abs(root.imag()) < 1e-6) realRoots.push_back(root.real());
            }
            return realRoots; 
        }

        // 归一化：x^4 + a x^3 + b x^2 + c x + d = 0
        double a = B / A, b = C / A, c = D / A, d = E / A;

        // 换元 x = y - a/4  =>  y^4 + p y^2 + q y + r = 0
        double a2 = a * a;
        double p = b - 0.375 * a2;
        double q = c - 0.5 * a * b + 0.125 * a2 * a;
        double r = d - 0.25 * a * c + 0.0625 * a2 * b - 0.01171875 * a2 * a2;

        std::vector<Complex> y_roots;

        // 特殊情况：如果是双二次方程, 即 q = 0
        if (abs(q) < 1e-14) {
            // y^4 + p y^2 + r = 0  =>  (y^2)^2 + p(y^2) + r = 0
            std::vector<Complex> u_roots;
            solveQuadratic(1.0, p, r, u_roots);
            for (const auto& u : u_roots) {
                y_roots.push_back(sqrt(u));
                y_roots.push_back(-sqrt(u));
            }
            std::cout << "[Debug] Double Quadratic Roots: " << u_roots[0] << ", " << u_roots[1] << std::endl;
        } else {
            // 三次方程：z^3 + 2p z^2 + (p^2 - 4r) z - q^2 = 0
            std::vector<Complex> z_roots;
            solveCubic(2.0 * p, p * p - 4.0 * r, -q * q, z_roots);

            // 寻找最稳健的z (实部最大的根)
            int best_idx = 0;
            double max_real = -1e300;
            for(int i = 0; i < z_roots.size(); i++){
                if(z_roots[i].real() > max_real){
                    max_real = z_roots[i].real();
                    best_idx = i;
                }
            }
            Complex z = z_roots[best_idx]; 
            Complex sqrt_z = sqrt(z);
            Complex q_over_sqrt_z;
            if(abs(z) < 1e-12) q_over_sqrt_z = 0.0;
            else q_over_sqrt_z = q / sqrt_z;

            Complex m = 0.5 * (p + z - q_over_sqrt_z);
            Complex n = 0.5 * (p + z + q_over_sqrt_z);

            solveQuadratic(1.0, sqrt_z, m, y_roots);
            solveQuadratic(1.0, -sqrt_z, n, y_roots);
        }

        // 转换回 x 并筛选实根
        for (const auto& y : y_roots) {
            if (std::abs(y.imag()) > 1e-3 * std::abs(y.real())) continue;  // 若虚部绝对值大于实部的 0.1%，则认为是复数根
            double x = y.real() - a / 4.0;
            
            x = polishRoot(x, A, B, C, D, E);  //用牛顿迭代法迭代一到两次
            
            double val = (((A * x + B) * x + C) * x + D) * x + E;
            if (std::abs(val) < 1e-6 * max_coeff) {
                realRoots.push_back(x);
            }
        }

        return realRoots;
    }
}}