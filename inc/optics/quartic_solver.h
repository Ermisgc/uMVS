/**
 * @brief 四元一次方程求解器
 */

#ifndef QUARTIC_SOLVER_H
#define QUARTIC_SOLVER_H
#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include "utils.h"

NAMESPACE_BEGIN{ namespace optics{
    /**
     * @brief 四元一次方程求解器
     * 
     */
    class QuarticSolver {
    private:
        /**
         * @brief 复数类型
         */
        typedef std::complex<double> Complex;

        /**
         * @brief 解一元二次方程 ax^2 + bx + c = 0
         * @param a 系数a
         * @param b 系数b
         * @param c 系数c
         * @param roots 根
         */
        static void solveQuadratic(Complex a, Complex b, Complex c, std::vector<Complex>& roots);

        /**
         * @brief 辅助函数：解一元三次方程 x^3 + ax^2 + bx + c = 0 (归一化后)，使用卡丹公式 (Cardano's Method)
         * @param a 系数a
         * @param b 系数b
         * @param c 系数c
         * @param roots 根
         */
        static void solveCubic(double a, double b, double c, std::vector<Complex>& roots);

    public:
        /**
         * @brief 解四次方程 A x^4 + B x^3 + C x^2 + D x + E = 0，使用费拉里公式 (Ferrari's Method)
         * @param A 系数A
         * @param B 系数B
         * @param C 系数C
         * @param D 系数D
         * @param E 系数E
         * @return std::vector<double> 根
         */
        static std::vector<double> solve(double A, double B, double C, double D, double E);
    };
}}
#endif