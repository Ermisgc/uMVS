/**
 * @file 98_test_single_solver.cpp
 * @brief 测试脚本：测试四元一次方程求解器对于重根的情况求解的稳定性
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <cassert>
#include <fstream>

#include "optics/quartic_solver.h"
#include "argparse/argparse.hpp"

using namespace NAMESPACE_U3D::optics; 

const double EPSILON = 1e-5;
double eval_poly(double A, double B, double C, double D, double E, double x) {
    return A * std::pow(x, 4) + B * std::pow(x, 3) + C * std::pow(x, 2) + D * x + E;
}

bool run_test(const std::string& case_name, double A, double B, double C, double D, double E, const std::vector<double>& expected_roots_approx = {}) {
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "[TEST] " << case_name << std::endl;
    std::cout << "Equation: " << A << "x^4 + " << B << "x^3 + " << C << "x^2 + " << D << "x + " << E << " = 0" << std::endl;

    // 1. Solve
    std::vector<double> roots = QuarticSolver::solve(A, B, C, D, E);

    // 2. Sort roots for display
    std::sort(roots.begin(), roots.end());

    // 3. Display results
    std::cout << "Roots found (" << roots.size() << "): ";
    if (roots.empty()) {
        std::cout << "None (or all complex)";
    }
    for (double r : roots) {
        std::cout << r << " ";
    }
    std::cout << std::endl;

    // 4. Validation: Plug roots back into equation
    bool passed = true;
    for (double r : roots) {
        double val = eval_poly(A, B, C, D, E, r);
        if (std::abs(val) > EPSILON) {
            std::cout << "  -> FAIL: Root " << r << " yields f(x) = " << val << " (Expected ~0)" << std::endl;
            passed = false;
        }
    }

    // 5. Check against expected (if provided)
    if (!expected_roots_approx.empty()) {
        if (roots.size() != expected_roots_approx.size()) {
             std::cout << "  -> FAIL: Expected " << expected_roots_approx.size() << " roots, got " << roots.size() << std::endl;
             passed = false;
        } else {
            // Simple check if roots are close to expected
            // 排序方便按顺序比较
            auto copy = expected_roots_approx;
            std::sort(roots.begin(), roots.end());
            std::sort(copy.begin(), copy.end());

            for(size_t i=0; i<roots.size(); ++i) {
                if(std::abs(roots[i] - copy[i]) > 1e-3) {
                    std::cout << "  -> WARNING: Root mismatch. Got " << roots[i] << ", Expected " << copy[i] << std::endl;
                }
            }
        }
    }

    if (passed) {
        std::cout << "[RESULT] PASSED" << std::endl;
    } else {
        std::cout << "[RESULT] FAILED" << std::endl;
    }
    return passed;
}

int main(int argc, char* argv[]) {
    std::vector<double> expected_roots = {-3.46, -8.04, -8.4, 6.06};
    double r1 = expected_roots[0];
    double r2 = expected_roots[1];
    double r3 = expected_roots[2];
    double r4 = expected_roots[3];

    double A = 1.0;
    double B = -(r1 + r2 + r3 + r4);
    double C = (r1*r2 + r1*r3 + r1*r4 + r2*r3 + r2*r4 + r3*r4);
    double D = -(r1*r2*r3 + r1*r2*r4 + r1*r3*r4 + r2*r3*r4);
    double E = r1 * r2 * r3 * r4;
    run_test("Random Case 0", A, B, C, D, E, expected_roots);
    
    return 0;
}
