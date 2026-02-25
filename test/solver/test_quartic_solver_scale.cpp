/**
 * @file test_quartic_solver_scale.cpp
 * @brief 测试脚本：大规模测试四元一次方程求解器
 * @example
 *     test_quartic_solver_scale --random_number 42 --test_count 100
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

/**
 * @brief Helper function to evaluate polynomial value: f(x) = Ax^4 + Bx^3 + Cx^2 + Dx + E
 */
double eval_poly(double A, double B, double C, double D, double E, double x) {
    return A * std::pow(x, 4) + B * std::pow(x, 3) + C * std::pow(x, 2) + D * x + E;
}

/**
 * @brief Test case runner
 */
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
    argparse::ArgumentParser parser("test_quartic_solver_scale");
    parser.add_argument("--random_number", "-r").default_value(42).help("Random number for testing").scan<'i', int>();
    parser.add_argument("--test_count", "-n").default_value(100).help("Number of test cases to run").scan<'i', int>();
    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    int random_number = parser.get<int>("--random_number");
    int test_count = parser.get<int>("--test_count");

    std::cout << "==================================================" << std::endl;
    std::cout << "       QuarticSolver Verification Suite           " << std::endl;
    std::cout << "==================================================" << std::endl;

    // Case 1: Simple Bi-quadratic (Depressed)
    // (x^2 - 1)(x^2 - 4) = x^4 - 5x^2 + 4 = 0
    // Roots: -2, -1, 1, 2
    // Tests the special q=0 optimization branch
    std::vector<double> exp1 = {-2.0, -1.0, 1.0, 2.0};
    run_test("Bi-quadratic (Standard)", 1.0, 0.0, -5.0, 0.0, 4.0, exp1);

    // Case 2: General Case (4 distinct real roots)
    // (x-1)(x-2)(x-3)(x+4) 
    // = (x^2 - 3x + 2)(x^2 + x - 12)
    // = x^4 + x^3 - 12x^2 - 3x^3 - 3x^2 + 36x + 2x^2 + 2x - 24
    // = x^4 - 2x^3 - 13x^2 + 38x - 24 = 0
    // Roots: -4, 1, 2, 3
    std::vector<double> exp2 = {-4.0, 1.0, 2.0, 3.0};
    run_test("General Case (4 Real Roots)", 1.0, -2.0, -13.0, 38.0, -24.0, exp2);

    // Case 3: Mixed Roots (2 Real, 2 Complex)
    // (x^2 + 1)(x - 2)(x + 3)
    // = (x^2 + 1)(x^2 + x - 6)
    // = x^4 + x^3 - 6x^2 + x^2 + x - 6
    // = x^4 + x^3 - 5x^2 + x - 6 = 0
    // Real Roots: -3, 2
    std::vector<double> exp3 = {-3.0, 2.0};
    run_test("Mixed Roots (2 Real, 2 Complex)", 1.0, 1.0, -5.0, 1.0, -6.0, exp3);

    // Case 4: Double Roots / Multiplicity
    // (x - 2)^2 * (x + 2)^2 = (x^2 - 4)^2 = x^4 - 8x^2 + 16 = 0
    // Roots: -2, -2, 2, 2
    // Note: Numerical solvers might return distinct roots very close to each other (e.g. 1.9999 and 2.0001)
    // or just two roots depending on implementation logic. 
    // This tests stability.
    run_test("Double Roots (Multiplicity)", 1.0, 0.0, -8.0, 0.0, 16.0);

    // Case 5: No Real Roots
    // (x^2 + 1)(x^2 + 2) = x^4 + 3x^2 + 2 = 0
    // Roots: None
    std::vector<double> exp5 = {};
    run_test("No Real Roots", 1.0, 0.0, 3.0, 0.0, 2.0, exp5);

    // Case 6: Optical Simulation Context (Small coefficients)
    // Simulating a scenario closer to your actual use case (normalized geometry)
    // Example conceptual values
    run_test("Optical Physics Context", 0.8, -0.1, 0.05, 0.002, -0.01);

    std::cout << "==================================================" << std::endl;
    std::cout << "Tests Completed." << std::endl;

    srand(random_number);
    int success_count = 0;
    for(int i=0; i<test_count; ++i) {
        std::vector<double> expected_roots(4);
        for(int j = 0; j < 4; ++j) {
            expected_roots[j] = (rand() % 2000 - 1000) / 100.0; // 范围 [-10, 10]
        }
        double r1 = expected_roots[0];
        double r2 = expected_roots[1];
        double r3 = expected_roots[2];
        double r4 = expected_roots[3];

        double A = 1.0;
        double B = -(r1 + r2 + r3 + r4);
        double C = (r1*r2 + r1*r3 + r1*r4 + r2*r3 + r2*r4 + r3*r4);
        double D = -(r1*r2*r3 + r1*r2*r4 + r1*r3*r4 + r2*r3*r4);
        double E = r1 * r2 * r3 * r4;
        if(run_test("Random Case " + std::to_string(i), A, B, C, D, E, expected_roots)) {
            success_count++;
        } else {
            //将错误的测试用例保存到文件里面
             std::ofstream outfile("failed_test_case.txt", std::ios_base::app);
             outfile << "Random Case " << i << std::endl;
             outfile << "A: " << A << ", B: " << B << ", C: " << C << ", D: " << D << ", E: " << E << std::endl;
             outfile << "Expected Roots: " << expected_roots[0] << ", " << expected_roots[1] << ", " << expected_roots[2] << ", " << expected_roots[3] << std::endl;
             outfile << std::endl;
        }
    }

    std::cout << "==================================================" << std::endl;
    std::cout << "Random Tests Completed. Success Rate: " << success_count << "/" << test_count << std::endl;
    
    return 0;
}
