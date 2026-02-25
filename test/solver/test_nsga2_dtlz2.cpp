/**
 * @file test_nsga2_dtlz2.cpp
 * @brief 测试NSGA-II在DTLZ2问题上的性能，DTLZ2问题是一个三维线性超平面
 * @version 0.1
 * @date 2026-02-25
 * @example ./bin/Release/test_nsga2_dtlz2.exe
 * @copyright Copyright (c) 2026
 * 
 */

#include "optics/nsga2.h"
#include <opencv2/opencv.hpp>
#include "cmath"
#include <fstream>
#include "argparse/argparse.hpp"

using namespace NAMESPACE_U3D;
using namespace NAMESPACE_U3D::optics;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief DTLZ2问题，DTLZ2问题是一个三维球面
 */
class DTLZ2Problem: public IMultiObjectiveProblem{
private:
    int m_numVariables = 12;
    int m_numObjectives = 3;
public:
    DTLZ2Problem() = default;
    ~DTLZ2Problem() override = default;

    int getVariableCount() const override{
        return m_numVariables;
    }

    int getObjectiveCount() const override{
        return m_numObjectives;
    }

    void getBounds(cv::Mat& lowerBounds, cv::Mat& upperBounds) const override{
        lowerBounds = cv::Mat::ones(1, m_numVariables, CV_64FC1) * 0.0;
        upperBounds = cv::Mat::ones(1, m_numVariables, CV_64FC1) * 1.0;
    }

    void evaluate(cv::InputArray populationVars, cv::OutputArray out_populationObjs) const override{
        auto vars = populationVars.getMat();
        int N = vars.rows;
        int k = m_numVariables - m_numObjectives + 1;
        CV_Assert(k >= 0);
        cv::Mat objs(N, m_numObjectives, CV_64FC1);
                
        for(int i = 0; i < N; i++){
            const double * x = vars.ptr<double>(i);
            double * f = objs.ptr<double>(i);

            double sum = 0.0;
            for(int j = m_numObjectives - 1; j < m_numVariables; ++j){
                double x_j = x[j] - 0.5;
                sum += x_j * x_j;
            }
            double g = sum;
            f[0] = (1 + g) * cos(0.5 * M_PI * x[0]) * cos(0.5 * M_PI * x[1]);
            f[1] = (1 + g) * cos(0.5 * M_PI * x[0]) * sin(0.5 * M_PI * x[1]);
            f[2] = (1 + g) * sin(0.5 * M_PI * x[0]);
        }

        if(out_populationObjs.needed()){
            objs.copyTo(out_populationObjs);
        }
    }
};



int main(int argc, char* argv[]) {
    argparse::ArgumentParser parser("test_nsga2_dtlz2");
    parser.add_argument("--random_seed", "-r").default_value(42).help("Random seed for testing").scan<'i', int>();
    parser.add_argument("--population_size", "-n").default_value(1000).help("Population size for testing").scan<'i', int>();
    parser.add_argument("--max_generations", "-g").default_value(250).help("Max generations for testing").scan<'i', int>();
    parser.add_argument("--out_file", "-o").default_value("result.csv").help("Output file for testing");
    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    int random_seed = parser.get<int>("--random_seed");
    int population_size = parser.get<int>("--population_size");
    int max_generations = parser.get<int>("--max_generations");
    std::string out_file = parser.get<std::string>("--out_file");

    //问题
    auto problem = std::make_shared<DTLZ2Problem>();
    
    //配置
    OptimizerConfig config;
    config.populationSize = population_size;
    config.maxGenerations = max_generations;
    config.crossoverProb = 0.9;
    config.mutationProb = 0.1 / problem->getVariableCount();
    config.crossoverDistribution = 10.0;
    config.mutationDistribution = 15.0;
    config.randomSeed = random_seed;

    //回调函数
    auto callback = [](int gen, const cv::Mat& popObjs, const cv::Mat& popVars){
        double minObj1, maxObj1;
        double minObj2, maxObj2;
        cv::minMaxLoc(popObjs.col(0), &minObj1, &maxObj1);  
        cv::minMaxLoc(popObjs.col(1), &minObj2, &maxObj2);  
        std::cout << "\rgen: " << gen + 1 
                  << " | Obj 1 Min: " << minObj1 << " Max: " << maxObj1 
                  << " | Obj 2 Min: " << minObj2 << " Max: " << maxObj2 << std::flush;
    };

    NSGAOptimizer optimizer(problem, config);
    optimizer.setIterationCallback(callback);
    cv::Mat outputPopObjs, outputPopVars;
    outputPopVars = optimizer.optimize(outputPopObjs);

    std::cout << "优化完成，正在将结果写入" << out_file << std::endl;
    std::ofstream file(out_file);
    file << "index,obj1_min,obj1_max,obj2_min,obj2_max,obj3_min,obj3_max" << std::endl;
    for(int i = 0; i < outputPopObjs.rows; i++){
        double minObj1 = outputPopObjs.at<double>(i, 0), maxObj1 = outputPopObjs.at<double>(i, 0);
        double minObj2 = outputPopObjs.at<double>(i, 1), maxObj2 = outputPopObjs.at<double>(i, 1);
        double minObj3 = outputPopObjs.at<double>(i, 2), maxObj3 = outputPopObjs.at<double>(i, 2);
        file << i << "," << minObj1 << "," << maxObj1 << "," << minObj2 << "," << maxObj2 << "," << minObj3 << "," << maxObj3 << std::endl;
    }
    file.close();
    return 0;
}