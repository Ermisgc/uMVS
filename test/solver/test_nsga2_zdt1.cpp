/**
 * @file test_nsga2_zdt1.cpp
 * @brief 测试NSGA-II在ZDT1问题上的性能
 * @version 0.1
 * @date 2026-02-25
 * @example ./bin/Release/test_nsga2_zdt1.exe
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

/**
 * @brief ZDT1问题
 */
class ZDT1Problem: public IMultiObjectiveProblem{
private:
    int m_numVariables = 30;
    int m_numObjectives = 2;
public:
    ZDT1Problem() = default;
    ~ZDT1Problem() override = default;

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

        cv::Mat objs(N, m_numObjectives, CV_64FC1);
                
        for(int i = 0; i < N; i++){
            const double * x = vars.ptr<double>(i);
            double * f = objs.ptr<double>(i);

            f[0] = x[0]; //函数1：f1(x) = x1
            double sum = 0.0;
            for(int j = 1;j < m_numVariables; ++j){
                sum += x[j];
            }
            double g = 1.0 + 9.0 * sum / (m_numVariables - 1.0);
            f[1] = g * (1.0 - std::sqrt(f[0] / g)); //函数2：f2(x) = g(x) * (1 - sqrt(f1(x) / g(x)))
        }

        if(out_populationObjs.needed()){
            objs.copyTo(out_populationObjs);
        }
    }
};



int main(int argc, char* argv[]) {
    argparse::ArgumentParser parser("test_nsga2_zdt1");
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
    auto problem = std::make_shared<ZDT1Problem>();
    
    //配置
    OptimizerConfig config;
    config.populationSize = population_size;
    config.maxGenerations = max_generations;
    config.crossoverProb = 0.9;
    config.mutationProb = 0.1 / problem->getVariableCount();
    config.crossoverDistribution = 20.0;
    config.mutationDistribution = 20.0;
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
    file << "index,obj1_min,obj1_max,obj2_min,obj2_max" << std::endl;
    for(int i = 0; i < outputPopObjs.rows; i++){
        double minObj1 = outputPopObjs.at<double>(i, 0), maxObj1 = outputPopObjs.at<double>(i, 0);
        double minObj2 = outputPopObjs.at<double>(i, 1), maxObj2 = outputPopObjs.at<double>(i, 1);
        file << i << "," << minObj1 << "," << maxObj1 << "," << minObj2 << "," << maxObj2 << std::endl;
    }
    file.close();
    return 0;
}