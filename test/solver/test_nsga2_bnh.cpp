/**
 * @file test_nsga2_bnh.cpp
 * @brief 测试NSGA-II在BNH问题上的性能
 * @version 0.1
 * @date 2026-02-25
 * @example ./bin/Release/test_nsga2_bnh.exe
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
 * @brief BNH问题，该问题只有2个变量和2个目标函数，它的特点是：离散可行域，能够验证算法能否避开不可行域，并在断裂的边界上收敛
 */
class BNHProblem: public IMultiObjectiveProblem{
private:
    int m_numVariables = 2;
    int m_numObjectives = 2;
public:
    BNHProblem() = default;
    ~BNHProblem() override = default;

    int getVariableCount() const override{
        return m_numVariables;
    }

    int getObjectiveCount() const override{
        return m_numObjectives;
    }

    void getBounds(cv::Mat& lowerBounds, cv::Mat& upperBounds) const override{
        lowerBounds = (cv::Mat_<double>(1, 2) << 0.0, 0.0);
        upperBounds = (cv::Mat_<double>(1, 2) << 5.0, 3.0);
    }

    void evaluate(cv::InputArray populationVars, cv::OutputArray out_populationObjs) const override{
        auto vars = populationVars.getMat();
        int N = vars.rows;
        cv::Mat objs(N, m_numObjectives, CV_64FC1);
                
        for(int i = 0; i < N; i++){
            double x1 = vars.at<double>(i, 0);
            double x2 = vars.at<double>(i, 1);

            double * f = objs.ptr<double>(i);

            f[0] = 4.0 * x1 * x1 + 4.0 * x2 * x2; //函数1：f1(x) = 4 * x1^2 + 4 * x2^2
            f[1] = std::pow(x1 - 5.0, 2.0) + std::pow(x2 - 5.0, 2.0); //函数2：f2(x) = (x1 - 5)^2 + (x2 - 5)^2
            double g1 = std::pow(x1 - 5.0, 2.0) + x2 * x2 - 25.0;
            double g2 = 7.7 - (std::pow(x1 - 8.0, 2.0) + std::pow(x2 + 3.0, 2.0));

            double cv = 0.0;
            if(g1 > 0.0) cv += g1;
            if(g2 > 0.0) cv += g2;
            double penalty = 1000.0 * cv;

            f[0] += penalty;
            f[1] += penalty;
        }

        if(out_populationObjs.needed()){
            objs.copyTo(out_populationObjs);
        }
    }
};

int main(int argc, char* argv[]) {
    argparse::ArgumentParser parser("test_nsga2_bnh");
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
    auto problem = std::make_shared<BNHProblem>();
    
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