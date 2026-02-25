#ifndef NSGA2_H
#define NSGA2_H
#include "utils.h"
#include <opencv2/core.hpp>
#include <functional>
#include <memory>

NAMESPACE_BEGIN{ namespace optics{
    /**
     * @brief 多目标函数定义接口
     */
    class IMultiObjectiveProblem{
    public:
        virtual ~IMultiObjectiveProblem() = default;

        /**
         * @brief 获取变量维度
         * @return int 变量维度
         */
        virtual int getVariableCount() const = 0;

        /**
         * @brief 获取目标函数维度
         * @return int 目标函数维度
         */
        virtual int getObjectiveCount() const = 0;

        /**
         * @brief 获取变量边界
         * @param lowerBounds 下边界矩阵，大小为1x变量维度
         * @param upperBounds 上边界矩阵，大小为1x变量维度
         */
        virtual void getBounds(cv::Mat& lowerBounds, cv::Mat& upperBounds) const = 0;

        /**
         * @brief 核心评估函数：输入种群变量矩阵 (N x D)，输出对应的目标函数矩阵 (N x M)
         * @param populationVars 种群变量矩阵，大小为NxD，N为种群数量，D为变量维度
         * @param populationObjs 目标函数矩阵，大小为NxM，N为种群数量，M为目标函数维度
         */
        virtual void evaluate(cv::InputArray populationVars, cv::OutputArray out_populationObjs) const = 0;
    };

    /**
     * @brief NSGA-II 优化器的超参数
     * @note 所有超参数均为可选，未指定时会使用默认值
     * @param populationSize 种群大小，即数学表示的符号N，种群越大，算法单次迭代的搜索对象就越多，
     * 但是同时由于复杂度也与它相关，因此种群大小不宜过大，也不宜过小。
     * @param maxGenerations 最大迭代次数
     * @param mutationProb 变异概率，变异概率p_m决定了每个基因发生变异的几率。
     * 如果p_m太大，算法会退化为随机搜索，因为每个基因都会发生频繁的变异；
     * 如果p_m太小，算法容易陷入局部最优。学术界公认的最佳默认值是p_m = 1 / D，即平均每个个体每次只期望变异1个变量。
     * 这个变量的合法值为 (0, 1]，结构体中的默认值为-1，但是NSGA-II优化器识别到不再范围内时会根据变量维度自动计算默认值为1 / D。
     * @param mutationDistribution 变异分布指数 (eta_m)
     * @param crossoverProb 交叉概率 (P_c)
     * @param crossoverDistribution 交叉分布指数 (eta_c)
     * @param randomSeed 随机种子，用于初始化随机数生成器，用于确保结果可复现
     */
    struct OptimizerConfig{
        int populationSize = 100;         ///< 种群大小 (N)
        int maxGenerations = 250;         ///< 最大迭代次数
        double crossoverProb = 0.9;       ///< 模拟二进制交叉(SBX)概率
        double crossoverDistribution = 20.0; ///< SBX 分布指数
        double mutationProb = -1.0;
        double mutationDistribution = 20.0;  ///< PM 分布指数
        int randomSeed = 42;                   
    };

    /**
     * @brief NSGA-II 优化器
     * @param m_problem 多目标问题实例
     * @param m_config 优化器超参数配置
     * @param m_rng，它是OpenCV提供的随机数生成器，选择它而不是C++的std::mt19937，
     * 是因为OpenCV的随机数生成器已经过优化，在随机生成矩阵时，能够在并行计算中提供更好的性能。
     */
    class NSGAOptimizer{
    public:
        NSGAOptimizer(std::shared_ptr<IMultiObjectiveProblem> problem, const OptimizerConfig& config);
        ~NSGAOptimizer() = default;

        /**
         * @brief 运行优化器
         * @param optimalObjs 最优目标函数值矩阵，大小为1xM，M为目标函数维度
         * @param verbose 是否输出每一代的最优目标函数值
         * @return 返回最优个体的决策变量矩阵
         */
        cv::Mat optimize(cv::OutputArray out_optimalObjs, bool verbose = false);

        /*
         * @brief 迭代回调函数，用于在每一代结束时输出进度或可视化
         * @param gen 当前迭代次数
         * @param popObjs 目标函数矩阵，大小为NxM，N为种群数量，M为目标函数维度
         * @param popVars 种群变量矩阵，大小为NxD，N为种群数量，D为变量维度
        */
        using IterationCallback = std::function<void(int gen, const cv::Mat& popObjs, const cv::Mat& popVars)>;
        /**
         * @brief 设置迭代回调函数，用于在每一代结束时输出进度或可视化
         * @param callback 迭代回调函数，参数为当前迭代次数、种群变量矩阵、目标函数矩阵
         */
        void setIterationCallback(IterationCallback callback);
    private:
        /**
         * @brief 初始化种群变量矩阵。种群的初始解应该在整个决策空间中均匀分布。假设决策空间为 [lb, ub]，
         * 为了保证初始种群的多样性，会通过公式x_i = x_min + (x_max - x_min) * rand(N, D)来初始化个体。
         * @param populationVars 输出，种群变量矩阵，大小为NxD，即N个个体，每个个体有D个变量
         */
        void initializePopulation(cv::Mat& populationVars);

        /**
         * @brief 生成子代，这里主要涉及SBX交叉和PM变异
         * @param parentVars 父代变量矩阵，即N个个体，每个个体有D个变量
         * @param offspringVars 输出，子代变量矩阵，大小也为NxD，即N个个体，每个个体有D个变量
         */
        void generateOffspring(const cv::Mat& parentVars, cv::Mat& offspringVars);

        /**
         * @brief 快速非支配排序 (基于 ENS 预排序将时间复杂度优化为MNlogN)
         * @param combinedObjs 合并后的目标函数矩阵，大小为2N x M，N为种群数量，由于经过一次变异，因此此时种群数量为2N，
         * M为目标函数维度
         * @return std::vector<std::vector<int>> 包含个体索引的 2D vector，每一行代表一个 Front 层级
         */
        std::vector<std::vector<int>> fastNonDominatedSortENS(const cv::Mat& combinedObjs);

        /**
         * @brief 计算拥挤度距离
         * @param objs 对应某一 Front 的目标值矩阵，大小为FxM，F为 Front 层级的个体数量，M为目标函数维度
         * @return std::vector<double> 每个个体的拥挤度距离
         */
        std::vector<double> calculateCrowdingDistance(const cv::Mat& objs);        

        /**
         * @brief 环境选择 (从 2N 的混合种群中挑选 N 个优秀个体进入下一代)
         * @param combinedVars 合并后的变量矩阵，大小为2NxD，N为种群数量，D为变量维度
         * @param combinedObjs 合并后的目标函数矩阵，大小为2NxM，N为种群数量，M为目标函数维度
         * @param nextPopVars 下一代变量矩阵，大小为NxD，N为种群数量，D为变量维度
         * @param nextPopObjs 下一代目标函数矩阵，大小为NxM，N为种群数量，M为目标函数维度
         */
        void environmentalSelection(const cv::Mat& combinedVars, const cv::Mat& combinedObjs,
                                    cv::Mat& nextPopVars, cv::Mat& nextPopObjs);

    private:
        std::shared_ptr<IMultiObjectiveProblem> m_problem;
        OptimizerConfig m_config;
        IterationCallback m_callback;
        
        cv::RNG m_rng; // OpenCV 随机数生成器            
    };

}}
#endif // NSGA2_H