#include "optics/nsga2.h"
#include <iostream>
#include <algorithm>
#include <numeric>

NAMESPACE_BEGIN{ namespace optics{
    NSGAOptimizer::NSGAOptimizer(std::shared_ptr<IMultiObjectiveProblem> problem, const OptimizerConfig& config):
        m_problem(problem), m_config(config), m_rng(config.randomSeed)
    {
        if(m_problem == nullptr){
            throw std::runtime_error("NSGAOptimizer: problem is nullptr");
        }

        if(m_config.mutationProb < 0 || m_config.mutationProb > 1) {
            // 如果未指定变异概率，按照经验默认设置为 1 / D
            m_config.mutationProb = 1.0 / m_problem->getVariableCount();
        }
    }

    void NSGAOptimizer::setIterationCallback(IterationCallback callback){
        m_callback = callback;
    }

    cv::Mat NSGAOptimizer::optimize(cv::OutputArray out_optimalObjs, bool verbose){
        cv::Mat popVars, popObjs;  // 父代的变量，与父代的函数值，矩阵分别为 N x D, N x M
        cv::Mat offspringVars, offspringObjs;  // 子代的变量，与子代的函数值，矩阵分别为 N x D, N x M
        cv::Mat combinedVars, combinedObjs;  // 父代与子代合并后的变量，与合并后的函数值，矩阵分别为 2N x D, 2N x M

        if(verbose) std::cout << "NSGAOptimizer: 初始化种群" << std::endl;
        initializePopulation(popVars);
        m_problem->evaluate(popVars, popObjs);
        if(verbose) std::cout << "NSGAOptimizer: 初始化种群完成" << std::endl;

        for(int gen = 0; gen < m_config.maxGenerations; ++gen){
            generateOffspring(popVars, offspringVars);
            m_problem->evaluate(offspringVars, offspringObjs);
            cv::vconcat(popVars, offspringVars, combinedVars);
            cv::vconcat(popObjs, offspringObjs, combinedObjs);

            environmentalSelection(combinedVars, combinedObjs, popVars, popObjs);
            if(m_callback){
                m_callback(gen, popObjs, popVars);
                continue;
            }
            if(verbose) std::cout << "\rNSGAOptimizer: 进化进度" << gen + 1 << "/" << m_config.maxGenerations << "代" << std::flush;
        }

        if(verbose) std::cout << "\nNSGAOptimizer: 进化完成，正在提取帕累托前沿" << std::endl;
        
        //对最后一代种群进行最后一次排序
        std::vector<std::vector<int>> finalFronts = fastNonDominatedSortENS(popObjs);
        const std::vector<int>& bestFront = finalFronts[0];
        int paretoSize = bestFront.size();

        cv::Mat optimalVars(paretoSize, popVars.cols, CV_64FC1);
        cv::Mat optimalObjs(paretoSize, popObjs.cols, CV_64FC1);
        for(int i = 0; i < paretoSize; ++i){
            int idx = bestFront[i];
            popVars.row(idx).copyTo(optimalVars.row(i));
            popObjs.row(idx).copyTo(optimalObjs.row(i));
        }

        if(out_optimalObjs.needed()) optimalObjs.copyTo(out_optimalObjs);

        // 返回最优个体的决策变量矩阵
        return optimalVars;
    }

    void NSGAOptimizer::initializePopulation(cv::Mat & populationVars){
        int N = m_config.populationSize;
        int D = m_problem->getVariableCount();
        
        cv::Mat lowerBounds, upperBounds;
        m_problem->getBounds(lowerBounds, upperBounds);

        //确保边界矩阵的类型为CV_64FC1，即double类型，且尺寸符合要求（1 x D）
        CV_Assert(lowerBounds.type() == CV_64FC1 && upperBounds.type() == CV_64FC1);
        CV_Assert(lowerBounds.cols == D && upperBounds.cols == D);

        // 生成随机数矩阵randMat，尺寸为N x D，元素范围为[0, 1)
        cv::Mat randMat(N, D, CV_64FC1);
        m_rng.fill(randMat, cv::RNG::UNIFORM, 0.0, 1.0);

        cv::Mat lowerMat = cv::repeat(lowerBounds, N, 1);  // 重复N次，得到N x D的矩阵
        cv::Mat upperMat = cv::repeat(upperBounds, N, 1);

        //计算公式：X = X_min + randMat * (X_max - X_min)
        //这里的mul是逐元素乘法
        populationVars = lowerMat + randMat.mul(upperMat - lowerMat);
    }

    void NSGAOptimizer::generateOffspring(const cv::Mat& parentVars, cv::Mat& offspringVars){
        int N = parentVars.rows;
        int D = parentVars.cols;

        offspringVars.create(N, D, CV_64FC1);

        cv::Mat lowerMat, upperMat;
        m_problem->getBounds(lowerMat, upperMat);
        const double * lower = lowerMat.ptr<double>(0);
        const double * upper = upperMat.ptr<double>(0);

        double pc = m_config.crossoverProb;              // 交叉概率
        double pm = m_config.mutationProb;               // 变异概率
        double nc = m_config.crossoverDistribution;  // 交叉分布指数
        double nm = m_config.mutationDistribution;  // 变异分布指数

        std::vector<int> indices(N);
        std::iota(indices.begin(), indices.end(), 0);   // 填充indices为0, 1, 2, ..., N-1
        cv::randShuffle(indices, 1.0, &m_rng);  // 随机打乱indices

        //每次取出两个父代，生成两个子代
        for(int i = 0;i < N / 2; ++i){
            // 提取出两个父代
            int p1_idx = indices[2 * i];
            int p2_idx = indices[2 * i + 1];

            const double * p1 = parentVars.ptr<double>(p1_idx);
            const double * p2 = parentVars.ptr<double>(p2_idx);
            double * c1 = offspringVars.ptr<double>(2 * i);
            double * c2 = offspringVars.ptr<double>(2 * i + 1);

            for(int j = 0; j < D; ++j){
                double val_p1 = p1[j];
                double val_p2 = p2[j];
                double val_c1 = val_p1;  // 默认子代等于父代，可以确保不变异时不发生改变
                double val_c2 = val_p2;

                //判断是否要进行交叉
                if(m_rng.uniform(0.0, 1.0) <= pc && std::abs(val_p1 - val_p2) > 1e-14){
                    double u = m_rng.uniform(0.0, 1.0);
                    double beta = 0.0;
                    if(u <= 0.5){
                        beta = std::pow(2.0 * u, 1.0 / (nc + 1.0));
                    } else beta = std::pow(1.0 / (2.0 * (1.0 - u)), 1.0 / (nc + 1.0));

                    val_c1 = 0.5 * ((1.0 + beta) * val_p1 + (1.0 - beta) * val_p2);
                    val_c2 = 0.5 * ((1.0 - beta) * val_p1 + (1.0 + beta) * val_p2);
                }

                auto applyMutation = [&](double & val) -> void {
                    if(m_rng.uniform(0.0, 1.0) > pm) return;
                    double u = m_rng.uniform(0.0, 1.0);
                    double delta = 0.0;
                    if(u <= 0.5){
                        delta = std::pow(2.0 * u, 1.0 / (nm + 1.0)) - 1.0;
                    } else delta = 1.0 - std::pow(2.0 * (1.0 - u), 1.0 / (nm + 1.0));
                    val += delta * (upper[j] - lower[j]);
                };

                applyMutation(val_c1);
                applyMutation(val_c2);

                // 确保子代在边界内
                val_c1 = std::max(lower[j], std::min(upper[j], val_c1));
                val_c2 = std::max(lower[j], std::min(upper[j], val_c2));

                c1[j] = val_c1;
                c2[j] = val_c2;
            }
        }
    }

    std::vector<std::vector<int>> NSGAOptimizer::fastNonDominatedSortENS(const cv::Mat& combinedObjs){
        int N2 = combinedObjs.rows;
        int M = combinedObjs.cols;  // 目标函数的维度

        std::vector<int> indices(N2);
        std::iota(indices.begin(), indices.end(), 0);   // 填充indices为0, 1, 2, ..., N2-1

        // 基于字典序预排序，字典序小的个体优先
        std::sort(indices.begin(), indices.end(), [&combinedObjs, M](int a, int b){
            const double* objA = combinedObjs.ptr<double>(a);
            const double* objB = combinedObjs.ptr<double>(b);
            for (int i = 0; i < M; ++i) {
                if (std::abs(objA[i] - objB[i]) > 1e-10) {
                    return objA[i] < objB[i];
                }
            }
            return false;          
        });

        auto dominates = [&combinedObjs, M](int a, int b) {
            const double* objA = combinedObjs.ptr<double>(a);
            const double* objB = combinedObjs.ptr<double>(b);
            bool strictlyBetter = false;
            for (int i = 0; i < M; ++i) {
                if (objA[i] > objB[i] + 1e-10) {
                    return false; // a 在某一个目标上比 b 差，a 绝不可能支配 b
                }
                if (objA[i] < objB[i] - 1e-10) {
                    strictlyBetter = true; // a 在某一个目标上严格优于 b
                }
            }
            return strictlyBetter;  //这里严格化，因为有可能a和b相等，此时不能算作a支配b
        };

        std::vector<std::vector<int>> fronts;
        
        for (int idx : indices) {
            bool assigned = false;
            
            for (auto& front : fronts) {
                bool isDominatedByFront = false;
                
                for (int member : front) {
                    if (dominates(member, idx)) {
                        isDominatedByFront = true;
                        break; // 一旦被支配，立刻判定无法加入当前 Front，跳出内层循环
                    }
                }
                
                if (!isDominatedByFront) {
                    front.push_back(idx);
                    assigned = true;
                    break; // 成功放入，处理下一个个体
                }
            }
            
            // 如果当前个体被所有已存在的 Front 支配，则为它开辟一个新的 Front 层级
            if (!assigned) {
                fronts.push_back({idx});
            }
        }

        return fronts;        
    }

    std::vector<double> NSGAOptimizer::calculateCrowdingDistance(const cv::Mat& objs) {
        int F = objs.rows; //当前Front中的个体数量
        int M = objs.cols;

        std::vector<double> distances(F, 0.0);
        if(F == 0) return distances;
        if(F <= 2){
            std::fill(distances.begin(), distances.end(), std::numeric_limits<double>::infinity());
            return distances; //边界的拥挤度默认无穷大
        }

        //对每个目标维度独立计算距离并累加
        for(int m = 0; m < M; ++m){
            std::vector<int> indices(F);
            std::iota(indices.begin(), indices.end(), 0);

            std::sort(indices.begin(), indices.end(), [&objs, m](int a, int b){
                return objs.at<double>(a, m) < objs.at<double>(b, m);
            });

            //计算第一个和最后一个个体的距离为无穷大
            distances[indices.front()] = std::numeric_limits<double>::infinity();
            distances[indices.back()] = std::numeric_limits<double>::infinity();

            double objMin = objs.at<double>(indices.front(), m);
            double objMax = objs.at<double>(indices.back(), m);
            double range = objMax - objMin;
            if(range < 1e-10) continue; //大家都很接近，拥挤度这一维全为0，不再考虑

            for (int i = 1; i < F - 1; ++i) {
                int currentIdx = indices[i];
                
                if (distances[currentIdx] == std::numeric_limits<double>::infinity()) continue;
                double nextObj = objs.at<double>(indices[i + 1], m);
                double prevObj = objs.at<double>(indices[i - 1], m);                    
                distances[currentIdx] += (nextObj - prevObj) / range;
            }            
        }

        return distances;
    }

    void NSGAOptimizer::environmentalSelection(const cv::Mat& combinedVars, const cv::Mat& combinedObjs,
                            cv::Mat& nextPopVars, cv::Mat& nextPopObjs)
    {
        int N = m_config.populationSize;  // 种群大小
        int D = combinedVars.cols;  // 变量维度
        int M = combinedObjs.cols;  // 目标维度

        nextPopVars.create(N, D, CV_64FC1);
        nextPopObjs.create(N, M, CV_64FC1);

        std::vector<std::vector<int>> fronts = fastNonDominatedSortENS(combinedObjs);
        size_t count = 0;

        for(const auto & front : fronts){
            if(count + front.size() <= static_cast<size_t>(N)){
                for(int idx : front){
                    combinedVars.row(idx).copyTo(nextPopVars.row(count));
                    combinedObjs.row(idx).copyTo(nextPopObjs.row(count));
                    count++;
                }
                if(count == static_cast<size_t>(N)) break;
                else continue;
            }

            //剩余的做拥挤度排序，然后取前N-count个
            int remaining = N - static_cast<int>(count);
            cv::Mat frontObjs(front.size(), M, CV_64FC1);
            for(int i = 0; i < front.size(); ++i){
                combinedObjs.row(front[i]).copyTo(frontObjs.row(i));
            }
            std::vector<double> crowdingDistances = calculateCrowdingDistance(frontObjs);
            std::vector<int> sortedIndices(front.size());
            std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
            std::sort(sortedIndices.begin(), sortedIndices.end(), [&crowdingDistances](int a, int b){
                return crowdingDistances[a] > crowdingDistances[b];
            });

            for(int i = 0; i < remaining; ++i){
                int idx = front[sortedIndices[i]];
                combinedVars.row(idx).copyTo(nextPopVars.row(count));
                combinedObjs.row(idx).copyTo(nextPopObjs.row(count));
                count++;
            }
            break; 
        }

        CV_Assert(count == N);
    }

}}  //namespace optics, NAMESPACE_END