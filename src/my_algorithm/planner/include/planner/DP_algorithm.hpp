#ifndef DP_ALGORITHM_HPP
#define DP_ALGORITHM_HPP

#include "DP_tamplate.h"
#include <chrono>
#include <algorithm>
#include <queue>
#include <iostream>
#include <iomanip>
namespace AD_algorithm {
namespace planner {

// ==================== DpPlanner 实现 ====================

template<typename StateType>
DpPlanner<StateType>::DpPlanner(
    std::shared_ptr<CostFunctionStrategy<StateType>> cost_function,
    std::shared_ptr<ConstraintCheckerStrategy<StateType>> constraint_checker,
    std::shared_ptr<SamplingStrategy<StateType>> sampling_strategy,
    std::shared_ptr<BacktrackStrategy<StateType>> backtrack_strategy,
    const DpPlannerConfig& config
) : _cost_function(cost_function),
    _constraint_checker(constraint_checker),
    _sampling_strategy(sampling_strategy),
    _backtrack_strategy(backtrack_strategy),
    _config(config),
    _has_goal_state(false) {
    
    _config.validate();
}

template<typename StateType>
void DpPlanner<StateType>::validateInputs() const {
    if (!_cost_function) {
        throw std::runtime_error("Cost function strategy is not set");
    }
    if (!_constraint_checker) {
        throw std::runtime_error("Constraint checker strategy is not set");
    }
    if (!_sampling_strategy) {
        throw std::runtime_error("Sampling strategy is not set");
    }
    if (!_backtrack_strategy) {
        throw std::runtime_error("Backtrack strategy is not set");
    }
}

template<typename StateType>
typename DpPlanner<StateType>::PlanningResult 
DpPlanner<StateType>::plan(
    const StateType& start_state,
    int num_layers,
    const StateType& goal_state
) {
    validateInputs();
    
    auto start_time = std::chrono::high_resolution_clock::now();
    PlanningResult result;
    
    try {
        // 初始化搜索树
        _search_tree.clear();
        std::vector<std::shared_ptr<DpNode<StateType>>> start_layer;
        start_layer.emplace_back(std::make_shared<DpNode<StateType>>(start_state, 0.0, nullptr, 0, 0));
        _search_tree.push_back(start_layer);
        
        // 设置目标状态
        _goal_state = goal_state;
        _has_goal_state = true;
        
        auto sampling_grid=_sampling_strategy->generateSamplingGrid(start_state,_config.max_layers);
        // 执行前向搜索
        forwardSearch(sampling_grid, start_state);


        for (size_t i = 0; i < _search_tree.size(); i++)
        {
            result.num_nodes_expanded += _search_tree[i].size();       
        }
        DpNode<StateType> optimal_node;
        // 查找最优节点
        if (!_search_tree.empty()) {optimal_node = findOptimalNode();}

        // 回溯路径
        if(_succes){
            result.optimal_path = backtrackPath(optimal_node);
            result.total_cost = optimal_node.cumulative_cost;
            result.success = true;
            result.message = "Planning succeeded";
        }
        else {
            result.success = false;
            result.message = "No feasible path found";
        }

        // 记录搜索树(如果启用)
        if (_config.store_full_tree) {
            result.search_tree = _search_tree;
        }
        
    } catch (const std::exception& e) {
        result.success = false;
        result.message = std::string("Planning failed: ") + e.what();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    return result;
}

template<typename StateType>
typename DpPlanner<StateType>::PlanningResult 
DpPlanner<StateType>::planWithGrid(
    const std::vector<std::vector<StateType>>& sampling_grid,
    const StateType& start_state,
    const StateType& goal_state
) {
    validateInputs();
    auto start_time = std::chrono::high_resolution_clock::now();
    PlanningResult result;
    // sampling_grid第一层是不是起点都行，前向搜索会处理掉
    try {
        // 设置目标状态
        _goal_state = goal_state;
        _has_goal_state = true;
        // 执行前向搜索
        forwardSearch(sampling_grid, start_state);

        for (size_t i = 0; i < _search_tree.size(); i++)
        {
            result.num_nodes_expanded += _search_tree[i].size();       
        }

        // 查找最优节点
        DpNode<StateType> optimal_node;
        // 查找最优节点
        if (!_search_tree.empty()) {optimal_node = findOptimalNode();}
        // 回溯路径
        if(_succes){
            result.optimal_path = backtrackPath(optimal_node);
            result.total_cost = optimal_node.cumulative_cost;
            result.success = true;
            result.message = "Planning succeeded";
        }
        else {
            result.success = false;
            result.message = "No feasible path found";
        }

        // 记录搜索树(如果启用)
        if (_config.store_full_tree) {
            result.search_tree = _search_tree;
        }
        
    } catch (const std::exception& e) {
        result.success = false;
        result.message = std::string("Planning failed: ") + e.what();
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    return result;
}

template<typename StateType>
void DpPlanner<StateType>::forwardSearch(
    const std::vector<std::vector<StateType>>& sampling_grid,
    const StateType& start_state
) {
    // 初始化搜索树
    _search_tree.clear();
    // 创建起始节点
    std::vector<std::shared_ptr<DpNode<StateType>>> start_layer;
    start_layer.emplace_back(std::make_shared<DpNode<StateType>>(start_state, 0.0, nullptr, 0, 0));
    _search_tree.push_back(start_layer);
    
    // 逐层扩展
    for (size_t layer_idx = 0; layer_idx < sampling_grid.size(); ++layer_idx) {
        if (layer_idx >= static_cast<size_t>(_config.max_layers)) {
            break;
        }
        
        // 扩展下一层
        expandLayer(
            _search_tree.back(),
            sampling_grid[layer_idx],
            static_cast<int>(layer_idx + 1)
        );
        
        // 剪枝
        if (_config.enable_pruning && 
            _search_tree.back().size() > static_cast<size_t>(_config.pruning_keep_num)) {
            pruneLayer(_search_tree.back(), _config.pruning_keep_num);
        }
        // 检查时间限制
        // (在实际实现中需要添加时间检查)
    }
    
}

template<typename StateType>
void DpPlanner<StateType>::expandLayer(
    std::vector<std::shared_ptr<DpNode<StateType>>>& current_layer,
    const std::vector<StateType>& next_states,
    int layer_index
) {
    std::vector<std::shared_ptr<DpNode<StateType>>> next_layer;
    // 遍历当前层所有节点，填满下一层的DP_array,即搜索树
    double min_cumulative_cost;
    int count=0;
    std::shared_ptr<DpNode<StateType>> best_parent;
    // 首先遍历下一层所有可能状态
    // std::cout<<"下一层状态数量："<<next_states.size()<<std::endl;
    for(size_t j = 0; j < next_states.size(); ++j) {
        // 复制状态，防止修改原状态,原来的状态网格没有变化
        // 修改的是节点网格中的节点对应的状态
        // std::cout<<"状态："<<j<<std::endl;

        auto next_state = next_states[j];
        min_cumulative_cost = std::numeric_limits<double>::infinity();
        best_parent = nullptr;

        // 检查状态约束
        if (!_constraint_checker->checkState(next_state)) {
            // std::cout<<"状态："<<j<<"检测不通过"<<std::endl;
            continue;
        }

        StateType best_next_state;//在速度规划时会使用from节点计算to的信息，修改to的属性
        // 对当前层每个节点尝试转移到next_state
        for (size_t i = 0; i < current_layer.size(); ++i) {
            const auto& current_node = current_layer[i];
            
            // 检查转移约束
            if (!_constraint_checker->checkTransition(
                current_node->state, next_state)) {
                // std::cout<<"第"<<layer_index-1<<"层第"<<i<<"个节点向下一层第"<<j<<"个状态不满足转移约束，"<<"转移失败"<<std::endl;
                continue;
            }
            
            // 计算转移代价
            double transition_cost = 0;
            transition_cost =_cost_function->calculateTransitionCost(
                current_node->state, next_state);
            
            // 计算启发式代价(如果启用)
            double heuristic_cost = 0.0;
            if (_config.enable_heuristic && _has_goal_state) {
                heuristic_cost = _cost_function->calculateHeuristicCost(
                    next_state, _goal_state);
            }
            
            // 计算累积代价
            double cumulative_cost =0;
            cumulative_cost = current_node->cumulative_cost +transition_cost + heuristic_cost;

            // std::cout<<"节点的累计代价："<<cumulative_cost<<std::endl;

            if (cumulative_cost < min_cumulative_cost) {
                min_cumulative_cost = cumulative_cost;
                best_parent = current_node;
                // 支持在状态转移计算过程中修改next_state的属性以供下一层计算使用
                best_next_state = next_state;
            }
        }

        if(min_cumulative_cost==std::numeric_limits<double>::infinity()){
            // 代价无穷大，不可行，就不新建这个节点了
            continue;
        }

        count++;
        // 创建新节点，选择能转移到当前状态并且总代价最低的上一层的节点作为父节点
        next_layer.emplace_back(std::make_shared<DpNode<StateType>>(
            best_next_state,
            min_cumulative_cost,
            best_parent,  // 共享指针指向最好的父节点
            layer_index,
            static_cast<int>(j)
        ));
    }
    // std::cout<<"当前层扩展了"<<count<<"个节点\n";
    // 添加到搜索树
    _search_tree.push_back(next_layer);
}

template<typename StateType>
void DpPlanner<StateType>::pruneLayer(
    std::vector<std::shared_ptr<DpNode<StateType>>>& layer, 
    int keep_num
) {
    // 按代价排序
    std::sort(layer.begin(), layer.end(),
        [](const std::shared_ptr<DpNode<StateType>>& a, const std::shared_ptr<DpNode<StateType>>& b) {
            return a->cumulative_cost < b->cumulative_cost;
        });
    
    // 保留前keep_num个节点
    if (layer.size() > static_cast<size_t>(keep_num)) {
        layer.resize(keep_num);
    }
}

template<typename StateType>
DpNode<StateType> DpPlanner<StateType>::findOptimalNode() {
    std::shared_ptr<DpNode<StateType>> optimal_node;
    double min_cost=std::numeric_limits<double>::infinity();
    if(!_search_tree.back().empty()){
        auto last_layer=_search_tree.back();
        if (last_layer.empty()) {
            throw std::runtime_error("Last layer is empty");
        }

        for(size_t j=0;j<last_layer.size();j++){
            // std::cout<<"当前节点的s:"<<last_layer[j]->state.s<<std::endl;
            if((last_layer[j]->cumulative_cost)<min_cost){
                min_cost=last_layer[j]->cumulative_cost;
                optimal_node=last_layer[j];
                // std::cout<<"找到一个在T边界上的最优节点，代价为："<<min_cost<<           
                // "节点信息(layer,s,t)："<<optimal_node->layer_index<<","<<
                // optimal_node->state.s<<","<<optimal_node->state.t
                // <<std::endl;
            }
        }
    }

    if(min_cost!=std::numeric_limits<double>::infinity()){
        _succes=true;
        // std::cout<<"总代价："<<optimal_node->cumulative_cost<<std::endl;
        return *optimal_node;

    }
    else{
        return DpNode<StateType>();
    }
}

template<typename StateType>
std::vector<StateType> DpPlanner<StateType>::backtrackPath(
    const DpNode<StateType>& optimal_node
) const {
    return _backtrack_strategy->backtrack(optimal_node);
}

template<typename StateType>
void DpPlanner<StateType>::updateConfig(const DpPlannerConfig& config) {
    config.validate();
    _config = config;
}

template<typename StateType>
void DpPlanner<StateType>::reset() {
    _search_tree.clear();
    _has_goal_state = false;
}
template<typename StateType>
void DpPlanner<StateType>::printSearchTree(
        std::function<void(const StateType&)> state_printer
    ){
        if (_search_tree.empty()) {
            std::cout << "搜索树为空" << std::endl;
            return;
        }
        std::cout << "\n=== DP搜索树信息 ===" << std::endl;
        std::cout << "总层数: " << _search_tree.size() << std::endl;
        std::cout << "目标状态设置: " << (_has_goal_state ? "是" : "否") << std::endl;
        
        for (size_t layer_idx = 0; layer_idx < _search_tree.size(); ++layer_idx) {
            const auto& layer = _search_tree[layer_idx];
            
            std::cout << "\n--- 第 " << layer_idx << " 层 (节点数: " << layer.size() << ") ---" << std::endl;
            std::cout << "索引 | 累积代价 | 父节点 | 状态" << std::endl;
            std::cout << "--------------------------------------" << std::endl;
            
            for (size_t node_idx = 0; node_idx < layer.size(); ++node_idx) {
                const auto& node_ptr = layer[node_idx];
                
                std::cout << "[" << node_ptr->layer_index << "," 
                          << node_ptr->node_index << "] | ";
                
                std::cout << std::fixed << std::setprecision(3) 
                          << node_ptr->cumulative_cost << " | ";
                
                if (node_ptr->parent) {
                    std::cout << "[" << node_ptr->parent->layer_index 
                              << "," << node_ptr->parent->node_index << "] | ";
                } else {
                    std::cout << "  None  | ";
                }
                
                // 打印状态
                if (state_printer) {
                    state_printer(node_ptr->state);
                } else {
                    std::cout << "[状态详情]";
                }
                
                std::cout << std::endl;
            }
        }
        
        // 打印最优路径信息
        if (!_search_tree.empty()) {
            auto optimal_node = findOptimalNode();
            std::cout << "\n=== 最优节点信息 ===" << std::endl;
            std::cout << "层: " << optimal_node.layer_index 
                      << ", 索引: " << optimal_node.node_index 
                      << ", 累积代价: " << optimal_node.cumulative_cost << std::endl;
            
            // 回溯路径
            auto path = backtrackPath(optimal_node);
            std::cout << "路径长度: " << path.size() << std::endl;
        }
    }
}
}
#endif //