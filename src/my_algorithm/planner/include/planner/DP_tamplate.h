#ifndef DP_TAMPLATE_H
#define DP_TAMPLATE_H

#include <vector>
#include <memory>
#include <functional>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <rclcpp/logging.hpp>

namespace AD_algorithm {
namespace planner {

// ==================== 基础数据结构 ====================
/**
 * @brief 通用状态接口
 * @tparam StateType 状态类型
 */
template<typename StateType>
struct DpNode {
    StateType state;                              // 状态值
    double cumulative_cost;                       // 累积代价
    std::shared_ptr<DpNode<StateType>> parent;    // 父节点指针
    int layer_index;                              // 所在层索引
    int node_index;                               // 节点在层内的索引
    
    // 添加默认构造函数
    DpNode() : cumulative_cost(0.0), parent(nullptr), layer_index(0), node_index(0) {}
    
    DpNode(const StateType& s, double cost = 0.0, 
           std::shared_ptr<DpNode<StateType>> p = nullptr, 
           int layer = 0, int index = 0)
        : state(s), cumulative_cost(cost), parent(p), 
          layer_index(layer), node_index(index) {}
    
    // 比较运算符用于优先队列
    bool operator>(const DpNode& other) const {
        return cumulative_cost > other.cumulative_cost;
    }
};

// ==================== 策略接口 ====================

/**
 * @brief 代价函数策略接口
 * @tparam StateType 状态类型
 */
template<typename StateType>
class CostFunctionStrategy {
public:
    virtual ~CostFunctionStrategy() = default;
    
    /**
     * @brief 计算状态转移代价
     * @param from 起始状态
     * @param to 目标状态
     * @return 转移代价
     */
    virtual double calculateTransitionCost(
        StateType& from, 
        StateType& to
    ) = 0;
    
    /**
     * @brief 计算状态启发式代价
     * @param state 当前状态
     * @param goal 目标状态（可选）
     * @return 启发式代价
     */
    virtual double calculateHeuristicCost(
        const StateType& state,
        const StateType& goal = StateType{}
    ) const {
        return 0.0;  // 默认不启用启发式
    }
};

/**
 * @brief 约束检查策略接口
 * @tparam StateType 状态类型
 */
template<typename StateType>
class ConstraintCheckerStrategy {
public:
    virtual ~ConstraintCheckerStrategy() = default;
    
    /**
     * @brief 检查状态是否满足约束
     * @param state 待检查状态
     * @return 是否满足约束
     */
    virtual bool checkState(const StateType& state) const = 0;
    
    /**
     * @brief 检查状态转移是否满足约束
     * @param from 起始状态
     * @param to 目标状态
     * @return 是否满足转移约束
     */
    virtual bool checkTransition(
        const StateType& from, 
        const StateType& to
    ) const = 0;
};

/**
 * @brief 采样策略接口
 * @tparam StateType 状态类型
 */
template<typename StateType>
class SamplingStrategy {
public:
    virtual ~SamplingStrategy() = default;
    
    /**
     * @brief 生成下一层状态采样
     * @param current_layer 当前层状态
     * @param layer_index 目标层索引
     * @return 下一层状态集合
     */
    virtual std::vector<StateType> generateNextLayer(
        const std::vector<StateType>& current_layer,
        int layer_index
    ) const = 0;
    
    /**
     * @brief 生成初始状态采样
     * @param start_state 起始状态
     * @param num_layers 总层数
     * @return 状态采样网格
     */
    virtual std::vector<std::vector<StateType>> generateSamplingGrid(
        const StateType& start_state,
        int num_layers
    ) const {
        std::vector<std::vector<StateType>> grid(num_layers);
        std::vector<StateType> current_layer = {start_state};
        
        for (int i = 0; i < num_layers; ++i) {
            grid[i] = generateNextLayer(current_layer, i);
            current_layer = grid[i];
        }
        
        return grid;
    }
};

/**
 * @brief 回溯策略接口
 * @tparam StateType 状态类型
 */
template<typename StateType>
class BacktrackStrategy {
public:
    virtual ~BacktrackStrategy() = default;
    
    /**
     * @brief 从最优节点回溯路径
     * @param optimal_node 最优节点
     * @return 状态序列
     */
    virtual std::vector<StateType> backtrack(
        const DpNode<StateType>& optimal_node
    ) const = 0;
    
    /**
     * @brief 判断是否为终止状态
     * @param state 待判断状态
     * @param layer_index 所在层索引
     * @return 是否为终止状态
     */
    virtual bool isTerminalState(
        const StateType& state,
        int layer_index
    ) const = 0;
};

// ==================== DP规划器配置 ====================

/**
 * @brief DP规划器配置结构体
 */
struct DpPlannerConfig {
    // 搜索参数
    int max_layers = 10;                     // 最大规划层数
    bool enable_pruning = true;              // 是否启用剪枝
    int pruning_keep_num = 5;                // 每层保留的节点数
    bool enable_heuristic = false;           // 是否启用启发式
    bool enable_dynamic_programming = true;  // 是否启用动态规划
    
    // 性能参数
    double time_limit_ms = 100.0;            // 时间限制(ms)
    int max_nodes_per_layer = 50;            // 每层最大节点数
    
    // 输出参数
    bool debug_mode = false;                 // 调试模式
    bool store_full_tree = false;            // 是否存储完整搜索树
    
    // 验证函数
    void validate() const {
        if (max_layers <= 0) {
            throw std::invalid_argument("max_layers must be positive");
        }
        if(enable_pruning){
            if (pruning_keep_num <= 0) {
                throw std::invalid_argument("pruning_keep_num must be positive");
            }
        }
    }
};

// ==================== 主DP规划器类 ====================

/**
 * @brief 通用DP规划器
 * @tparam StateType 状态类型
 * 
 * 支持路径规划(SL平面)和速度规划(ST平面)的通用DP算法
 */
template<typename StateType>
class DpPlanner {
public:
    /**
     * @brief 规划结果结构体
     */
    struct PlanningResult {
        std::vector<StateType> optimal_path;      // 最优路径
        double total_cost;                         // 总代价
        bool success;                              // 是否成功
        std::string message;                       // 结果信息
        int num_nodes_expanded;                    // 扩展节点数
        double computation_time_ms;                // 计算时间(ms)
        std::vector<std::vector<std::shared_ptr<DpNode<StateType>>>> search_tree; // 搜索树(可选)
        
        PlanningResult() 
            : total_cost(0.0), success(false), 
              num_nodes_expanded(0), computation_time_ms(0.0) {}
    };
    
    /**
     * @brief 构造函数
     * @param cost_function 代价函数策略
     * @param constraint_checker 约束检查策略
     * @param sampling_strategy 采样策略
     * @param backtrack_strategy 回溯策略
     * @param config DP配置
     */
    DpPlanner(
        std::shared_ptr<CostFunctionStrategy<StateType>> cost_function,
        std::shared_ptr<ConstraintCheckerStrategy<StateType>> constraint_checker,
        std::shared_ptr<SamplingStrategy<StateType>> sampling_strategy,
        std::shared_ptr<BacktrackStrategy<StateType>> backtrack_strategy,
        const DpPlannerConfig& config = DpPlannerConfig{}
    );
    
    /**
     * @brief 执行DP规划
     * @param start_state 起始状态
     * @param num_layers 规划层数
     * @param goal_state 目标状态(可选)
     * @return 规划结果
     */
    PlanningResult plan(
        const StateType& start_state,
        int num_layers,
        const StateType& goal_state = StateType{}
    );
    
    /**
     * @brief 执行DP规划(使用预设采样网格)
     * @param sampling_grid 预设采样网格
     * @param start_state 起始状态
     * @param goal_state 目标状态(可选)
     * @return 规划结果
     */
    PlanningResult planWithGrid(
        const std::vector<std::vector<StateType>>& sampling_grid,
        const StateType& start_state,
        const StateType& goal_state = StateType{}
    );
    
    /**
     * @brief 获取规划器配置
     * @return 当前配置
     */
    const DpPlannerConfig& getConfig() const { return _config; }
    
    /**
     * @brief 更新规划器配置
     * @param config 新配置
     */
    void updateConfig(const DpPlannerConfig& config);
    
    /**
     * @brief 重置规划器状态
     */
    void reset();


    void printSearchTree(
        std::function<void(const StateType&)> state_printer = nullptr
        );
protected:
    // 这里的最优节点应该由回溯策略来实现，速度规划和路径规划是不同的
    

    // 内部方法
    virtual void forwardSearch(
        const std::vector<std::vector<StateType>>& sampling_grid,
        const StateType& start_state
    );

    virtual DpNode<StateType> findOptimalNode();
    // 配置
    DpPlannerConfig _config;
    
    // 内部状态
    std::vector<std::vector<std::shared_ptr<DpNode<StateType>>>> _search_tree;

    void expandLayer(
        std::vector<std::shared_ptr<DpNode<StateType>>>& current_layer,
        const std::vector<StateType>& next_states,
        int layer_index
    );
    
    bool _succes=false;

    void pruneLayer(std::vector<std::shared_ptr<DpNode<StateType>>>& layer, int keep_num);

private:
    // 策略组件
    std::shared_ptr<CostFunctionStrategy<StateType>> _cost_function;
    std::shared_ptr<ConstraintCheckerStrategy<StateType>> _constraint_checker;
    std::shared_ptr<SamplingStrategy<StateType>> _sampling_strategy;
    std::shared_ptr<BacktrackStrategy<StateType>> _backtrack_strategy;
    

    StateType _goal_state;
    bool _has_goal_state;
    
    std::vector<StateType> backtrackPath(
        const DpNode<StateType>& optimal_node
    ) const;
    
    void validateInputs() const;
};

// ==================== 默认策略实现 ====================

/**
 * @brief 默认回溯策略
 * @tparam StateType 状态类型
 */
template<typename StateType>
class DefaultBacktrackStrategy : public BacktrackStrategy<StateType> {
public:
    DefaultBacktrackStrategy() = default;
    
    std::vector<StateType> backtrack(
        const DpNode<StateType>& optimal_node
    ) const override {
        std::vector<StateType> path;
        const DpNode<StateType>* current = &optimal_node;
        
        while (current != nullptr) {
            path.insert(path.begin(), current->state);
            current = current->parent.get();
        }
        
        return path;
    }
    
    bool isTerminalState(
        const StateType& state,
        int layer_index
    ) const override {
        // 默认实现：最后一层即为终止状态
        (void)state;  // 避免未使用参数警告
        (void)layer_index;
        return false;
    }
};

}
} // namespace planning

#endif //