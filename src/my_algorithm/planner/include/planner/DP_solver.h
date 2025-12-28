#ifndef DP_SOLVER_HPP
#define DP_SOLVER_HPP

#include "planner/planner_weights.h"  // 添加包含
#include "planner/DP_tamplate.h"      // 移除 planner/emplanner.h
#include "planner/DP_algorithm.hpp"   // 添加模板实现
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "general_modules/math_tool.h"  // 添加 normalize_angle 声明
#include <vector>
#include <unordered_map>
#include <memory>

namespace AD_algorithm {
namespace planner {

// 路径规划状态（SL平面）
struct SLState {
    double s = 0.0;        // 纵向位置
    double l = 0.0;        // 横向位置
    double l_prime = 0.0;  // 一阶导数
    double l_prime_prime = 0.0; // 二阶导数
    
    // 默认构造函数
    SLState() = default;
    
    // 带参数的构造函数
    SLState(double s_val, double l_val, double lp = 0.0, double lpp = 0.0)
        : s(s_val), l(l_val), l_prime(lp), l_prime_prime(lpp) {}
    
    // 用于状态比较
    bool operator==(const SLState& other) const {
        return std::abs(s - other.s) < 1e-6 &&
               std::abs(l - other.l) < 1e-6;
    }
};

// SL障碍物表示
struct SLObstacle {
    double s_center;    // 障碍物中心s坐标
    double l_center;    // 障碍物中心l坐标
    double length;      // 障碍物长度（s方向）
    double width;       // 障碍物宽度（l方向）
    double safety_margin; // 安全边界
    
    SLObstacle(double s, double l, double len = 5.0, double wid = 2.0, double margin = 0.5)
        : s_center(s), l_center(l), length(len), width(wid), safety_margin(margin) {}
};

// 辅助函数：将FrenetPoint转换为SLObstacle
std::vector<SLObstacle> convertToSLObstacles(
    const std::vector<AD_algorithm::general::FrenetPoint>& frenet_obstacles,
    double length = 5.0,
    double width = 2.0,
    double safety_margin = 0.5);

// 路径规划策略类声明
class PathCostFunction : public planner::CostFunctionStrategy<SLState> {
private:
    std::vector<SLObstacle> _static_obstacles;
    WeightCoefficients _weights;
    PathPlannerConfig _config;
public:
    PathCostFunction(const WeightCoefficients& weights,
                        const PathPlannerConfig& config,
                    const std::vector<SLObstacle>& obstacles);
    
    double calculateTransitionCost(
        SLState& from, 
        SLState& to
    )  override;
    
    double calculateHeuristicCost(
        const SLState& state,
        const SLState& goal = SLState{}
    ) const override;
    
private:
    double calculateObstacleCost(const std::vector<double> s_set, const std::vector<double> l_set) const ;

    double calculateDistanceToRectangle(const double& s, const double& l, 
                                                   const SLObstacle& obs) const ;
};

class PathConstraintChecker : public planner::ConstraintCheckerStrategy<SLState> {
private:
    std::vector<SLObstacle> _static_obstacles;
    double _max_curvature = 0.1;  // 最大曲率
    double _max_l_dot = 3.0;  // 最大直线斜率dl/ds
    
public:
    PathConstraintChecker(double road_up, double road_low,
                         const std::vector<SLObstacle>& obstacles);
    
    bool checkState(const SLState& state) const override;
    bool checkTransition(const SLState& from, const SLState& to) const override;
    
private:
    bool isCollidingWithObstacle(const SLState& state, 
                                const SLObstacle& obs) const;
    bool checkInterpolatedPath(const SLState& from, const SLState& to) const;
};

class PathSamplingStrategy : public planner::SamplingStrategy<SLState> {
private:
    double _s_step;      // s方向步长
    int _num_layers;     
    int _l_samples;      // l方向采样点数
    std::vector<double> _road_width_left_vec;// l方向采样范围
    std::vector<double> _road_width_right_vec;
    double _resolution_l = 1.0; // l方向采样分辨率
public:
    PathSamplingStrategy(const PathPlannerConfig& config,const std::vector<double>& road_width_left_vec,
                         const std::vector<double>& road_width_right_vec,double resolution_l=1.0);
    
    std::vector<SLState> generateNextLayer(
        const std::vector<SLState>& current_layer,
        int layer_index
    ) const override;
    
    // 确保起点的s是0
    std::vector<std::vector<SLState>> generateSamplingGrid(
        const SLState& start_state,
        int num_layers
    ) const override;
};

// 速度规划相关结构体
struct STState {
    double t = 0.0;            // 时间
    double s = 0.0;            // 纵向位置
    double s_dot = 0.0;        // 速度
    double s_dot_dot = 0.0;    // 加速度
    
    STState() = default;
    
    STState(double t_val, double s_val, double sd = 0.0, double sdd = 0.0)
        : t(t_val), s(s_val), s_dot(sd), s_dot_dot(sdd) {}
    
    bool operator==(const STState& other) const {
        return std::abs(t - other.t) < 1e-6 &&
               std::abs(s - other.s) < 1e-6;
    }

};

// ST障碍物表示
struct STObstacle {
    double t_in;     // 进入时间
    double t_out;    // 离开时间
    double s_in;     // 进入位置
    double s_out;    // 离开位置
    
    STObstacle(double ti, double to, double si, double so)
        : t_in(ti), t_out(to), s_in(si), s_out(so) {}
    
    // 检查(t,s)点是否在障碍物区域内
    bool contains(double t, double s, double safety_margin = 2.0) const;
    
    // 计算点到障碍物线段的最小距离
    double minDistanceTo(double t, double s) const;
};


// 辅助函数：将ST图节点转换为STObstacle
std::vector<STObstacle> convertToSTObstacles(
    const std::vector<std::unordered_map<std::string, double>>& st_graph);

// 速度规划策略类声明
class SpeedCostFunction : public planner::CostFunctionStrategy<STState> {
private:
    WeightCoefficients _weights;
    double _reference_speed;
    std::vector<STObstacle> _st_obstacles;
    int _dp_poly_pnt_num; 
public:
    SpeedCostFunction(const WeightCoefficients& weights,
                    const SpeedPlannerConfig& config,
                     double reference_speed,
                     const std::vector<STObstacle>& obstacles);
    
    double calculateTransitionCost(
        STState& from, 
        STState& to
    ) override;
    
private:
    double calculateObstacleCost(
    const std::vector<std::pair<double, double>>& sample_points
    ) const;
};

class SpeedConstraintChecker : public planner::ConstraintCheckerStrategy<STState> {
private:
    std::vector<STObstacle> _st_obstacles;
    double _max_acceleration = 4.0;
    double _max_deceleration = -6.0;
    double _max_jerk = 10.0;
    double _max_s;
    
public:
    SpeedConstraintChecker(const SpeedPlannerConfig& config,
                          const std::vector<STObstacle>& obstacles = {});
    
    bool checkState(const STState& state) const override;
    bool checkTransition(const STState& from, const STState& to) const override;
    
private:
    bool checkInterpolatedPath(const STState& from, const STState& to) const;
};

class SpeedSamplingStrategy : public planner::SamplingStrategy<STState> {
private:
    double _t_step = 0.5;
    double _s_step = 0.5; // 初始s采样步长
    double _max_t = 8.0;
    double _max_s=100.0;
    double _increase_ratio = 0.1;  // 采样步长递增比例

    std::vector<double> _s_serier,_t_series;
public:
    SpeedSamplingStrategy(const SpeedPlannerConfig& config);
    std::vector<STState> generateNextLayer(
        const std::vector<STState>& current_layer,
        int layer_index
    ) const override;
    
    std::vector<std::vector<STState>> generateSamplingGrid(
        const STState& start_state,
        int num_layers
    ) const override;
private:
    std::vector<double> _sample_s_t_method(double max_border,double init_step) const ;
};

// 速度规划的DP有点不一样，需要重写进行实现，不同点是s,t只要有一个先到达终点，就找到了最优路径，后面的层就不用继续规划了

class SpeedDpPlanner : public DpPlanner<STState> {
public:
    /**
     * @brief 构造函数
     * @param cost_function 代价函数策略
     * @param constraint_checker 约束检查策略
     * @param sampling_strategy 采样策略
     * @param backtrack_strategy 回溯策略
     * @param config DP规划器配置
     */
    SpeedDpPlanner(
        std::shared_ptr<CostFunctionStrategy<STState>> cost_function,
        std::shared_ptr<ConstraintCheckerStrategy<STState>> constraint_checker,
        std::shared_ptr<SamplingStrategy<STState>> sampling_strategy,
        std::shared_ptr<BacktrackStrategy<STState>> backtrack_strategy,
        const DpPlannerConfig& config = DpPlannerConfig{}
    );
    
protected:
    /**
     * @brief 重写的前向搜索算法
     * @param sampling_grid 采样网格
     * @param start_state 起点状态
     * @return 规划结果
     */
    void forwardSearch(
        const std::vector<std::vector<STState>>& sampling_grid,
        const STState& start_state
    ) override;
    
    /**
     * @brief 重写的最优节点查找算法
     * @param last_layer 最后一层节点
     * @return 最优节点
     */
    DpNode<STState> findOptimalNode()override;

    double _grid_max_s=0;
    // std::shared_ptr<DpNode<STState>> _optimal_node=nullptr;
};


// 使用示例函数
void demo();

} // namespace planner
} // namespace AD_algorithm

#endif // DP_DEMO_HPP