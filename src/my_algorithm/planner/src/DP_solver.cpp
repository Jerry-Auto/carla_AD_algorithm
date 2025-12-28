#include "planner/DP_solver.h"  
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <Eigen/Dense>
#include <iomanip>  
#include "general_modules/polynomial_curve.h"  

using namespace AD_algorithm::general;


namespace AD_algorithm {
namespace planner {

// ==================== 辅助函数实现 ====================

std::vector<SLObstacle> convertToSLObstacles(
    const std::vector<FrenetPoint>& frenet_obstacles,
    double length,
    double width,
    double safety_margin) {
    
    std::vector<SLObstacle> obstacles;
    for (const auto& fp : frenet_obstacles) {
        obstacles.emplace_back(fp.s, fp.l, length, width, safety_margin);
    }
    return obstacles;
}

std::vector<STObstacle> convertToSTObstacles(
    const std::vector<std::unordered_map<std::string, double>>& st_graph) {
    
    std::vector<STObstacle> obstacles;
    for (const auto& node : st_graph) {
        if (node.count("t_in") && node.count("t_out") && 
            node.count("s_in") && node.count("s_out")) {
            obstacles.emplace_back(
                node.at("t_in"), node.at("t_out"),
                node.at("s_in"), node.at("s_out")
            );
        }
    }
    return obstacles;
}


// ==================== STObstacle 成员函数实现 ====================

bool STObstacle::contains(double t, double s, double safety_margin) const {
    if (t < t_in || t > t_out) return false;
    
    // 线性插值得到该时间对应的障碍物位置
    double k = (s_out - s_in) / (t_out - t_in + 1e-6);
    double obs_s = s_in + k * (t - t_in);
    
    return std::abs(s - obs_s) < safety_margin;
}

double STObstacle::minDistanceTo(double t, double s) const {
    Eigen::Vector2d p1(t_in, s_in);
    Eigen::Vector2d p2(t_out, s_out);
    Eigen::Vector2d p(t, s);
    
    Eigen::Vector2d v = p2 - p1;
    Eigen::Vector2d w = p - p1;
    
    double c1 = w.dot(v);
    if (c1 <= 0) return (p - p1).norm();
    
    double c2 = v.dot(v);
    if (c2 <= c1) return (p - p2).norm();
    
    double b = c1 / c2;
    Eigen::Vector2d pb = p1 + b * v;
    return (p - pb).norm();
}

// ==================== PathCostFunction 成员函数实现 ====================

PathCostFunction::PathCostFunction(const WeightCoefficients& weights,
                        const PathPlannerConfig& config,
                                 const std::vector<SLObstacle>& obstacles)
    : _config(config), _static_obstacles(obstacles), _weights(weights) {}

double PathCostFunction::calculateTransitionCost(
    SLState& from, 
    SLState& to
)  {
    double cost = 0.0;
    PolynomialCurve poly_curve;

        // 截断极小的数值误差
    if (std::abs(from.l_prime) < 1e-10) from.l_prime = 0.0;
    if (std::abs(from.l_prime_prime) < 1e-10) from.l_prime_prime = 0.0;
    if (std::abs(to.l_prime) < 1e-10) to.l_prime = 0.0;
    if (std::abs(to.l_prime_prime) < 1e-10) to.l_prime_prime = 0.0;
    
    // // 调试输出
    // static int call_count = 0;
    // if (call_count < 20) {  // 只输出前20次调用
    //     std::cout << "\n[" << call_count << "] 计算转移代价:" << std::endl;
    //     std::cout << "  从: s=" << from.s << ", l=" << from.l 
    //               << ", l'=" << from.l_prime << ", l''=" << from.l_prime_prime << std::endl;
    //     std::cout << "  到: s=" << to.s << ", l=" << to.l 
    //               << ", l'=" << to.l_prime << ", l''=" << to.l_prime_prime << std::endl;
    // }
    
    // 1. 曲线拟合
    bool success = poly_curve.curve_fitting(
        from.s, from.l, from.l_prime, from.l_prime_prime,
        to.s, to.l, to.l_prime, to.l_prime_prime
    );
    
    if (!success) {
        std::cout << "  多项式拟合失败！" << std::endl;
        std::cout << "  从: s=" << from.s << ", l=" << from.l 
                  << ", l'=" << from.l_prime << ", l''=" << from.l_prime_prime << std::endl;
        std::cout << "  到: s=" << to.s << ", l=" << to.l 
                  << ", l'=" << to.l_prime << ", l''=" << to.l_prime_prime << std::endl;
        return std::numeric_limits<double>::max();
    }
    
    // 2. 插值计算
    auto linspace = Eigen::VectorXd::LinSpaced(_config.dp_poly_pnt_num, from.s, to.s);
    
    double cost_ref = 0.0;
    double cost_smooth = 0.0;
    
    std::vector<double> s_set, l_set;
    for (int i = 1; i < linspace.size(); ++i) {
        double s = linspace[i];
        double l = poly_curve.value_evaluation(s, 0);
        double dl = poly_curve.value_evaluation(s, 1);
        double ddl = poly_curve.value_evaluation(s, 2);
        double dddl = poly_curve.value_evaluation(s, 3);
        
        cost_ref += _weights.path_dp_w_ref * l * l;
        cost_smooth += _weights.path_dp_w_dl * dl * dl 
                     + _weights.path_dp_w_ddl * ddl * ddl
                     + _weights.path_dp_w_dddl * dddl * dddl;
        s_set.emplace_back(s);
        l_set.emplace_back(poly_curve.value_evaluation(s, 0));
    }
    
    // 3. 障碍物代价
    double cost_obs = calculateObstacleCost(s_set, l_set);
    
    double total_cost = cost_ref + cost_smooth + cost_obs;
    
    // if (call_count < 20) {
    //     std::cout << "  代价组成: ref=" << cost_ref 
    //               << ", smooth=" << cost_smooth 
    //               << ", obs=" << cost_obs 
    //               << ", 总代价=" << total_cost << std::endl;
    //     std::cout << "  更新后to状态: l'=" << to.l_prime << ", l''=" << to.l_prime_prime << std::endl;
    //     call_count++;
    // }

    // 更新终点的导数，并截断
    // to.l_prime = poly_curve.value_evaluation(to.s, 1);
    // to.l_prime_prime = poly_curve.value_evaluation(to.s, 2);
    // if (std::abs(to.l_prime) < 1e-10) to.l_prime = 0.0;
    // if (std::abs(to.l_prime_prime) < 1e-10) to.l_prime_prime = 0.0;

    return total_cost;
}

double PathCostFunction::calculateHeuristicCost(
    const SLState& state,
    const SLState& goal
) const {
    // 如果没有明确目标，可以不实现或返回0
    return 0.0;
}

double PathCostFunction::calculateObstacleCost(const std::vector<double> s_set, const std::vector<double> l_set) const {
    double cost = 0.0;
    
    for (const auto& obs : _static_obstacles) {
        for (size_t i = 0; i < s_set.size(); i++) {
            double min_dist = calculateDistanceToRectangle(s_set[i], l_set[i], obs);
            // 根据距离计算代价
            if (min_dist <= 0) {
                // 与障碍物重叠，极大代价
                cost += _weights.path_dp_w_obs;
            } else if (min_dist < obs.safety_margin) {
                // 在安全边界内，高代价
                cost += _weights.path_dp_w_obs / (min_dist + 1e-6);
            }
            // 在安全边界外，代价为0
        }
    }
    return cost;
}

double PathCostFunction::calculateDistanceToRectangle(const double& s, const double& l, 
                                                   const SLObstacle& obs) const {
    // 矩形范围
    double s_min = obs.s_center - obs.length / 2.0;
    double s_max = obs.s_center + obs.length / 2.0;
    double l_min = obs.l_center - obs.width / 2.0;
    double l_max = obs.l_center + obs.width / 2.0;
    
    // 检查点是否在矩形内
    if (s >= s_min && s <= s_max &&
        l >= l_min && l <= l_max) {
        return 0.0;  // 点在矩形内
    }
    
    // 计算到矩形边界的最短距离
    double ds = 0.0;
    if (s < s_min) ds = s_min - s;
    else if (s > s_max) ds = s - s_max;
    
    double dl = 0.0;
    if (l < l_min) dl = l_min - l;
    else if (l > l_max) dl = l - l_max;
    
    return std::hypot(ds, dl);
}

// ==================== PathConstraintChecker 成员函数实现 ====================

PathConstraintChecker::PathConstraintChecker(double road_up, double road_low,
                                           const std::vector<SLObstacle>& obstacles)
    : _static_obstacles(obstacles) {}

bool PathConstraintChecker::checkState(const SLState& state) const {

    // 2. 检查障碍物碰撞
    for (const auto& obs : _static_obstacles) {
        if (isCollidingWithObstacle(state, obs)) {
            // std::cout << "[DEBUG] checkState failed: Collision with obstacle" << std::endl;
            return false;
        }
    }
    return true;
}

bool PathConstraintChecker::checkTransition(const SLState& from, const SLState& to) const {
    // 检查路径段是否与障碍物碰撞
    if (!checkInterpolatedPath(from, to)) {
        // std::cout << "[DEBUG] checkTransition failed: Collision in interpolated path" << std::endl;
        return false;
    }
    // 检查dl/ds约束
    double delta_s = to.s - from.s;
    if (delta_s <= 1e-6) return false;// 说明s没有前进，是本层的点，不满足约束

    double lateral_velocity = (to.l - from.l) / delta_s;
    // if (std::abs(lateral_velocity) > _max_l_dot) {
    //     std::cout << "[DEBUG] checkTransition failed: l_dot=" << lateral_velocity 
    //               << " > max=" << _max_l_dot 
    //               << " (from l=" << from.l << ", to l=" << to.l << ", ds=" << delta_s << ")" << std::endl;
    //     return false;
    // }
    return true;
}

bool PathConstraintChecker::isCollidingWithObstacle(const SLState& state, 
                                                  const SLObstacle& obs) const {
    // 考虑安全边界的碰撞检测
    double s_min = obs.s_center - obs.length / 2.0 - obs.safety_margin;
    double s_max = obs.s_center + obs.length / 2.0 + obs.safety_margin;
    double l_min = obs.l_center - obs.width / 2.0 - obs.safety_margin;
    double l_max = obs.l_center + obs.width / 2.0 + obs.safety_margin;
    
    return (state.s >= s_min && state.s <= s_max &&
            state.l >= l_min && state.l <= l_max);
}

bool PathConstraintChecker::checkInterpolatedPath(const SLState& from, const SLState& to) const {
    const int num_intervals = 5;
    for (int i = 1; i <= num_intervals; ++i) {
        double t = static_cast<double>(i) / (num_intervals + 1);
        SLState interp;
        interp.s = from.s + t * (to.s - from.s);
        interp.l = from.l + t * (to.l - from.l);
        // 检查每个插值点
        for (const auto& obs : _static_obstacles) {
            if (isCollidingWithObstacle(interp, obs)) {
                return false;
            }
        }
    }
    return true;
}

// ==================== PathSamplingStrategy 成员函数实现 ====================

PathSamplingStrategy::PathSamplingStrategy(const PathPlannerConfig& config
    ,const std::vector<double>& road_width_left_vec,const std::vector<double>& road_width_right_vec,double resolution_l)
    : _s_step(config.s_sample_distance), _num_layers(config.s_sample_number)
    , _l_samples(config.l_sample_number),_resolution_l(resolution_l){
    if (_l_samples < 2) _l_samples = 2;  // 至少2个采样点
    // 计算l方向采样范围
    // 以车道线(参考线)为中心，横向均匀
    // 需要通过s方向采样距离来确定对应的l采样范围,
    for(int i=0;i<_num_layers;++i){
        double s = i*_s_step;
        // 通过s找到对应的road_width_left和road_width_right
        int index = static_cast<int>(s/_resolution_l);//默认是1m分辨率
        if (index >= static_cast<int>(road_width_left_vec.size())) {
            index = road_width_left_vec.size() - 1;
        }
        if (index < 0) index = 0;
        
        _road_width_left_vec.push_back(road_width_left_vec[index]);
        _road_width_right_vec.push_back(road_width_right_vec[index]);
    }
}

std::vector<SLState> PathSamplingStrategy::generateNextLayer(
    const std::vector<SLState>& current_layer,
    int layer_index
) const {
    std::vector<SLState> next_states;
    double s_value = (layer_index + 1) * _s_step;
    double left_bound = _road_width_left_vec[layer_index+1];
    double right_bound = _road_width_right_vec[layer_index+1];

    // 确保至少有一个采样点（0点）
    if (_l_samples <= 0) return next_states;

    // 计算左右两侧的采样点数
    // 假设0点必选，剩余点数按左右宽度比例分配
    int n_total = _l_samples;
    int n_right = 0;
    int n_left = 0;

    if (n_total > 1) {
        double total_width = left_bound - right_bound;
        if (total_width > 1e-3) {
            // 使用std::abs确保宽度为正
            n_right = std::round((n_total - 1) * std::abs(right_bound) / total_width);
            n_left = n_total - 1 - n_right;
        } else {
            n_right = (n_total - 1) / 2;
            n_left = n_total - 1 - n_right;
        }
    }

    // 1. 生成右侧采样点 (从右边界向0逼近，保持l递增顺序)
    if (n_right > 0) {
        double step = std::abs(right_bound) / n_right;
        for (int i = n_right; i >= 1; --i) {
            SLState next_state;
            next_state.s = s_value;
            next_state.l = -i * step; // right_bound 是负数，这里生成负数
            next_state.l_prime = 0.0;
            next_state.l_prime_prime = 0.0;
            next_states.push_back(next_state);
        }
    }

    // 2. 生成中心点 (0)
    {
        SLState next_state;
        next_state.s = s_value;
        next_state.l = 0.0;
        next_state.l_prime = 0.0;
        next_state.l_prime_prime = 0.0;
        next_states.push_back(next_state);
    }

    // 3. 生成左侧采样点 (从0向左边界逼近)
    if (n_left > 0) {
        double step = left_bound / n_left;
        for (int i = 1; i <= n_left; ++i) {
            SLState next_state;
            next_state.s = s_value;
            next_state.l = i * step;
            next_state.l_prime = 0.0;
            next_state.l_prime_prime = 0.0;
            next_states.push_back(next_state);
        }
    }
    return next_states;
}

std::vector<std::vector<SLState>> PathSamplingStrategy::generateSamplingGrid(
    const SLState& start_state,
    int num_layers
) const {
    // 继承模版基类不能改变函数签名，只能在函数体内处理

    //这里不处理起点，因为每个规划周期起点不一样，但是后面的网格是一样的 
    std::vector<std::vector<SLState>> grid;

    for (int i = 0; i < _num_layers; ++i) {
        // generateNextLayer 生成的是相对起点的s (从 _s_step 开始)
        // 我们需要加上 start_state.s
        std::vector<SLState> dummy_layer; // generateNextLayer 不使用 current_layer
        auto next_layer = generateNextLayer(dummy_layer, i);
        
        if (next_layer.empty()) break;
        
        // 加上起点s偏移
        for (auto& state : next_layer) {
            state.s += start_state.s;
        }
        
        grid.push_back(next_layer);
    }
    
    return grid;
}

// ==================== SpeedCostFunction 成员函数实现 ====================
SpeedCostFunction::SpeedCostFunction(const WeightCoefficients& weights,
                                    const SpeedPlannerConfig& config,
                                   double reference_speed,
                                   const std::vector<STObstacle>& obstacles) 
    : _weights(weights), _reference_speed(reference_speed), _st_obstacles(obstacles),_dp_poly_pnt_num(config.dp_poly_pnt_num) {}

double SpeedCostFunction::calculateTransitionCost(
    STState& from, 
    STState& to
) {
    // 计算完了需要保存to的速度和加速度供下一次计算使用
    double delta_t = to.t - from.t;
    if (delta_t <= 1e-6) {
        std::cout << "  ERROR: delta_t太小或为0" << std::endl;
        return std::numeric_limits<double>::max();
    }
    // 计算速度
    double s_dot = (to.s - from.s) / delta_t;
    // 计算加速度
    double s_dot_dot = (s_dot - from.s_dot) / delta_t;
    // 3. 加加速度（jerk）代价
    double s_dot_dot_dot = (s_dot_dot - from.s_dot_dot) / delta_t;
    // 更新to的速度和加速度    
    // 4. 更新 to 状态的速度和加速度（确保状态一致性）
    // 再次截断极小值
    if (std::abs(to.s_dot) < 1e-10) {to.s_dot = 0.0;}
    else{to.s_dot = s_dot;}
    if (std::abs(to.s_dot_dot) < 1e-10) {to.s_dot_dot = 0.0;}
    else{to.s_dot_dot = s_dot_dot;}

    auto t_linspace = Eigen::VectorXd::LinSpaced(_dp_poly_pnt_num, from.t, to.t);

    double cost_ref_speed = 0.0;
    double cost_a = 0.0;
    double cost_jerk = 0.0;
    for (int i = 0; i < t_linspace.size(); ++i) {
        double t = t_linspace[i];
        double alpha = (t - from.t) / delta_t;  // 归一化参数，0到1之间
        // 使用线性插值计算s
        double s = from.s + alpha * (to.s - from.s);
        // 速度：使用递推公式计算的值（全程恒定）
        double v = s_dot;
        // 加速度：使用递推公式计算的值（全程恒定）
        double a = s_dot_dot;
        // 加加速度：使用递推公式计算的值（全程恒定）
        double jerk = s_dot_dot_dot;
        // 参考速度代价（每点）
        cost_ref_speed += _weights.speed_dp_w_ref_speed * std::pow(v - _reference_speed, 2);
        // 加速度硬约束 + 二次代价
        if (a > 4.0 || a < -6.0) {
            // 超出物理极限：施加极大惩罚
            cost_a += 1e6;
        } else {
            cost_a += _weights.speed_dp_w_a * std::pow(a, 2);
        }
        // Jerk 代价
        cost_jerk += _weights.speed_dp_w_jerk * std::pow(jerk, 2);
    }

    // 2. 障碍物代价：基于线性插值采样
    std::vector<std::pair<double, double>> sample_points;
    for (int i = 0; i < t_linspace.size(); ++i) {
        double t = t_linspace[i];
        double alpha = (t - from.t) / delta_t;
        double s = from.s + alpha * (to.s - from.s);
        sample_points.emplace_back(t, s);
    }
    
    double cost_obs = calculateObstacleCost(sample_points);
    // 5. 总代价
    double total_cost = cost_ref_speed + cost_a + cost_jerk + cost_obs;
    return total_cost;
}

double SpeedCostFunction::calculateObstacleCost(
    const std::vector<std::pair<double, double>>& sample_points
) const {
    double cost_obs = 0.0;
    if (_st_obstacles.empty()) {
        return cost_obs;
    }

    // 对每个采样点计算障碍物代价
    for (const auto& sample : sample_points) {
        double t_sample = sample.first;
        double s_sample = sample.second;
        // 检查是否在障碍物内部（使用contains方法）
        bool inside_obstacle = false;
        for (const auto& obs : _st_obstacles) {
            if (obs.contains(t_sample, s_sample, 2.0)) {
                // 如果采样点在障碍物内部，使用固定权重代价
                cost_obs += _weights.speed_dp_w_obs;
                inside_obstacle = true;
                break;  // 一个点只需要计算一次障碍物代价
            }
        }
        // 如果不在障碍物内部，计算最小距离
        if (!inside_obstacle) {
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& obs : _st_obstacles) {
                double distance = obs.minDistanceTo(t_sample, s_sample);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
            // 根据距离计算代价（与原代码一致）
            if (min_distance < 2.0) {
                // 如果距离小于2米，即使不在障碍物内部，也给予较大代价
                cost_obs += _weights.speed_dp_w_obs;
            } else if (min_distance <= 3.0) {
                // 2-3米之间，使用平方反比关系
                cost_obs += 1000.0 / (min_distance * min_distance);
            }
            // 距离大于3米，代价为0
        }
    }
    return cost_obs;
}

// ==================== SpeedConstraintChecker 成员函数实现 ====================

SpeedConstraintChecker::SpeedConstraintChecker(const SpeedPlannerConfig& config,
                                             const std::vector<STObstacle>& obstacles)
    :_st_obstacles(obstacles),_max_acceleration(config.max_acceleration)
    ,_max_deceleration(config.max_deceleration),_max_jerk(config.max_jerk),_max_s(config.s_max) {}


bool SpeedConstraintChecker::checkState(const STState& state) const {
    // 1. 检查s边界
    if (state.s < 0 || state.s >_max_s) {
        return false;
    }
    // 4. 检查是否与障碍物冲突
    for (const auto& obs : _st_obstacles) {
        if (obs.contains(state.t, state.s)) {
            return false;
        }
    }
    return true;
}

bool SpeedConstraintChecker::checkTransition(const STState& from, const STState& to) const {
    // 这里就不用更新to的状态了，因为不合格的就被剔除掉了，下一步留下来的会计算状态转移代价，并且更新to的状态
    double delta_t = to.t - from.t;
    if (delta_t <= 1e-6) return false;
    // 检查加速度变化率（jerk）
    double s_dot = (to.s - from.s) / delta_t;
    // 不能倒车,可以停下来
    if(s_dot<-1e-6)return false;
    double s_dot_dot = (s_dot - from.s_dot) / delta_t;
    // 加速度也不能超过限制
    if(s_dot_dot>_max_acceleration||s_dot_dot<_max_deceleration)return false;

    double jerk = (s_dot_dot - from.s_dot_dot) / delta_t;
    
    if (std::abs(jerk) > _max_jerk) {
        // std::cout << "[DEBUG] checkTransition失败: jerk=" << jerk 
        //           << " > max_jerk=" << _max_jerk << std::endl;
        // std::cout << "  从: t=" << from.t << ", s=" << from.s 
        //           << ", v=" << from.s_dot << ", a=" << from.s_dot_dot << std::endl;
        // std::cout << "  到: t=" << to.t << ", s=" << to.s 
        //           << ", v=" << s_dot << ", a=" << s_dot_dot << std::endl;
        // std::cout << "  计算值: v_calc=" << s_dot << ", a_calc=" << s_dot_dot << std::endl;
        return false;
    }
    
    // 检查整个转移路径是否与障碍物冲突
    return checkInterpolatedPath(from, to);
}

bool SpeedConstraintChecker::checkInterpolatedPath(const STState& from, const STState& to) const {
    const int num_intervals = 5;
    
    for (int i = 1; i <= num_intervals; ++i) {
        double t = from.t + i * (to.t - from.t) / (num_intervals + 1);
        double s = from.s + (to.s - from.s) * i / (num_intervals + 1);
        // 检查每个插值点
        for (const auto& obs : _st_obstacles) {
            if (obs.contains(t, s)) {
                // std::cout<<from.t<<","<<from.s<<"->"<<to.t<<","<<to.s<<"转移路径上有障碍物"<<std::endl;
                return false;
            }
        }
    }
    return true;
}

// ==================== SpeedSamplingStrategy 成员函数实现 ====================

SpeedSamplingStrategy::SpeedSamplingStrategy(const SpeedPlannerConfig& config)
    : _t_step(config.t_step_init),_s_step(config.s_step_init)
    ,  _max_t(config.t_max),_max_s(config.s_max),
     _increase_ratio(config.increase_ratio) {
        // std::cout<<"初始化\n";
        _t_series.clear();
        _s_serier.clear();
        // 时间轴和s轴采样序列是可以固定的，先计算，后面都不会变化
        _t_series=_sample_s_t_method(_max_t,_t_step);
        // 同时去除第一列0点，因为每次规划的起点都是0,0
        if(!_t_series.empty()&&_t_series[0]==0.0){
            _t_series.erase(_t_series.begin());
        }
        _s_serier=_sample_s_t_method(_max_s,_s_step);
        // std::cout<<"初始化完成\n";
        // std::cout << "时间序列长度: " << _t_series.size() << std::endl;
        // std::cout << "s序列长度: " << _s_serier.size() << std::endl;
    }

std::vector<STState> SpeedSamplingStrategy::generateNextLayer(
    const std::vector<STState>& current_layer,
    int layer_index
) const {
    std::vector<STState> next_states;
    for(int i=0;i<_s_serier.size();++i){
        STState next_state;
        next_state.s = _s_serier[i];
        next_state.t = _t_series[layer_index];
        next_state.s_dot = 0.0;
        next_state.s_dot_dot = 0.0;
        next_states.push_back(next_state);
    }
    return next_states;
}

std::vector<std::vector<STState>> SpeedSamplingStrategy::generateSamplingGrid(
        const STState& start_state,
        int num_layers
) const  {
    // 生成从当前s开始的前向采样点
    // 每次的起点状态都是s=0,t=0，而且层之间没有关系，只与层号有关
    std::vector<std::vector<STState>> grid;
    // 首先是不处理起点，因为每个规划周期起点都是0,0，但是后面的网格是一样的
    for (int i = 0; i < _t_series.size(); ++i) {
        auto next_layer = generateNextLayer(grid.back(),i);
        if (next_layer.empty()) break;
        grid.push_back(next_layer);
    }
    return grid; 
}

// 定义s,t轴的采样序列生成方法
std::vector<double> SpeedSamplingStrategy::_sample_s_t_method(double max_border,double init_step) const {
    std::vector<double> serier;
    double current = 0.0;
    int max_points = 1000;
    double ratio = _increase_ratio;
    if (ratio < 0) ratio = 0.0;
    int count = 0;
    while (current <= max_border && serier.size() < max_points) {
        serier.push_back(current);
        current += init_step * (1 + ratio * serier.size());
        if (count++ > 1000) break; // 安全措施
    }
    return serier;
}
// ==================== SpeedDpPlanner 成员函数实现 ====================
SpeedDpPlanner::SpeedDpPlanner(
        std::shared_ptr<CostFunctionStrategy<STState>> cost_function,
        std::shared_ptr<ConstraintCheckerStrategy<STState>> constraint_checker,
        std::shared_ptr<SamplingStrategy<STState>> sampling_strategy,
        std::shared_ptr<BacktrackStrategy<STState>> backtrack_strategy,
        const DpPlannerConfig& config
    ): DpPlanner<STState>(cost_function, constraint_checker,sampling_strategy, backtrack_strategy, config){}


void SpeedDpPlanner::forwardSearch(
    const std::vector<std::vector<STState>>& sampling_grid,
    const STState& start_state
){
    // 初始化搜索树
    _search_tree.clear();
    if(_grid_max_s==0){
       _grid_max_s=sampling_grid.back().back().s; 
    }
    // 创建起始节点
    std::vector<std::shared_ptr<DpNode<STState>>> start_layer;
    start_layer.emplace_back(std::make_shared<DpNode<STState>>(start_state, 0.0, nullptr, 0, 0));
    _search_tree.push_back(start_layer);
    // 还是要走到终点，看哪个点的代价低
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

DpNode<STState> SpeedDpPlanner::findOptimalNode(){
    // 遍历搜索树的最后一层和每一层的s最大的节点(s最大的节点一定是最后一个节点)
    std::shared_ptr<DpNode<STState>> optimal_node;
    double min_cost=std::numeric_limits<double>::infinity();
    // std::cout<<"S边界："<<_grid_max_s<<std::endl;
    // std::cout<<"搜索树层数："<<_search_tree.size()<<std::endl;

    for(size_t i=0;i<_search_tree.size();i++){
        const auto& temp=_search_tree[i].back();
        // std::cout<<"当前节点的s:"<<temp->state.s<<std::endl;
        if((std::abs(temp->state.s-_grid_max_s)<1e-2)&&(temp->cumulative_cost<min_cost)){
            min_cost=temp->cumulative_cost;
            optimal_node=temp;
            // std::cout<<"找到一个在S边界上的最优节点，代价为："<<min_cost<<
            // "\n节点信息(layer,s,t)："<<optimal_node->layer_index<<" , "<<
            // optimal_node->state.s<<" , "<<optimal_node->state.t
            // <<std::endl;
        }
    }

    if(!_search_tree.back().empty()){
        const auto& last_layer=_search_tree.back();
        if (last_layer.empty()) {
            throw std::runtime_error("Last layer is empty");
        }
        for(size_t j=0;j<last_layer.size();j++){
            // std::cout<<"当前节点的s:"<<last_layer[j]->state.s<<std::endl;
            if((last_layer[j]->cumulative_cost)<min_cost){
                min_cost=last_layer[j]->cumulative_cost;
                optimal_node=last_layer[j];
            }
        }
    }

    if(min_cost!=std::numeric_limits<double>::infinity()){
        _succes=true;
        return *optimal_node;
    }
    else{
        return DpNode<STState>();
    }
}












void testPolynomialCurve() {
    using namespace std;
    std::cout << "\n=== 测试多项式曲线拟合 ===" << std::endl;
    
    AD_algorithm::general::PolynomialCurve curve;
    
    // 测试1: 直线 (起点和终点导数都为0) - 使用坐标平移
    double x1 = 60.0, y1 = 0.0, dy1 = 0.0, ddy1 = 0.0;
    double x2 = 70.0, y2 = -2.33333, dy2 = 0.0, ddy2 = 0.0;
    
    std::cout << "测试1: 从第6层到第7层的典型转移" << std::endl;
    std::cout << "起点: (" << x1 << ", " << y1 << "), 导数: " << dy1 << ", " << ddy1 << std::endl;
    std::cout << "终点: (" << x2 << ", " << y2 << "), 导数: " << dy2 << ", " << ddy2 << std::endl;
    
    bool success = curve.curve_fitting(x1, y1, dy1, ddy1, x2, y2, dy2, ddy2);
    if (success) {
        std::cout << "拟合成功!" << std::endl;
        std::cout << "多项式系数: ";
        auto coeffs = curve.getCoefficients();
        for (int i = 0; i < coeffs.size(); ++i) {
            std::cout << coeffs[i] << " ";
        }
        std::cout << std::endl;
        
        // 在起点和终点评估
        std::cout << "起点(" << x1 << ")值: " << curve.value_evaluation(x1, 0) 
                  << ", 一阶导数: " << curve.value_evaluation(x1, 1) 
                  << ", 二阶导数: " << curve.value_evaluation(x1, 2) << std::endl;
        std::cout << "终点(" << x2 << ")值: " << curve.value_evaluation(x2, 0) 
                  << ", 一阶导数: " << curve.value_evaluation(x2, 1) 
                  << ", 二阶导数: " << curve.value_evaluation(x2, 2) << std::endl;
        
        // 在中间点评估
        double mid_x = (x1 + x2) / 2.0;
        std::cout << "中点(" << mid_x << ")值: " << curve.value_evaluation(mid_x, 0) << std::endl;
        std::cout << "中点一阶导数: " << curve.value_evaluation(mid_x, 1) << std::endl;
        std::cout << "中点二阶导数: " << curve.value_evaluation(mid_x, 2) << std::endl;
        std::cout << "中点三阶导数: " << curve.value_evaluation(mid_x, 3) << std::endl;
    } else {
        std::cout << "拟合失败!" << std::endl;
    }
    
    // 测试2: 带有小数值误差的情况
    std::cout << "\n测试2: 带有数值误差的情况" << std::endl;
    dy1 = -1.09139e-11; ddy1 = -9.09495e-13;
    success = curve.curve_fitting(x1, y1, dy1, ddy1, x2, y2, dy2, ddy2);
    std::cout << "带有数值误差的导数: " << (success ? "成功" : "失败") << std::endl;
    
    // 测试3: 曲线 (起点终点导数非0)
    std::cout << "\n测试3: 曲线拟合" << endl;
    dy1 = 0.1; ddy1 = 0.01;
    dy2 = -0.05; ddy2 = -0.02;
    success = curve.curve_fitting(x1, y1, dy1, ddy1, x2, y2, dy2, ddy2);
    double mid_x=(x1+y1)/2;
    if (success) {
        std::cout << "曲线拟合成功" << std::endl;
        std::cout << "中点(" << mid_x << ")三阶导数: " << curve.value_evaluation(mid_x, 3) << std::endl;
    }
}



void path_DP(){
    // 1. 创建测试数据
    WeightCoefficients weights;
    // 初始化权重系数（这里使用默认值）
    
    std::vector<SLObstacle> path_obstacles;
    path_obstacles.emplace_back(30.0, 1.5);   // 障碍物1
    path_obstacles.emplace_back(60.0, -1.0);  // 障碍物2
    
    // 2. 创建路径规划配置
    PathPlannerConfig path_config;
    path_config.s_sample_distance = 10;  // s方向采样间隔
    path_config.s_sample_number = 10;       // 规划层数
    path_config.lane_width = 7.0;             // 路宽
    path_config.l_sample_number = 7;       // l方向采样点数，应该是奇数
    
    std::vector<double> road_width_left(200, 3.5);
    std::vector<double> road_width_right(200, -3.5);

    // 3. 创建路径规划策略
    auto path_cost_func = std::make_shared<PathCostFunction>(weights, path_config, path_obstacles);
    auto path_constraints = std::make_shared<PathConstraintChecker>(4.0, -4.0, path_obstacles);
    auto path_sampling = std::make_shared<PathSamplingStrategy>(path_config, road_width_left, road_width_right);
    auto path_backtrack = std::make_shared<planner::DefaultBacktrackStrategy<SLState>>();
    
    // 4. 配置DP规划器
    planner::DpPlannerConfig config;
    config.max_layers = path_config.s_sample_number+1;
    config.enable_pruning = false;
    config.pruning_keep_num = 30;
    config.debug_mode = true;
    config.time_limit_ms = 1000.0;
    config.store_full_tree=true;

    // 5. 创建路径DP规划器
    planner::DpPlanner<SLState> path_planner(
        path_cost_func,
        path_constraints,
        path_sampling,
        path_backtrack,
        config
    );
    
// 6. 调试：打印权重值
    std::cout << "\n=== 使用的权重系数 ===" << std::endl;
    std::cout << "path_dp_w_ref: " << weights.path_dp_w_ref << std::endl;
    std::cout << "path_dp_w_dl: " << weights.path_dp_w_dl << std::endl;
    std::cout << "path_dp_w_ddl: " << weights.path_dp_w_ddl << std::endl;
    std::cout << "path_dp_w_dddl: " << weights.path_dp_w_dddl << std::endl;
    std::cout << "path_dp_w_obs: " << weights.path_dp_w_obs << std::endl;
    
    // 7. 执行路径规划
    SLState start_sl(0.0, 0.0, 0.0, 0.0);
    SLState goal_sl;
    
    // 生成采样网格
    auto grid = path_sampling->generateSamplingGrid(start_sl, 0);
    
    std::cout << "\n=== 采样网格 ===" << std::endl;
    std::cout << "总层数: " << grid.size() << std::endl;
    for (size_t i = 0; i < grid.size(); i++) {
        std::cout << "第" << i+1 << "层 (s=" << (i+1)*path_config.s_sample_distance << "): ";
        for (size_t j = 0; j < grid[i].size(); j++) {
            std::cout << std::setw(8) << grid[i][j].l;
        }
        std::cout << std::endl;
    }
    
    // 执行规划
    auto path_result = path_planner.planWithGrid(grid, start_sl, goal_sl);
    
    if (path_result.success) {
        std::cout << "路径规划成功!" << std::endl;
        std::cout << "总代价: " << path_result.total_cost << std::endl;
        std::cout << "路径点数: " << path_result.optimal_path.size() << std::endl;
        std::cout << "计算时间: " << path_result.computation_time_ms << " ms" << std::endl;
    } else {
        std::cout << "路径规划失败: " << path_result.message << std::endl;
    }
    

    // 方法1：使用默认打印
    // path_planner.printSearchTree();

    // 方法2：使用自定义状态打印函数
    path_planner.printSearchTree([](const SLState& state) {
        std::cout << "s=" << std::setw(6) << state.s 
                  << ", l=" << std::setw(6) << state.l 
                  << ", l'=" << std::setw(6) << state.l_prime
                  << ", l''=" << std::setw(6) << state.l_prime_prime;
    });

}


void speed_DP_debug() {
    std::cout << "\n=== 速度规划调试Demo ===" << std::endl;
    
    // 1. 创建权重系数
    WeightCoefficients weights;
    std::cout << "=== 速度规划权重系数 ===" << std::endl;
    std::cout << "speed_dp_w_ref_speed: " << weights.speed_dp_w_ref_speed << std::endl;
    std::cout << "speed_dp_w_a: " << weights.speed_dp_w_a << std::endl;
    std::cout << "speed_dp_w_jerk: " << weights.speed_dp_w_jerk << std::endl;
    std::cout << "speed_dp_w_obs: " << weights.speed_dp_w_obs << std::endl;
    
    // 2. 创建速度规划配置
    SpeedPlannerConfig speed_config;
    // 使用合理的参数，避免采样点过多
    speed_config.t_step_init = 0.5;        // 时间步长
    speed_config.s_step_init=0.3;
    speed_config.t_max = 8.0;             // 最大时间
    speed_config.s_max = 30.0;            // 最大距离
    speed_config.increase_ratio = 0.1;    // 递增比例
    speed_config.max_acceleration = 4.0;
    speed_config.max_deceleration = -6.0;
    speed_config.max_jerk = 10.0;
    speed_config.dp_poly_pnt_num = 3;   // 减少插值点数
    
    std::cout << "\n=== 速度规划配置 ===" << std::endl;
    std::cout << "最大时间: " << speed_config.t_max << " s" << std::endl;
    std::cout << "最大距离: " << speed_config.s_max << " m" << std::endl;
    std::cout << "时间初始步长: " << speed_config.t_step_init << " s" << std::endl;
    std::cout << "距离初始步长: " << speed_config.s_step_init << " m" << std::endl;
    std::cout << "递增比例: " << speed_config.increase_ratio << std::endl;
    std::cout << "线性插值点数: " << speed_config.dp_poly_pnt_num << std::endl;
    
    // 3. 创建障碍物
    std::vector<STObstacle> speed_obstacles;
    // 障碍物1：在t=1-3s，s=5-15m区域
    speed_obstacles.emplace_back(2.0, 3.0, 15.0, 20.0);
    // // 障碍物2：在t=3-4s，s=20-25m区域
    speed_obstacles.emplace_back(5.0, 6.0, 20.0, 25.0);
    
    std::cout << "\n=== 障碍物信息 ===" << std::endl;
    for (size_t i = 0; i < speed_obstacles.size(); ++i) {
        const auto& obs = speed_obstacles[i];
        std::cout << "障碍物" << i << ": t=[" << obs.t_in << ", " << obs.t_out 
                  << "], s=[" << obs.s_in << ", " << obs.s_out << "]" << std::endl;
    }
    
    // 4. 创建速度规划策略
    double reference_speed = 8.0;
    
    std::cout << "\n创建速度规划策略..." << std::endl;
    std::cout << "\n创建代价策略..." << std::endl;
    auto speed_cost_func = std::make_shared<SpeedCostFunction>(
        weights, speed_config, reference_speed, speed_obstacles);
    std::cout << "\n创建约束检查策略..." << std::endl;
    auto speed_constraints = std::make_shared<SpeedConstraintChecker>(speed_config, speed_obstacles);
    std::cout << "\n创建采样策略..." << std::endl;
    auto speed_sampling = std::make_shared<SpeedSamplingStrategy>(speed_config);
    std::cout << "\n创建回溯策略..." << std::endl;
    auto speed_backtrack = std::make_shared<planner::DefaultBacktrackStrategy<STState>>();


    // 7. 生成采样网格
    STState start_st(0.0, 0.0, 5.0, 1.0);  // 起点：t=0, s=0, v=5m/s, a=0
    STState goal_st;  // 目标状态（可选）
    
    std::cout << "\n=== 起点状态 ===" << std::endl;
    std::cout << "t=" << start_st.t << "s, s=" << start_st.s 
              << "m, v=" << start_st.s_dot << "m/s, a=" << start_st.s_dot_dot << "m/s²" << std::endl;
    
    auto grid = speed_sampling->generateSamplingGrid(start_st, 0);

    // 5. 配置DP规划器
    planner::DpPlannerConfig config;
    config.max_layers = grid.size()+1;                // 减少层数，便于调试
    config.enable_pruning = false;                  // 关闭剪枝，查看完整搜索树
    config.pruning_keep_num = 2;
    config.debug_mode = true;
    config.time_limit_ms = 5000.0;
    config.store_full_tree = true;        // 存储完整搜索树
    
    std::cout << "\n=== DP规划器配置 ===" << std::endl;
    std::cout << "最大层数: " << config.max_layers << std::endl;
    std::cout << "启用剪枝: " << (config.enable_pruning ? "是" : "否") << std::endl;
    std::cout << "剪枝保留数: " << config.pruning_keep_num << std::endl;
    std::cout << "存储搜索树: " << (config.store_full_tree ? "是" : "否") << std::endl;
    
    // 6. 创建速度DP规划器
    std::cout << "\n创建速度DP规划器..." << std::endl;
    SpeedDpPlanner speed_planner(
        speed_cost_func,
        speed_constraints,
        speed_sampling,
        speed_backtrack,
        config
    );
    

    // 8. 打印采样网格信息
    std::cout << "\n=== 采样网格信息 ===" << std::endl;
    std::cout << "网格层数: " << grid.size() << std::endl;
    
    if (grid.empty()) {
        std::cout << "错误：采样网格为空！" << std::endl;
        return;
    }
    
    // 显示每层的采样点数量
    for (size_t i = 0; i < grid.size(); ++i) {
        if (i < 3 || i >= grid.size() - 3) {  // 显示前3层和后3层
            std::cout << "第" << std::setw(2) << i << "层: " << grid[i].size() << "个点";
            
            if (!grid[i].empty()) {
                // 显示该层的时间范围
                double min_t = grid[i][0].t;
                double max_t = grid[i][0].t;
                double min_s = grid[i][0].s;
                double max_s = grid[i][0].s;
                
                for (const auto& state : grid[i]) {
                    if (state.t < min_t) min_t = state.t;
                    if (state.t > max_t) max_t = state.t;
                    if (state.s < min_s) min_s = state.s;
                    if (state.s > max_s) max_s = state.s;
                }
                
                std::cout << "，t∈[" << min_t << ", " << max_t << "]，s∈[" 
                          << min_s << ", " << max_s << "]";
                
                // 显示前几个点的详细信息
                if (i < 2) {
                    std::cout << "\n    前3个点: ";
                    for (int j = 0; j < std::min(3, (int)grid[i].size()); ++j) {
                        std::cout << "(t=" << grid[i][j].t << ",s=" << grid[i][j].s << ") ";
                    }
                }
            }
            std::cout << std::endl;
        } else if (i == 3) {
            std::cout << "... 中间层省略 ..." << std::endl;
        }
    }
    
    // 9. 执行规划
    std::cout << "\n=== 开始速度规划 ===" << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto result = speed_planner.planWithGrid(grid, start_st, goal_st);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // 10. 输出规划结果
    std::cout << "\n=== 规划结果 ===" << std::endl;
    std::cout << "成功: " << (result.success ? "是" : "否") << std::endl;
    std::cout << "消息: " << result.message << std::endl;
    std::cout << "总代价: " << result.total_cost << std::endl;
    std::cout << "路径点数: " << result.optimal_path.size() << std::endl;
    std::cout << "扩展节点数: " << result.num_nodes_expanded << std::endl;
    std::cout << "计算时间: " << result.computation_time_ms << " ms" << std::endl;
    std::cout << "实际耗时: " << duration.count() << " ms" << std::endl;
    
    // 11. 打印最优路径
    if (result.success && !result.optimal_path.empty()) {
        std::cout << "\n=== 最优速度路径 ===" << std::endl;
        std::cout << "序号 | 时间(t) | 距离(s) | 速度(v) | 加速度(a)" << std::endl;
        std::cout << "-----------------------------------------------" << std::endl;
        
        for (size_t i = 0; i < result.optimal_path.size(); ++i) {
            const auto& state = result.optimal_path[i];
            std::cout << std::setw(3) << i << " | "
                      << std::setw(7) << std::fixed << std::setprecision(2) << state.t << " | "
                      << std::setw(7) << state.s << " | "
                      << std::setw(7) << state.s_dot << " | "
                      << std::setw(7) << state.s_dot_dot << std::endl;
        }
        
        // 分析路径特性
        std::cout << "\n=== 路径特性分析 ===" << std::endl;
        if (result.optimal_path.size() > 1) {
            double total_distance = result.optimal_path.back().s - result.optimal_path.front().s;
            double total_time = result.optimal_path.back().t - result.optimal_path.front().t;
            double avg_speed = total_distance / total_time;
            
            std::cout << "总距离: " << total_distance << " m" << std::endl;
            std::cout << "总时间: " << total_time << " s" << std::endl;
            std::cout << "平均速度: " << avg_speed << " m/s" << std::endl;
            std::cout << "参考速度: " << reference_speed << " m/s" << std::endl;
            
            // 检查是否满足约束
            std::cout << "\n=== 约束检查 ===" << std::endl;
            bool constraints_ok = true;
            for (size_t i = 0; i < result.optimal_path.size(); ++i) {
                const auto& state = result.optimal_path[i];
                
                // 检查速度非负
                if (state.s_dot < 0) {
                    std::cout << "❌ 点" << i << ": 速度负值 (" << state.s_dot << " m/s)" << std::endl;
                    constraints_ok = false;
                }
                
                // 检查加速度范围
                if (state.s_dot_dot > speed_config.max_acceleration || 
                    state.s_dot_dot < speed_config.max_deceleration) {
                    std::cout << "❌ 点" << i << ": 加速度超出范围 (" 
                              << state.s_dot_dot << " m/s²)" << std::endl;
                    constraints_ok = false;
                }
            }
            
            if (constraints_ok) {
                std::cout << "✅ 所有点满足基本约束" << std::endl;
            }
            
            // 检查障碍物避让
            std::cout << "\n=== 障碍物避让检查 ===" << std::endl;
            bool obstacle_ok = true;
            for (size_t i = 0; i < result.optimal_path.size(); ++i) {
                const auto& state = result.optimal_path[i];
                for (size_t j = 0; j < speed_obstacles.size(); ++j) {
                    const auto& obs = speed_obstacles[j];
                    if (obs.contains(state.t, state.s, 1.0)) {  // 安全边界1.0米
                        std::cout << "❌ 点" << i << "(t=" << state.t << ", s=" << state.s 
                                  << ") 在障碍物" << j << "区域内！" << std::endl;
                        obstacle_ok = false;
                    }
                }
            }
            
            if (obstacle_ok) {
                std::cout << "✅ 路径成功避开所有障碍物" << std::endl;
            }
        }
    }
    
    // 12. 打印搜索树
    // 使用printSearchTree方法
    speed_planner.printSearchTree([](const STState& state) {
        std::cout << "t=" << std::setw(5) << std::fixed << std::setprecision(2) << state.t 
                  << "s, s=" << std::setw(6) << state.s 
                  << "m, v=" << std::setw(6) << state.s_dot 
                  << "m/s, a=" << std::setw(6) << state.s_dot_dot << "m/s²";
    });
    
    // 13. 如果有存储搜索树，也可以直接访问result.search_tree
    if (config.store_full_tree && !result.search_tree.empty()) {
        std::cout << "\n=== 详细搜索树分析 ===" << std::endl;
        
        // 只分析前几层和后几层
        std::vector<int> layers_to_analyze = {0, 1, 2, -2, -1}; // -1表示最后一层，-2表示倒数第二层
        
        for (int layer_idx : layers_to_analyze) {
            int actual_idx = layer_idx;
            if (layer_idx < 0) {
                actual_idx = result.search_tree.size() + layer_idx;
            }
            
            if (actual_idx >= 0 && actual_idx < static_cast<int>(result.search_tree.size())) {
                const auto& layer = result.search_tree[actual_idx];
                
                std::cout << "\n第 " << actual_idx << " 层分析:" << std::endl;
                std::cout << "  节点总数: " << layer.size() << std::endl;
                
                if (!layer.empty()) {
                    // 找出本层最优节点
                    auto best_node_it = std::min_element(layer.begin(), layer.end(),
                        [](const auto& a, const auto& b) {
                            return a->cumulative_cost < b->cumulative_cost;
                        });
                    
                    const auto& best_node = *best_node_it;
                    std::cout << "  最优节点: [" << best_node->layer_index << "," 
                              << best_node->node_index << "]" << std::endl;
                    std::cout << "  最优代价: " << best_node->cumulative_cost << std::endl;
                    std::cout << "  最优状态: t=" << best_node->state.t << ", s=" << best_node->state.s 
                              << ", v=" << best_node->state.s_dot << ", a=" << best_node->state.s_dot_dot << std::endl;
                    
                    // 如果有父节点
                    if (best_node->parent) {
                        std::cout << "  父节点: [" << best_node->parent->layer_index << ","
                                  << best_node->parent->node_index << "]" << std::endl;
                    }
                    
                }
            }
        }
    }
    
    std::cout << "\n=== 速度规划调试完成 ===" << std::endl;
}

void demo() {
    std::cout << "=== DP模板使用示例 ===" << std::endl;
    // 先测试多项式曲线
    // testPolynomialCurve();
    path_DP();
    // // 暂停一下，然后测试速度规划
    // std::cout << "\n按Enter键继续测试速度规划..." << std::endl;
    // std::cin.get();
    // speed_DP_debug();    
    std::cout << "\n测试完成。" << std::endl;
}

// 显式实例化模板类，以解决链接错误
template class DpPlanner<SLState>;
template class DpPlanner<STState>;

} // namespace planner
} // namespace AD_algorithm