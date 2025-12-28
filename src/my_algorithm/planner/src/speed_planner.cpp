#include "planner/speed_planner.h"
#include "planner/DP_solver.h"
#include "general_modules/Trajectory.h"
#include "general_modules/polynomial_curve.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace AD_algorithm::general;

namespace AD_algorithm {
namespace planner {


SpeedPlanner::SpeedPlanner(const WeightCoefficients& weights, const SpeedPlannerConfig& config,
                           std::shared_ptr<general::Logger> logger)
    : weights_(weights), config_(config), logger_(logger) {
    // 初始化
    _qp_solver = std::make_shared<OsqpEigen::Solver>();
    _qp_solver->settings()->setWarmStart(true);
}

std::vector<STPoint> SpeedPlanner::planSpeed(
        const FrenetFrame& ref_path_frame,
        const TrajectoryPoint& planning_start_point,
        double reference_speed,
        const std::vector<FrenetPoint>& dynamic_frenet_obstacles){
    // 障碍物和路径规划的轨迹都是基于全局坐标系的，都没有时间概念
    _dp_speed_profile.clear();
    _qp_speed_profile.clear();

    // 验证配置
    if (!config_.validate()) {
        log("ERROR", "Invalid configuration");
        return {};
    }
    log("INFO", "Step 5: Speed planning...");

    auto start_time = std::chrono::high_resolution_clock::now();
    
    log("INFO", "Starting speed planning with reference speed: " + std::to_string(reference_speed));
    log("INFO", "Dynamic obstacles: " + std::to_string(dynamic_frenet_obstacles.size()));
    
    // 1. 生成ST图
    log("INFO", "Generating ST graph...");
    auto st_graph = generateSTGraph(dynamic_frenet_obstacles);
    log("INFO", "Generated ST graph with " + std::to_string(st_graph.size()) + " obstacles");
    
    // 2. 计算规划起点（ST坐标系）
    STPoint start_point;
    start_point.t = 0.0;
    start_point.s = 0.0;
    start_point.s_dot = planning_start_point.v;
    
    // 计算切向加速度
    double heading = planning_start_point.heading;
    double cos_heading = std::cos(heading);
    double sin_heading = std::sin(heading);
    start_point.s_dot_dot = planning_start_point.ax * cos_heading + 
                           planning_start_point.ay * sin_heading;
    
    log("INFO", "Planning start point: t=" + std::to_string(start_point.t) + 
        ", s=" + std::to_string(start_point.s) + 
        ", s_dot=" + std::to_string(start_point.s_dot) + 
        ", s_dot_dot=" + std::to_string(start_point.s_dot_dot));
    
    // 3. 使用DP进行速度规划
    log("INFO", "Running DP speed planning...");
    STState start_state(start_point.t, start_point.s, 
                       start_point.s_dot, start_point.s_dot_dot);
    
    // 转换障碍物
    std::vector<STObstacle> st_obstacles = convertToSTObstacles(st_graph);
    // 获取轨迹的最大S值
    PathPoint last_PathPoint=ref_path_frame[ref_path_frame.size()-1];
    config_.s_max=last_PathPoint.accumulated_s;
    
    // 创建策略
    auto cost_function = std::make_shared<SpeedCostFunction>(
        weights_,config_, reference_speed, st_obstacles);
    auto constraint_checker = std::make_shared<SpeedConstraintChecker>(
        config_, st_obstacles);

    auto sampling_strategy = std::make_shared<SpeedSamplingStrategy>(config_);
    auto backtrack_strategy = std::make_shared<DefaultBacktrackStrategy<STState>>();
    
    // 预计算网格
    if(!_sample_grid) {
        _sample_grid = std::make_shared<std::vector<std::vector<STState>>>(sampling_strategy->generateSamplingGrid(start_state, 0));
    }
    
    // 配置DP规划器
    DpPlannerConfig dp_config;
    int num_layers = static_cast<int>(_sample_grid->size()+2);
    dp_config.max_layers = num_layers;
    dp_config.enable_pruning = true;
    dp_config.pruning_keep_num = 5;
    dp_config.debug_mode = false;  // 关闭调试模式以提高性能
    
    // 执行规划
    SpeedDpPlanner dp_planner(
        cost_function, constraint_checker,
        sampling_strategy, backtrack_strategy, dp_config
    );
    auto result = dp_planner.planWithGrid(*_sample_grid, start_state, STState());
    
    if (!result.success) {
        log("ERROR", "DP speed planning failed: " + result.message);
        return {};
    }
    
    log("DP speed planning succeeded, found profile with " + 
        std::to_string(result.optimal_path.size()) + " points");
    log("INFO", "Total cost: " + std::to_string(result.total_cost));
    log("INFO", "Computation time: " + std::to_string(result.computation_time_ms) + " ms");
    
    // 转换结果
    for (const auto& state : result.optimal_path) {
        STPoint point;
        point.t = state.t;
        point.s = state.s;
        point.s_dot = state.s_dot;
        point.s_dot_dot = state.s_dot_dot;
        _dp_speed_profile.push_back(point);
    }
    
    // 进行ST加密
    increaseSpeedProfile(_dp_speed_profile,config_.qp_dense_path_interval);

    // 生成凸空间
    std::vector<double> s_lb,s_ub,s_dot_lb,s_dot_ub;
    generate_convex_space(ref_path_frame,st_graph,s_lb,s_ub,s_dot_lb,s_dot_ub);
    
    // 验证凸空间约束
    if (s_lb.empty() || s_ub.empty() || s_dot_lb.empty() || s_dot_ub.empty()) {
        log("Convex space constraints are empty, skipping QP", "WARN");
    } else {
        log("INFO", "Convex space constraints generated successfully");
        log("INFO", "s_lb size: " + std::to_string(s_lb.size()) + ", s_ub size: " + std::to_string(s_ub.size()));
    }

    // QP二次规划
    log("INFO", "Running speed smoothing...");
    bool qp_success = QP_traj_optimal(s_lb,s_ub,s_dot_lb,s_dot_ub,reference_speed);
    
    if (!qp_success) {
        log("QP optimization failed, using DP result", "WARN");
        // QP失败时，使用DP结果，将deque转换为vector
        _qp_speed_profile.assign(_dp_speed_profile.begin(), _dp_speed_profile.end());
    }
    
    for (size_t i = 1; i < _qp_speed_profile.size(); ++i) {
        if (_qp_speed_profile[i].s < _qp_speed_profile[i-1].s) {
            _qp_speed_profile[i].s = _qp_speed_profile[i-1].s; // 强制拉平
        }
    }
    
    // 5. 加密速度剖面
    log("INFO", "Increasing speed profile density...");
    increaseSpeedProfile(_qp_speed_profile,config_.final_path_interval);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    log("INFO", "Speed planning completed in " + std::to_string(duration.count()) + " ms");
    log("INFO", "Generated " + std::to_string(_qp_speed_profile.size()) + " speed profile points");
    return _qp_speed_profile;
}


std::vector<std::unordered_map<std::string, double>> SpeedPlanner::generateSTGraph(
    const std::vector<FrenetPoint>& dynamic_obstacles,
    double delta_l) {
    // 时间t是以当前时刻为0点的相对时间,
    // 也就是从现在开始，障碍物几秒后移动到我的安全边界内,这个点就是t_in
    // t_in对应会有位置s_in,离开安全边界的时间是t_out,位置是s_out
    std::vector<std::unordered_map<std::string, double>> st_graph;
    for (const auto& obs : dynamic_obstacles) {
        // 1. 严格过滤静态障碍物 (Strict Static Obstacle Filter)
        // 如果障碍物是静止的 (s_dot 接近 0)，它不可能横向移动 (l_dot 应该是 0)。
        // 任何非零的 l_dot 都是感知噪声或Frenet投影误差。
        // 因此，如果它是静止的且当前在车道外，直接忽略，绝对不会切入。
        if (std::abs(obs.s_dot) < 0.5) {
            if (std::abs(obs.l) > delta_l) {
                continue; // 静止物体在界外 -> 永远在界外
            }
            // 如果在界内，则是真正的静止阻挡物，需要保留
        }

        // 2. 忽略横向速度过小的障碍物
        // 提高阈值到 0.5 m/s，过滤微小的抖动
        if (std::abs(obs.l_dot) <= 0.5) {
            continue; 
        }

        double t_in, t_out;
        if (std::abs(obs.l) > delta_l) { // 障碍物初始在界外
            if (obs.l * obs.l_dot > 0) { // 远离边界
                continue;
            } else { // 向边界移动
                // 长度除以速度，就是花费的时间，还要减掉安全边界的距离花费的时间
                t_in = std::abs(obs.l / obs.l_dot) - std::abs(delta_l / obs.l_dot);
                t_out = std::abs(obs.l / obs.l_dot) + std::abs(delta_l / obs.l_dot);
            }
        } else { // 障碍物初始在界内
            t_in = 0.0;
            if (obs.l_dot > 0) { // 向上移动
                t_out = (delta_l - obs.l) / obs.l_dot;
            } else { // 向下移动
                t_out = (-delta_l - obs.l) / obs.l_dot;
            }
        }
        
        // 3. 快速横向穿越过滤 (Ghost obstacles usually pass through quickly)
        // 正常变道需要 3-5秒。
        // 如果停留时间 < 1.5秒，说明横向速度极快 (> 1.3 m/s)，可能是投影误差
        // 或者是垂直穿越的物体，但在弯道场景下，这通常是误报
        if (t_out - t_in < 1.5) {
            continue;
        }

        if (t_out >= 8.0 || t_out <= 1.0) {
            continue; // 忽略太远或太近的障碍物
        }

        // Log the obstacle that passed all filters
        // log("INFO", "Obstacle passed filter: s=" + std::to_string(obs.s) + 
        //     ", l=" + std::to_string(obs.l) + 
        //     ", s_dot=" + std::to_string(obs.s_dot) + 
        //     ", l_dot=" + std::to_string(obs.l_dot) + 
        //     ", t_in=" + std::to_string(t_in) + 
        //     ", t_out=" + std::to_string(t_out));

        std::unordered_map<std::string, double> obstacle;
        obstacle["t_in"] = t_in;
        obstacle["t_out"] = t_out;
        obstacle["s_in"] = obs.s + obs.s_dot * t_in;
        obstacle["s_out"] = obs.s + obs.s_dot * t_out;
        st_graph.push_back(obstacle);
    }
    return st_graph;
}

void SpeedPlanner::generate_convex_space(
    FrenetFrame ref_path_frenet,
    const std::vector<std::unordered_map<std::string, double>>& dynamic_obs_st_graph,
    std::vector<double>& s_lb, std::vector<double>& s_ub,
    std::vector<double>& s_dot_lb, std::vector<double>& s_dot_ub)
{
    // 前提条件，需要确保轨迹时间和障碍物时间的参考系相同

    // 清空输出向量
    s_lb.clear();
    s_ub.clear();
    s_dot_lb.clear();
    s_dot_ub.clear();
    
    // 获取速度剖面（假设_dp_speed_profile是成员变量）
    const auto& speed_profile = _dp_speed_profile;
    if (speed_profile.empty()) {
        throw std::runtime_error("Speed profile is empty");
    }
    // 获取参考路径的最大s值
    const auto& ref_path_points = ref_path_frenet.get_reference_path();
    double max_path_s = ref_path_points.back().accumulated_s;
    // 1. 初始化起点约束
    s_lb.emplace_back(speed_profile.front().s);
    s_ub.emplace_back(speed_profile.front().s);
    s_dot_lb.emplace_back(speed_profile.front().s_dot);
    s_dot_ub.emplace_back(speed_profile.front().s_dot);
    
    // 2. 确定速度上下界（基于横向加速度限制）
    for (size_t i = 1; i < speed_profile.size(); i++) {
        // 获取当前点的s值
        double cur_s = speed_profile[i].s;
        // 通过FrenetFrame获取路径点信息
        PathPoint path_point;
        if (cur_s >= max_path_s) {
            // 如果超出路径终点，使用最后一个点
            path_point = ref_path_points.back();
        } else {
            path_point=ref_path_frenet[i];
        }
        // 计算曲率
        double cur_kappa = path_point.kappa;
        // 由曲率计算速度上下界
        // 避免除零和负值
        double kappa_abs = std::abs(cur_kappa);
        double speed_limit = 0.0;
        
        if (kappa_abs > 1e-6) {
            speed_limit = std::sqrt(std::abs(config_.max_lateral_acc / cur_kappa));
            // 移除安全系数，与plus_version保持一致
            // speed_limit *= 0.9; 
            
            // 增加最小过弯速度保护
            // 即使曲率很大，也允许以最低 2.0 m/s 的速度通过，防止完全刹停
            // 物理上这可能稍微超过侧向加速度限制，但为了连贯性是必要的
            speed_limit = std::max(speed_limit, 3.0);
        } else {
            // 直线段，速度上限可以很大
            speed_limit = 30.0; // 设置一个很大的值，比如30m/s
        }
           
        s_dot_ub.emplace_back(speed_limit);
        s_dot_lb.emplace_back(0.0); // 最小速度为0

        // 移除基于加速度限制的动态约束，让QP求解器自己处理
        // if (i > 0) {
        //     double dt = speed_profile[i].t - speed_profile[i-1].t;
        //     if (dt > 1e-6) {
        //         // 基于加速度限制计算最大可能的速度变化
        //         double max_speed_increase = s_dot_ub[i-1] + config_.max_acceleration * dt;
        //         double max_speed_decrease = s_dot_lb[i-1] + config_.max_deceleration * dt;
                
        //         s_dot_ub[i] = std::min(s_dot_ub[i], max_speed_increase);
        //         s_dot_lb[i] = std::max(s_dot_lb[i], max_speed_decrease);
        //     }
        // }

        // 确保速度上下界合理
        if (s_dot_lb[i] > s_dot_ub[i]) {
            // 如果计算出的下界大于上界（通常是因为曲率限制太严格），
            // 我们优先保证上界（安全），但给一个最小的可行速度，防止卡死
            // s_dot_lb[i] = 0.0;
            // s_dot_ub[i] = config_.max_acceleration * 2.0; 
            
            // 修正策略：如果曲率限制导致速度过低，我们稍微放宽一点，或者取两者的平均值
            // 但绝对不能让下界大于上界
            double mid = (s_dot_lb[i] + s_dot_ub[i]) / 2.0;
            s_dot_lb[i] = 0.0;
            s_dot_ub[i] = std::max(mid, 1.0); // 至少给1m/s的速度
        }
    }
    
    // 3. 确定位置上下界（基于动态障碍物）
    // 首先初始化基本约束
    for (size_t i = 1; i < speed_profile.size(); i++) {
        s_ub.emplace_back(max_path_s);  // 不能超过路径终点
        s_lb.emplace_back(0.0);         // 不能倒车
    }
    
    // 4. 处理每个障碍物，为每个轨迹点计算最近的上下障碍物边界
    for (size_t i = 1; i < speed_profile.size(); i++) {  // 从第二个点开始，第一个点是起点
        double t_i = speed_profile[i].t;
        double s_i = speed_profile[i].s;
        
        // 初始化当前点的障碍物边界
        double nearest_upper_bound = std::numeric_limits<double>::max();    // 上边界（最小值）
        double nearest_lower_bound = std::numeric_limits<double>::lowest(); // 下边界（最大值）
        
        // 遍历所有障碍物，找到影响当前时间点的障碍物
        for (const auto& cur_obs : dynamic_obs_st_graph) {
            // 提取障碍物信息
            double t_in = cur_obs.at("t_in");
            double t_out = cur_obs.at("t_out");
            double s_in = cur_obs.at("s_in");
            double s_out = cur_obs.at("s_out");
            
            // 检查当前时间点是否在障碍物时间范围内
            if (t_i < t_in || t_i > t_out) {
                continue; // 当前时间点不受此障碍物影响
            }
            
            // 获取障碍物参数
            double obs_length = cur_obs.count("length") > 0 ? cur_obs.at("length") : 3.0;
            double safety_margin_s = cur_obs.count("safety_margin_s") > 0 ? cur_obs.at("safety_margin_s") :0.5;
            
            // 计算障碍物在当前时间的中心位置
            double k = (s_out - s_in) / (t_out - t_in); // 障碍物速度
            double obs_s_center = s_in + k * (t_i - t_in);
            
            // 计算障碍物的安全边界
            double half_obs_length_with_margin = obs_length / 2.0 + safety_margin_s;
            double obs_lower_bound = obs_s_center - half_obs_length_with_margin; // 障碍物前边界
            double obs_upper_bound = obs_s_center + half_obs_length_with_margin; // 障碍物后边界
            
            // 根据轨迹点与障碍物的相对位置，更新最近的边界
            if (s_i > obs_upper_bound) {
                // 轨迹点在障碍物上方，障碍物在下方
                // 需要确保不低于障碍物的上边界
                nearest_lower_bound = std::max(nearest_lower_bound, obs_upper_bound);
            } else if (s_i < obs_lower_bound) {
                // 轨迹点在障碍物下方，障碍物在上方
                // 需要确保不高于障碍物的下边界
                nearest_upper_bound = std::min(nearest_upper_bound, obs_lower_bound);
            } else {
                // 轨迹点在障碍物安全区域内
                // 这种情况通常是因为DP的离散误差或障碍物预测的不确定性导致的
                // 或者是障碍物刚好擦过路径边缘
                
                // 计算重叠量
                double overlap = std::min(s_i - obs_lower_bound, obs_upper_bound - s_i);
                
                // 如果重叠量很小（例如小于0.5米），我们认为这是数值误差，选择忽略或轻微调整
                if (overlap < 0.5) {
                    // 忽略此障碍物约束，或者给予一个宽松的边界
                    // 这里选择忽略，让QP去平滑处理，因为DP已经尽力避让了
                    continue; 
                } else {
                    // 如果重叠量较大，说明真的有碰撞风险
                    // 但为了防止QP无解或急刹车，我们不强制推开，而是限制在当前位置附近
                    // 这样QP会尝试减速而不是报错
                    
                    // 策略：根据相对位置决定是加速通过还是减速避让
                    // 这里简单处理：如果是追尾风险（障碍物在前方），则限制上界
                    double dist_to_lower = std::abs(s_i - obs_lower_bound);
                    double dist_to_upper = std::abs(s_i - obs_upper_bound);
                    
                    if (dist_to_lower < dist_to_upper) {
                        // 离下边界更近（障碍物在前方），限制上界
                        // 但保证上界不小于当前s_i，避免无解
                        nearest_upper_bound = std::min(nearest_upper_bound, std::max(s_i, obs_lower_bound));
                    } else {
                        // 离上边界更近（障碍物在后方），限制下界
                        nearest_lower_bound = std::max(nearest_lower_bound, std::min(s_i, obs_upper_bound));
                    }
                }
            }
        }
        
        // 如果没有找到障碍物约束，使用默认值
        if (nearest_upper_bound == std::numeric_limits<double>::max()) {
            nearest_upper_bound = max_path_s; // 默认上边界为路径终点
        }
        if (nearest_lower_bound == std::numeric_limits<double>::lowest()) {
            nearest_lower_bound = 0.0; // 默认下边界为起点
        }
        
        // 更新该轨迹点的约束
        // 注意：s_lb是下边界的最小值，s_ub是上边界的最大值
        // 所以我们需要取所有约束中最严格的
        s_lb[i] = std::max(s_lb[i], nearest_lower_bound);
        s_ub[i] = std::min(s_ub[i], nearest_upper_bound);
        
        // 确保下边界不大于上边界（约束有效）
        if (s_lb[i] > s_ub[i]) {
            // 如果发生冲突，调整边界使它们相等（取中点）
            double middle = (s_lb[i] + s_ub[i]) / 2.0;
            s_lb[i] = middle;
            s_ub[i] = middle;
            
            // 记录警告
            log("WARN", "轨迹点", i, "(t=", t_i, ", s=", s_i, 
                    ") 的约束冲突，下边界=", s_lb[i], " > 上边界=", s_ub[i],
                    "，已调整为", middle);
        }
    }

}


void SpeedPlanner::increaseSpeedProfile(std::vector<general::STPoint>& DP_or_QP,double interval) {
    std::vector<general::STPoint> dense_path;
    
    for(size_t i=0;i<DP_or_QP.size()-1;++i){
        const auto& p0 = DP_or_QP[i];
        const auto& p1 = DP_or_QP[i+1];
        
        double ds = std::abs(p1.s - p0.s);
        double dt = p1.t - p0.t;
        
        // 动态计算采样点数：同时考虑距离间隔和时间分辨率
        // 1. 距离约束: ds / interval
        // 2. 时间约束: dt / 0.05 (至少20Hz)
        size_t num_s = (size_t)(ds / interval);
        size_t num_t = (size_t)(dt / 0.05); 
        size_t num = std::max(num_s, num_t);
        if (num < 2) num = 2; // 保证至少包含起点

        PolynomialCurve poly;
        poly.curve_fitting(
            p0.t, p0.s, p0.s_dot, p0.s_dot_dot,
            p1.t, p1.s, p1.s_dot, p1.s_dot_dot
        );
        auto linspace=Eigen::VectorXd::LinSpaced(
            num,p0.t, p1.t
        );
        for(int j=0;j<linspace.size()-1;++j){
            double t = linspace[j];
            if(t>p1.t) t=p1.t;//防止数值误差超过终点
            general::STPoint fp;
            fp.t = t;
            fp.s = poly.value_evaluation(t,0);
            fp.s_dot = poly.value_evaluation(t,1);
            fp.s_dot_dot = poly.value_evaluation(t,2);

            // Clamp acceleration
            fp.s_dot_dot = general::clamp(fp.s_dot_dot, config_.max_deceleration, config_.max_acceleration);
            
            // Ensure non-negative speed
            if (fp.s_dot < 0.0) {
                fp.s_dot = 0.0;
            }

            dense_path.push_back(fp);
        }
    }
    dense_path.push_back(DP_or_QP[DP_or_QP.size()-1]);
    DP_or_QP=dense_path; 
}


bool SpeedPlanner::QP_traj_optimal(const std::vector<double>& s_lb, const std::vector<double>& s_ub, 
     const std::vector<double>& s_dot_lb, const std::vector<double>& s_dot_ub,
     const double reference_speed)
{
    // 0. 初始化与验证
    size_t point_num = _dp_speed_profile.size();
    if (point_num < 2) {
        log("ERROR", "Not enough points for QP optimization");
        return false;
    }

    // 验证输入数据维度
    if (s_lb.size() != point_num || s_ub.size() != point_num ||
        s_dot_lb.size() != point_num || s_dot_ub.size() != point_num) {
        log("ERROR", "Constraint vector size mismatch");
        return false;
    }

    // 检查约束有效性
    for (size_t i = 0; i < point_num; i++) {
        if (s_lb[i] > s_ub[i]) {
            log("ERROR", "Invalid position constraint at index " + std::to_string(i));
            return false;
        }
        if (s_dot_lb[i] > s_dot_ub[i]) {
            log("ERROR", "Invalid velocity constraint at index " + std::to_string(i));
            return false;
        }
    }

    log("INFO", "Starting QP optimization for speed profile with " + std::to_string(point_num) + " points");

    // ==========================================
    // 1. 建立目标函数 (Cost Function)
    // J = 0.5 * x'Hx + f'x
    // 变量 x = [s_0, v_0, a_0, s_1, v_1, a_1, ...]
    // ==========================================

    // 1.1 H 矩阵 (Hessian Matrix)
    Eigen::SparseMatrix<double> H_total(3*point_num, 3*point_num);
    
    // 位置跟踪代价: 移除对DP路径的强行跟踪，以提高平滑度
    // for (size_t i = 0; i < point_num; i++) {
    //     H_total.coeffRef(3*i + 0, 3*i + 0) += 2 * weights_.speed_qp_w_ref_s;
    // }
    
    // 速度跟踪代价: 只跟踪外部参考速度，忽略DP速度的噪点
    // 合并两个权重以确保有足够的权重去跟踪参考速度
    double total_speed_weight = weights_.speed_qp_w_ref_speed + weights_.speed_qp_w_target_speed;
    for (size_t i = 0; i < point_num; i++) {
        H_total.coeffRef(3*i + 1, 3*i + 1) += 2 * total_speed_weight;
    }
    
    // 加速度代价: 保证平滑性，惩罚过大的加速度
    for (size_t i = 0; i < point_num; i++) {
        H_total.coeffRef(3*i + 2, 3*i + 2) += 2 * weights_.speed_qp_w_a;
    }
    
    // 加加速度 (Jerk) 代价: 保证加速度变化的平滑性
    // Jerk ~ (a_{i+1} - a_i) / dt
    for (size_t i = 0; i < point_num - 1; i++) {
        double w = 2 * weights_.speed_qp_w_jerk;
        H_total.coeffRef(3*i + 2, 3*i + 2) += w;
        H_total.coeffRef(3*i + 2, 3*(i+1) + 2) -= w;
        H_total.coeffRef(3*(i+1) + 2, 3*i + 2) -= w;
        H_total.coeffRef(3*(i+1) + 2, 3*(i+1) + 2) += w;
    }
    
    // 1.2 f 向量 (Gradient Vector)
    Eigen::VectorXd f = Eigen::VectorXd::Zero(3*point_num);
    // double total_speed_weight = weights_.speed_qp_w_ref_speed + weights_.speed_qp_w_target_speed; // Already calculated above
    for (size_t i = 0; i < point_num; i++) {
        // 目标是最小化 (s - s_ref)^2 -> 展开为 s^2 - 2*s*s_ref + s_ref^2
        // 一次项系数为 -2 * s_ref
        // 移除位置跟踪项
        // f[3*i + 0] = -2 * weights_.speed_qp_w_ref_s * _dp_speed_profile[i].s;
        
        // 速度跟踪项: 只锚定全局参考速度
        f[3*i + 1] = -2 * total_speed_weight * reference_speed;
    }

    // ==========================================
    // 2. 建立约束 (Constraints)
    // l <= Ax <= u
    // ==========================================
    
    double delta_t = _dp_speed_profile[1].t - _dp_speed_profile[0].t;
    if (delta_t <= 0) return false;

    // 2.1 连续性约束 (Continuity Constraints)
    // 运动学积分: s_{i+1} = s_i + v_i*dt + 0.5*a_i*dt^2
    //            v_{i+1} = v_i + a_i*dt
    Eigen::SparseMatrix<double> A_continuity(2*(point_num - 1), 3*point_num);
    
    for (size_t i = 0; i < point_num - 1; i++) {
        int r = 2*i;
        int c = 3*i;
        
        // 位置积分
        A_continuity.insert(r, c + 0) = 1.0;          // s_i
        A_continuity.insert(r, c + 1) = delta_t;      // v_i * dt
        A_continuity.insert(r, c + 2) = 0.5 * delta_t * delta_t; // 0.5*a_i*dt^2
        A_continuity.insert(r, c + 3) = -1.0;         // -s_{i+1}
        
        // 速度积分
        A_continuity.insert(r + 1, c + 1) = 1.0;      // v_i
        A_continuity.insert(r + 1, c + 2) = delta_t;  // a_i * dt
        A_continuity.insert(r + 1, c + 4) = -1.0;     // -v_{i+1}
    }
    // 等式约束，上下界均为0
    Eigen::VectorXd continuity_bound = Eigen::VectorXd::Zero(2*(point_num - 1));

    // 2.2 起点状态约束 (Start State Constraints)
    Eigen::SparseMatrix<double> A_start(3, 3*point_num);
    A_start.insert(0, 0) = 1.0;
    A_start.insert(1, 1) = 1.0;
    A_start.insert(2, 2) = 1.0;
    
    Eigen::VectorXd start_val(3);
    start_val << _dp_speed_profile[0].s, _dp_speed_profile[0].s_dot, _dp_speed_profile[0].s_dot_dot;

    // 2.3 速度非负约束 (Non-negative Speed)
    Eigen::SparseMatrix<double> A_speed_limit(point_num, 3*point_num);
    Eigen::VectorXd speed_lower(point_num), speed_upper(point_num);
    for (size_t i = 0; i < point_num; i++) {
        A_speed_limit.insert(i, 3*i + 1) = 1.0;
        speed_lower[i] = 0.0;
        speed_upper[i] = std::numeric_limits<double>::infinity();
    }

    // 2.4 凸空间约束 (Convex Space Constraints)
    // 包含位置边界 [s_lb, s_ub] 和 速度边界 [s_dot_lb, s_dot_ub]
    Eigen::SparseMatrix<double> A_convex(2*point_num, 3*point_num);
    Eigen::VectorXd convex_lower(2*point_num), convex_upper(2*point_num);
    
    for (size_t i = 0; i < point_num; i++) {
        // 位置约束
        A_convex.insert(2*i, 3*i + 0) = 1.0;
        convex_lower[2*i] = s_lb[i];
        convex_upper[2*i] = s_ub[i];
        
        // 速度约束
        A_convex.insert(2*i + 1, 3*i + 1) = 1.0;
        convex_lower[2*i + 1] = s_dot_lb[i];
        convex_upper[2*i + 1] = s_dot_ub[i];
    }

    // 2.5 组装所有约束
    // 使用转置拼接技巧
    int total_rows = A_continuity.rows() + A_start.rows() + A_speed_limit.rows() + A_convex.rows();
    Eigen::SparseMatrix<double> A_total_T(3*point_num, total_rows);
    
    int current_col = 0;
    A_total_T.middleCols(current_col, A_continuity.rows()) = A_continuity.transpose(); current_col += A_continuity.rows();
    A_total_T.middleCols(current_col, A_start.rows()) = A_start.transpose(); current_col += A_start.rows();
    A_total_T.middleCols(current_col, A_speed_limit.rows()) = A_speed_limit.transpose(); current_col += A_speed_limit.rows();
    A_total_T.middleCols(current_col, A_convex.rows()) = A_convex.transpose();
    
    Eigen::SparseMatrix<double> A_total = A_total_T.transpose();
    
    Eigen::VectorXd lower_total(total_rows), upper_total(total_rows);
    current_col = 0; // 重用作为行索引
    lower_total.segment(current_col, continuity_bound.size()) = continuity_bound;
    upper_total.segment(current_col, continuity_bound.size()) = continuity_bound;
    current_col += continuity_bound.size();
    
    lower_total.segment(current_col, start_val.size()) = start_val;
    upper_total.segment(current_col, start_val.size()) = start_val;
    current_col += start_val.size();
    
    lower_total.segment(current_col, speed_lower.size()) = speed_lower;
    upper_total.segment(current_col, speed_upper.size()) = speed_upper;
    current_col += speed_lower.size();
    
    lower_total.segment(current_col, convex_lower.size()) = convex_lower;
    upper_total.segment(current_col, convex_upper.size()) = convex_upper;

    // ==========================================
    // 3. 求解 (Solve)
    // ==========================================
    try {
        _qp_solver->clearSolver();
        _qp_solver->data()->clearHessianMatrix();
        _qp_solver->data()->clearLinearConstraintsMatrix();
        _qp_solver->settings()->setVerbosity(false);
        _qp_solver->settings()->setMaxIteration(4000);

        _qp_solver->data()->setNumberOfVariables(3*point_num);
        _qp_solver->data()->setNumberOfConstraints(total_rows);
        
        if (!_qp_solver->data()->setHessianMatrix(H_total)) return false;
        if (!_qp_solver->data()->setGradient(f)) return false;
        if (!_qp_solver->data()->setLinearConstraintsMatrix(A_total)) return false;
        if (!_qp_solver->data()->setLowerBound(lower_total)) return false;
        if (!_qp_solver->data()->setUpperBound(upper_total)) return false;
        
        if (!_qp_solver->initSolver()) return false;
        
        if (_qp_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            log("ERROR", "QP solver failed");
            return false;
        }
        
        // 提取结果
        auto solution = _qp_solver->getSolution();
        _qp_speed_profile.clear();
        for (size_t i = 0; i < point_num; i++) {
            STPoint cur_st_point;
            cur_st_point.t = _dp_speed_profile[i].t;
            cur_st_point.s = solution[3*i + 0];
            cur_st_point.s_dot = solution[3*i + 1];
            cur_st_point.s_dot_dot = solution[3*i + 2];
            _qp_speed_profile.emplace_back(cur_st_point);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        log("ERROR", "QP optimization exception: " + std::string(e.what()));
        return false;
    }
}



} // namespace planner
} // namespace AD_algorithm