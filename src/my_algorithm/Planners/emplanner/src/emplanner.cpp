#include "emplanner/emplanner.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include "general_modules/math_tool.h"

namespace AD_algorithm {
namespace planner {

using namespace general;

EMPlanner::EMPlanner() : PlannerBase("EMPlanner") {
    weights_=WeightCoefficients();

    // 初始化管理器，共享同一个 logger
    trajectory_manager_ = std::make_unique<TrajectoryManager>(logger_);
    
    // 初始化规划器，共享同一个 logger
    path_planner_ = std::make_unique<PathPlanner>(weights_, PathPlannerConfig(), logger_);
    
    speed_planner_ = std::make_unique<SpeedPlanner>(weights_, SpeedPlannerConfig(), logger_);
    
    log("INFO", "EMPlanner initialized");
}

void EMPlanner::set_log_enable(bool enable) {
    if (logger_) logger_->set_enable(enable);
}

void EMPlanner::setWeights(const WeightCoefficients& weights) {
    weights_ = weights;
    // TODO: 需要重新创建规划器以应用新的权重
}

void EMPlanner::setPathPlannerConfig(const PathPlannerConfig& config) {
    path_planner_->set_config(config);
}

void EMPlanner::setSpeedPlannerConfig(const SpeedPlannerConfig& config) {
    speed_planner_->set_config(config);
}

// 设置全局的参考线，来自于全局导航，基本上只用初始化一次，除非行程有变化
bool EMPlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
    // 创建 ReferenceLine 对象
    general::ReferenceLine ref_line(reference_line);
    // 使用 ReferenceLine 创建 FrenetFrame
    global_frenet_frame_ = std::make_shared<general::FrenetFrame>(ref_line);
    return true;
}

bool EMPlanner::isTrajectoryValid(
    const std::vector<general::TrajectoryPoint>& trajectory,
    std::string* reason,
    size_t min_points)
{
    if (!trajectory_manager_) {
        if (reason) {
            *reason = "trajectory_manager_ is null";
        }
        return false;
    }
    return trajectory_manager_->isTrajectoryValid(trajectory, reason, min_points);
}

std::vector<TrajectoryPoint> EMPlanner::plan(
    const std::shared_ptr<VehicleState>& ego_state,
    const std::vector<Obstacle>& obstacles,
    double reference_speed,
    double current_time) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    log("INFO", "Starting planning cycle at time: " + std::to_string(current_time));
    
    // 0. 设置当前时间
    trajectory_manager_->setCurrentTime(current_time);
    
    // 更新轨迹管理器中的自车状态副本
    trajectory_manager_->updateEgoState(ego_state);

    // 1. 障碍物处理
    log("INFO", "Step 1: Classifying obstacles...");
    std::vector<Obstacle> static_obstacles, dynamic_obstacles;
    trajectory_manager_->classifyObstacles(ego_state, obstacles, static_obstacles, dynamic_obstacles);
    
    if (static_obstacles.empty() && dynamic_obstacles.empty()) {
        log("WARN", "No relevant obstacles detected");
    }
    
    // 2. 计算规划起点
    log("INFO", "Step 2: Calculating planning start point...");
    TrajectoryPoint planning_start_point = trajectory_manager_->calculatePlanningStartPoint(ego_state,weights_.forward_predict_time);
    
    // 3. 静态障碍物投影到Frenet坐标系
    log("INFO", "Step 3: Projecting static obstacles to Frenet frame...");
    std::vector<std::vector<FrenetPoint>> static_frenet_obstacles;
    for (const auto& obs : static_obstacles) {
        log("INFO", "Raw Static Obstacle: id=" + std::to_string(obs.id) + " x=" + std::to_string(obs.x) + " y=" + std::to_string(obs.y));
        std::vector<FrenetPoint> frenet_corners = global_frenet_frame_->project_obstacle_to_frenet(obs);
        static_frenet_obstacles.push_back(frenet_corners);
        log("INFO", "Static obstacle projected with " + std::to_string(frenet_corners.size()) + " corners");
    }
    
    // 4. 路径规划
    log("INFO", "Step 4: Path planning...");
    FrenetPoint start_frenet = global_frenet_frame_->cartesian_to_frenet(planning_start_point);
    auto ego_frenet = global_frenet_frame_->cartesian_to_frenet(*ego_state);
    path_planner_->set_road_width(ego_state,start_frenet.s-ego_frenet.s);

    const size_t kMaxPathRetry = 3;
    std::vector<TrajectoryPoint> path_trajectory;
    std::string path_invalid_reason;
    bool path_valid = false;
    std::shared_ptr<FrenetFrame> path_frame_ptr;

    for (size_t attempt = 0; attempt < kMaxPathRetry; ++attempt) {
        path_trajectory = path_planner_->planPath(global_frenet_frame_, start_frenet, static_frenet_obstacles);

        if (path_trajectory.empty()) {
            log("WARN", "Path planning attempt " + std::to_string(attempt + 1) + " returned empty trajectory");
            continue;
        }
        auto path_frame = std::make_shared<FrenetFrame>(path_trajectory, false);
        if (!trajectory_manager_->isPathValid(*path_frame, &path_invalid_reason)) {
            std::string reason = path_invalid_reason.empty() ? "unknown" : path_invalid_reason;
            log("WARN", "Path planning attempt " + std::to_string(attempt + 1) + " produced invalid trajectory: " + reason);
            log("WARN","路径校验失败: %s", reason.c_str());
            continue;
        }

        path_valid = true;
        path_frame_ptr = path_frame;
        break;
    }

    if (!path_valid) {
        log("Path planning failed after retries, returning empty trajectory", "ERROR");
        return {};
    }

    log("Path planning completed, generated " + std::to_string(path_trajectory.size()) + " points");
    
    // 5. 速度规划
    // 动态障碍物投影到路径上
    log("INFO", "Step 5: Speed planning...");
    std::vector<std::vector<FrenetPoint>> dynamic_frenet_obstacles;
    for (const auto& obs : dynamic_obstacles) {
        std::vector<FrenetPoint> frenet_corners = path_frame_ptr->project_dynamic_obstacle_to_frenet(obs);
        dynamic_frenet_obstacles.push_back(frenet_corners);
    }

    const size_t kMaxSpeedRetry = 3;
    std::vector<STPoint> speed_profile;
    std::string speed_invalid_reason;
    bool speed_valid = false;

    for (size_t attempt = 0; attempt < kMaxSpeedRetry; ++attempt) {
        speed_profile = speed_planner_->planSpeed(*path_frame_ptr, planning_start_point, reference_speed, dynamic_frenet_obstacles);
        if (speed_profile.empty()) {
            log("WARN", "Speed planning attempt " + std::to_string(attempt + 1) + " returned empty profile");
            continue;
        }

        if (!trajectory_manager_->isSpeedProfileValid(speed_profile, &speed_invalid_reason)) {
            std::string reason = speed_invalid_reason.empty() ? "unknown" : speed_invalid_reason;
            log("WARN", "Speed planning attempt " + std::to_string(attempt + 1) + " produced invalid profile: " + reason);
            log("WARN", "速度校验失败: %s", reason.c_str());
            continue;
        }

        speed_valid = true;
        break;
    }

    if (!speed_valid) {
        log("Speed planning failed after retries, returning empty trajectory", "ERROR");
        return {};
    }

    log("Speed planning completed, generated " + std::to_string(speed_profile.size()) + " points");
    
    // 6. 生成最终轨迹
    log("INFO", "Step 6: Generating final trajectory...");
    auto final_trajectory = generateTrajectory(speed_profile, path_trajectory,planning_start_point.time_stamped);
    
    // std::cout<<"路径速度组合后起点信息(x,y,heading)："<<final_trajectory[0].x<<","<<final_trajectory[0].y<<","<<final_trajectory[0].heading<<std::endl;


    // 7. 轨迹拼接
    log("INFO", "Step 7: Stitching trajectory...");
    final_trajectory = trajectory_manager_->stitchTrajectory(final_trajectory);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    log("INFO", "Planning completed in " + std::to_string(duration.count()) + " ms");
    log("INFO", "Final trajectory has " + std::to_string(final_trajectory.size()) + " points");
    
    return final_trajectory;
}


// 这里的意思是，首先规划出路径，定死了，作为速度规划的参考线
// 假定车辆一定沿着这个路径走，但是不知道时间，即什么时候在这条路径上的什么位置
// 所以速度规划给出了每个时间点应该处于这个路径的位置，即s,s_dot,s_dot_dot
// 最后将两个东西合并起来，路径是没有时间的，但是速度规划结果st有时间
// 合并的时候以ST即速度规划为序列，在这个序列上加上位置(对应的路径信息)
std::vector<TrajectoryPoint> EMPlanner::generateTrajectory(
    const std::vector<STPoint>& speed_profile,
    const std::vector<TrajectoryPoint>& path_trajectory,
    double start_time) 
{
    // 1. 输入验证
    if (speed_profile.empty() || path_trajectory.empty()) {
        log("ERROR", "Cannot generate trajectory: empty input");
        return {};
    }
    if (path_trajectory.size() < 2) {
        log("ERROR", "Path trajectory too short for FrenetFrame");
        return {};
    }
    
    // 2. 创建Frenet坐标系
    FrenetFrame path_frenet(path_trajectory,false);
    const auto& ref_path = path_frenet.get_reference_path();
    if (ref_path.empty()) {
        log("ERROR", "Reference path is empty");
        return {};
    }
    log("INFO", "起点信息(x,y,heading，s)：", path_trajectory[0].x, ",", path_trajectory[0].y, ",", path_trajectory[0].heading, ",", ref_path[0].accumulated_s);
    log("INFO", "st起点信息(t,s,v)：", speed_profile[0].t, ",", speed_profile[0].s, ",", speed_profile[0].s_dot);
    double max_s = ref_path.back().accumulated_s;
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(speed_profile.size());
    
    // 3. 提取s值用于二分查找
    std::vector<double> s_values;
    s_values.reserve(ref_path.size());
    for (const auto& point : ref_path) {
        s_values.push_back(point.accumulated_s);
    }
    
    // 4. 为每个速度点生成轨迹点
    for (const auto& st_point : speed_profile) {
        TrajectoryPoint traj_point;
        traj_point.time_stamped = start_time + st_point.t;
        traj_point.v = st_point.s_dot;
        traj_point.a_tau = st_point.s_dot_dot;
        
        // 4.1 处理s值边界
        double cur_s = general::clamp(st_point.s, 0.0, max_s);
        
        // 4.2 二分查找找到s对应的索引区间
        auto it = std::lower_bound(s_values.begin(), s_values.end(), cur_s);
        size_t idx;
        
        if (it == s_values.end()) {
            // 超出范围，使用最后一个点
            idx = ref_path.size() - 1;
        } else if (it == s_values.begin()) {
            // 在第一个点之前
            idx = 0;
        } else {
            // 在两个点之间，it指向大于等于cur_s的点，我们取前一个
            idx = std::distance(s_values.begin(), it) - 1;
        }
        
        // 4.3 线性插值
        double alpha = 0.0;
        if (idx < ref_path.size() - 1 && 
            ref_path[idx + 1].accumulated_s > ref_path[idx].accumulated_s) 
        {
            double ds = ref_path[idx + 1].accumulated_s - ref_path[idx].accumulated_s;
            alpha = general::clamp((cur_s - ref_path[idx].accumulated_s) / ds, 0.0, 1.0);
        }

        // 4.4 插值得到轨迹点
        const auto& p0 = ref_path[idx];
        const auto& p1 = (idx < ref_path.size() - 1) ? ref_path[idx + 1] : p0;
        
        // 使用 path_trajectory 中的原始数据，避免 ReferenceLine 重新计算导致的数值噪声
        const auto& tp0 = path_trajectory[idx];
        const auto& tp1 = (idx < path_trajectory.size() - 1) ? path_trajectory[idx + 1] : tp0;

        traj_point.x = p0.x + alpha * (p1.x - p0.x);
        traj_point.y = p0.y + alpha * (p1.y - p0.y);
        traj_point.heading = normalize_angle(tp0.heading + alpha * (tp1.heading - tp0.heading));
        traj_point.kappa = tp0.kappa + alpha * (tp1.kappa - tp0.kappa);
        
        // 4.5 计算XY向加速度
        double a_n = traj_point.v * traj_point.v * traj_point.kappa;
        traj_point.ax = traj_point.a_tau * std::cos(traj_point.heading) - a_n * std::sin(traj_point.heading);
        traj_point.ay = traj_point.a_tau * std::sin(traj_point.heading) + a_n * std::cos(traj_point.heading);
        
        trajectory.push_back(std::move(traj_point));
    }
    
    log("INFO", "Generated " + std::to_string(trajectory.size()) + " trajectory points");
    return trajectory;
}


} // namespace planner
} // namespace AD_algorithm