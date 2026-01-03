#include "general_modules/Trajectory.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>

namespace AD_algorithm {
namespace general {

TrajectoryManager::TrajectoryManager(std::shared_ptr<Logger> logger) : logger_(logger) {
}

TrajectoryPoint TrajectoryManager::calculatePlanningStartPoint(
    const std::shared_ptr<VehicleState>& ego_state,
    double delta_T) {
    
    TrajectoryPoint planning_start_point;
    
    // 1. 首次运行处理或如果时间超过了上次规划的最大时间
    if (previous_trajectory_.empty() 
    || is_first_run_ 
    || previous_trajectory_.size() < 2
    ||current_time_>previous_trajectory_[previous_trajectory_.size()-1].time_stamped
    ) {
        previous_trajectory_.clear();
        log("First run or insufficient previous trajectory, using current ego state as planning start point");
        is_first_run_ = false;
        planning_start_point.x = ego_state->x;
        planning_start_point.y = ego_state->y;
        planning_start_point.v = ego_state->v;
        planning_start_point.heading = ego_state->heading;
        planning_start_point.ax = ego_state->ax;
        planning_start_point.ay = ego_state->ay;
        planning_start_point.kappa = 0.0;
        planning_start_point.time_stamped = current_time_;
        stitch_trajectory_.clear();
        return planning_start_point;
    } 

    // 2. 非首次运行，检查轨迹跟踪情况
    // 查找当前时间在上一周期轨迹中的匹配点
    FrenetFrame pre_ref(previous_trajectory_);
    auto trj_msg=pre_ref.get_matched_trj_point(current_time_);
    
    auto target_point = trj_msg.first;
    Eigen::Vector2d tau_target(std::cos(target_point.heading), std::sin(target_point.heading));
    Eigen::Vector2d nor_target(-std::sin(target_point.heading), std::cos(target_point.heading));
    Eigen::Vector2d host_to_target(target_point.x - ego_state->x, target_point.y - ego_state->y);
    
    double longitudinal_error = std::abs(host_to_target.dot(tau_target));
    double lateral_error = std::abs(host_to_target.dot(nor_target));
    
    log("INFO", "Tracking errors - lateral: " + std::to_string(lateral_error) + 
        ", longitudinal: " + std::to_string(longitudinal_error));
    
    // 3. 根据误差决定规划起点策略
    if (lateral_error < 0.5 && longitudinal_error < 1.5) {
        // 3.1 跟踪良好，使用上一周期轨迹进行拼接
        log("Good tracking, using previous trajectory");
        double start_time = current_time_ + delta_T;
        auto start_point=pre_ref.get_matched_trj_point(start_time);
        planning_start_point =start_point.first;
        
        // 轨迹裁剪,保留起点之前的点用于拼接
        stitch_trajectory_.clear();
        int stitch_num = std::max(0,_pre_pnt_num);
        int i = 0;
        for (; pre_ref[i].accumulated_s<= start_point.second; i++) {
            stitch_trajectory_.emplace_back(pre_ref.traj_point(i));
        }
        // 限制拼接点数量
        if(i+1>stitch_num){
            stitch_trajectory_.erase(stitch_trajectory_.begin(), stitch_trajectory_.begin()+(i-stitch_num));
        }
        log("INFO", "Stitching " + std::to_string(stitch_trajectory_.size()) + " points from previous trajectory");
    } else {
        // 3.2 误差过大，使用运动学外推
        previous_trajectory_.clear();
        log("Large tracking error, using kinematic extrapolation");
        planning_start_point.ax = ego_state->ax;
        planning_start_point.ay = ego_state->ay;
        
        double ego_vx = ego_state->v * std::cos(ego_state->heading);
        double ego_vy = ego_state->v * std::sin(ego_state->heading);
        
        // 如果误差大，直接从当前点开始使用运动学模拟规划轨迹
        stitch_trajectory_.clear();
        for(double time=0;time<=delta_T;time+=0.01){
            TrajectoryPoint p;
            double t_offset = time; // 相对时间
            
            p.x = ego_state->x + ego_vx * t_offset + 
                                    0.5 * ego_state->ax * t_offset * t_offset;
            p.y = ego_state->y + ego_vy * t_offset + 
                                    0.5 * ego_state->ay * t_offset * t_offset;
            double planning_vx = ego_vx + ego_state->ax * t_offset;
            double planning_vy = ego_vy + ego_state->ay * t_offset;
            p.v = std::hypot(planning_vx, planning_vy);
            p.heading = std::atan2(planning_vy, planning_vx);
            p.time_stamped = current_time_ + t_offset;
            p.kappa = 0.0;
            stitch_trajectory_.push_back(p);
            
            // 更新规划起点为最后一个点
            planning_start_point = p;
        }
    }
    
    // 4. 移除最后一个点作为规划起点（避免重复）
    if(!stitch_trajectory_.empty()){
        stitch_trajectory_.pop_back();
    }
    
    log("INFO", "Planning start point: x=" + std::to_string(planning_start_point.x) + 
        ", y=" + std::to_string(planning_start_point.y) + 
        ", v=" + std::to_string(planning_start_point.v));
    
    return planning_start_point;
}

void TrajectoryManager::classifyObstacles(
    const std::shared_ptr<VehicleState>& ego_state,
    const std::vector<Obstacle>& obstacles,
    std::vector<Obstacle>& static_obstacles,
    std::vector<Obstacle>& dynamic_obstacles) {
    
    static_obstacles.clear();
    dynamic_obstacles.clear();
    
    if (obstacles.empty()) {
        log("INFO", "No obstacles detected");
        return;
    }
    
    log("INFO", "Processing " + std::to_string(obstacles.size()) + " obstacles");
    
    for (const auto& obs : obstacles) {
        if (obs.id == ego_state->id) {
            continue; // 排除自车
        }
        
        double v_obs = obs.getSpeed();
        
        Eigen::Vector2d host_to_obs(
            obs.x - ego_state->x,
            obs.y - ego_state->y
        );
        
        Eigen::Vector2d tau_host(std::cos(ego_state->heading), std::sin(ego_state->heading));
        Eigen::Vector2d nor_host(-std::sin(ego_state->heading), std::cos(ego_state->heading));
        
        double longitudinal_d = host_to_obs.dot(tau_host);
        double lateral_d = host_to_obs.dot(nor_host);
        
        // 修正：提高静态障碍物判定阈值，防止传感器噪声导致误判
        // 之前的 1e-2 (0.01m/s) 太敏感，墙壁稍微有点读数漂移就会变成动态障碍物
        if (v_obs < 0.5) { // 静态障碍物 (阈值提高到 0.5 m/s)
            if (longitudinal_d >= -10.0 && longitudinal_d <= 60.0 &&
                lateral_d >= -10.0 && lateral_d <= 10.0) {
                static_obstacles.push_back(obs);
            }
        } else { // 动态障碍物
            // 额外检查：如果是低速动态障碍物 (0.5 - 2.0 m/s)，收紧横向范围
            // 防止远处的噪点被纳入
            double lat_limit = (v_obs < 2.0) ? 10.0 : 20.0;
            
            if (longitudinal_d >= -10.0 && longitudinal_d <= 60.0 &&
                lateral_d >= -lat_limit && lateral_d <= lat_limit) {
                dynamic_obstacles.push_back(obs);
            }
        }
    }
    
    log("INFO", "Static obstacles: " + std::to_string(static_obstacles.size()) + 
        ", Dynamic obstacles: " + std::to_string(dynamic_obstacles.size()));
}

std::vector<TrajectoryPoint> TrajectoryManager::stitchTrajectory(
    const std::vector<TrajectoryPoint>& current_trajectory,
    int /*stitch_points*/) {
    
    // int count=0;
    // if (enable_logging_ && current_trajectory.size() > 1) {
    //     std::cout<<"本周期规划的轨迹信息："<<std::endl;
    //     FrenetFrame ref_trj0(current_trajectory);
    //     count=0;
    //     for(auto& t:current_trajectory){
    //         if(count<100){
    //             std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
    //             <<"，位置(x,y)："<<t.x<<","<<t.y
    //             <<"，S:"<<ref_trj0[count].accumulated_s
    //             <<std::endl;
    //         }else{
    //             if(count%10==0){
    //                 std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
    //                 <<"，位置(x,y)："<<t.x<<","<<t.y
    //                 <<"，S:"<<ref_trj0[count].accumulated_s
    //                 <<std::endl;
    //             }
    //         }
    //         count++;
    //     }      
    // }
        
    std::vector<TrajectoryPoint> final_trajectory;
    
    // 添加拼接轨迹（如果存在）
    if (!stitch_trajectory_.empty()) {
        log("INFO", "添加 " + std::to_string(stitch_trajectory_.size()) + " 个规划起点之前的轨迹");
        final_trajectory.insert(final_trajectory.end(), 
                                stitch_trajectory_.begin(), 
                                stitch_trajectory_.end());                        
    }

    // 添加当前规划轨迹
    log("INFO", "添加 " + std::to_string(current_trajectory.size()) + " 个本周期规划的轨迹");
    
    // 检查拼接处是否存在重复点 (基于时间戳)
    if (!final_trajectory.empty() && !current_trajectory.empty()) {
        double last_time = final_trajectory.back().time_stamped;
        double first_time = current_trajectory.front().time_stamped;
        
        // 如果时间戳非常接近，跳过当前轨迹的第一个点，避免重复或极小间距导致的数值不稳定
        if (std::abs(first_time - last_time) < 1e-4) {
            final_trajectory.insert(final_trajectory.end(), 
                                    current_trajectory.begin() + 1, 
                                    current_trajectory.end());
        } else {
            final_trajectory.insert(final_trajectory.end(), 
                                    current_trajectory.begin(), 
                                    current_trajectory.end());
        }
    } else {
        final_trajectory.insert(final_trajectory.end(), 
                                current_trajectory.begin(), 
                                current_trajectory.end());
    }

    log("INFO", "拼接后的的轨迹有 " + std::to_string(final_trajectory.size()) + "个点");
    
    // if (enable_logging_ && final_trajectory.size() > 1) {
    //     std::cout<<"拼接后的的轨迹信息："<<std::endl;
    //     FrenetFrame ref_trj1(final_trajectory);
    //     count=0;
    //     for(auto& t:final_trajectory){
    //         if(count<100){
    //             std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
    //             <<"，位置(x,y)："<<t.x<<","<<t.y
    //             <<"，S:"<<ref_trj1[count].accumulated_s
    //             <<std::endl;
    //         }else{
    //             if(count%10==0){
    //                 std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
    //                 <<"，位置(x,y)："<<t.x<<","<<t.y
    //                 <<"，S:"<<ref_trj1[count].accumulated_s
    //                 <<std::endl;
    //             }
    //         }
    //         count++;
    //     }   
    // }

    return final_trajectory;
}

void TrajectoryManager::updateEgoState(const std::shared_ptr<VehicleState>& ego_state)
{
    if (ego_state) {
        latest_ego_state_ = *ego_state;
        has_latest_ego_state_ = true;
    }
}

bool TrajectoryManager::isPathValid(
    const std::vector<TrajectoryPoint>& path_trajectory,
    std::string* reason,
    size_t min_points) const
{
    auto fail = [&](const std::string& msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    if (path_trajectory.size() < min_points) {
        return fail("path points < min_points (" + std::to_string(path_trajectory.size()) + " < " + std::to_string(min_points) + ")");
    }

    auto is_finite = [](double v) { return std::isfinite(v); };
    constexpr double kMaxStepDist = 30.0;
    constexpr double kMaxHeadingJump = 1.0;
    constexpr double kMaxKappa = 5.0;

    for (size_t i = 0; i < path_trajectory.size(); ++i) {
        const auto& p = path_trajectory[i];
        if (!is_finite(p.x) || !is_finite(p.y) || !is_finite(p.heading) || !is_finite(p.kappa)) {
            return fail("non-finite path value at index " + std::to_string(i));
        }
        if (std::abs(p.kappa) > kMaxKappa) {
            return fail("abs(kappa) too large at index " + std::to_string(i));
        }
    }

    for (size_t i = 1; i < path_trajectory.size(); ++i) {
        const auto& p0 = path_trajectory[i - 1];
        const auto& p1 = path_trajectory[i];

        const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (dist > kMaxStepDist) {
            return fail("step distance too large at index " + std::to_string(i));
        }

        double heading_diff = p1.heading - p0.heading;
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
        if (std::abs(heading_diff) > kMaxHeadingJump) {
            return fail("heading jump too large at index " + std::to_string(i));
        }
    }

    if (reason) {
        reason->clear();
    }
    return true;
}

bool TrajectoryManager::isPathValid(
    const FrenetFrame& path_trajectory,
    std::string* reason,
    size_t min_points) const
{
    auto fail = [&](const std::string& msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    if (path_trajectory.size() < min_points) {
        return fail("path points < min_points (" + std::to_string(path_trajectory.size()) + " < " + std::to_string(min_points) + ")");
    }

    auto is_finite = [](double v) { return std::isfinite(v); };
    constexpr double kMaxStepDist = 30.0;
    constexpr double kMaxHeadingJump = 1.0;
    constexpr double kMaxKappa = 5.0;

    for (size_t i = 0; i < path_trajectory.size(); ++i) {
        const auto& p = path_trajectory.traj_point(i);
        if (!is_finite(p.x) || !is_finite(p.y) || !is_finite(p.heading) || !is_finite(p.kappa)) {
            return fail("non-finite path value at index " + std::to_string(i));
        }
        if (std::abs(p.kappa) > kMaxKappa) {
            return fail("abs(kappa) too large at index " + std::to_string(i));
        }
    }

    for (size_t i = 1; i < path_trajectory.size(); ++i) {
        const auto& p0 = path_trajectory.traj_point(i - 1);
        const auto& p1 = path_trajectory.traj_point(i);

        const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (dist > kMaxStepDist) {
            return fail("step distance too large at index " + std::to_string(i));
        }

        double heading_diff = p1.heading - p0.heading;
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2 * M_PI;
        if (std::abs(heading_diff) > kMaxHeadingJump) {
            return fail("heading jump too large at index " + std::to_string(i));
        }
    }

    if (has_latest_ego_state_) {
        double traj_heading =path_trajectory.get_matched_trj_point(latest_ego_state_.x, latest_ego_state_.y, latest_ego_state_.heading).first.heading;
        double ego_heading = latest_ego_state_.heading;
        double diff = traj_heading - ego_heading;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        double heading_diff = std::abs(diff);

        // 放宽航向误差检查，允许车辆在初始时刻或大角度偏差下仍能生成轨迹
        // 只要控制器能处理，规划器不应过度限制
        if (heading_diff > M_PI / 6) {
            return fail("Heading deviation too large at current time: traj=" + std::to_string(traj_heading) +
                        " ego=" + std::to_string(ego_heading) +
                        " diff=" + std::to_string(heading_diff));
        }
    }

    if (reason) {
        reason->clear();
    }
    return true;
}

bool TrajectoryManager::isSpeedProfileValid(
    const std::vector<FrenetPoint>& speed_profile,
    std::string* reason,
    size_t min_points) const
{
    auto fail = [&](const std::string& msg) {
        if (reason) *reason = msg;
        return false;
    };

    if (speed_profile.size() < min_points) {
        return fail("speed profile points < min_points (" + std::to_string(speed_profile.size()) +
                    " < " + std::to_string(min_points) + ")");
    }

    auto is_finite = [](double v) { return std::isfinite(v); };
    constexpr double kMaxSpeed = 120.0;   // m/s, already huge
    constexpr double kMaxAcc = 10.0;      // 30 -> 60 (temporary loosen)
    constexpr double kTimeEps = 1e-6;
    constexpr size_t kSkipHead = 5;       // skip first few points where QP may spike

    for (size_t i = 0; i < speed_profile.size(); ++i) {
        const auto& p = speed_profile[i];
        if (!is_finite(p.t) || !is_finite(p.s) || !is_finite(p.s_dot) || !is_finite(p.s_dot_dot)) {
            return fail("non-finite speed value at index " + std::to_string(i));
        }
        if (p.s < -1e-3) {
            return fail("negative s at index " + std::to_string(i));
        }
        if (std::abs(p.s_dot) > kMaxSpeed) {
            return fail("abs(s_dot) too large at index " + std::to_string(i));
        }
        // key change: ignore head points
        if (i >= kSkipHead && std::abs(p.s_dot_dot) > kMaxAcc) {
            return fail("abs(s_dot_dot) too large at index " + std::to_string(i));
        }
    }

    for (size_t i = 1; i < speed_profile.size(); ++i) {
        const auto& prev = speed_profile[i - 1];
        const auto& curr = speed_profile[i];

        const double dt = curr.t - prev.t;
        if (dt < -kTimeEps) {
            return fail("time not non-decreasing at index " + std::to_string(i));
        }

        const double ds = curr.s - prev.s;
        if (ds < -1) {
            return fail("s not non-decreasing at index " + std::to_string(i));
        }

        if (dt > kTimeEps) {
            const double implied_speed = ds / dt;
            if (std::abs(implied_speed) > kMaxSpeed * 1.5) {
                return fail("implied speed too large at index " + std::to_string(i));
            }
        }
    }

    if (reason) reason->clear();
    return true;
}

bool TrajectoryManager::isTrajectoryValid(
    const std::vector<TrajectoryPoint>& trajectory,
    std::string* reason,
    size_t min_points)
{
    auto fail = [&](const std::string& msg) {
        if (reason) {
            *reason = msg;
        }
        return false;
    };

    if (trajectory.size() < min_points) {
        return fail("trajectory points < min_points (" + std::to_string(trajectory.size()) + " < " + std::to_string(min_points) + ")");
    }

    auto is_finite = [](double v) { return std::isfinite(v); };

    // 基础数值检查
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& p = trajectory[i];
        if (!is_finite(p.x) || !is_finite(p.y) || !is_finite(p.heading) || !is_finite(p.v) || !is_finite(p.time_stamped)) {
            return fail("non-finite value at index " + std::to_string(i));
        }
        if (!is_finite(p.kappa) || !is_finite(p.ax) || !is_finite(p.ay) || !is_finite(p.a_tau)) {
            return fail("non-finite dynamics at index " + std::to_string(i));
        }
        // 使用可配置的物理边界：用于排除明显异常
        if (std::abs(p.v) > max_speed_) {
            return fail("abs(v) too large at index " + std::to_string(i));
        }
        if (std::abs(p.ax) > max_acc_ || std::abs(p.ay) > max_acc_ || std::abs(p.a_tau) > max_acc_) {
            std::ostringstream oss;
            oss << "[TrajectoryValid] acc too large at index " << i
                << ": ax=" << p.ax << ", ay=" << p.ay << ", a_tau=" << p.a_tau
                << ", limit=" << max_acc_
                << ", v=" << p.v
                << ", heading=" << p.heading
                << ", kappa=" << p.kappa
                << ", t=" << p.time_stamped;
            std::cout << oss.str() << std::endl;
            return fail(oss.str());
        }
        if (std::abs(p.kappa) > max_curvature_) {
            return fail("abs(kappa) too large at index " + std::to_string(i));
        }
    }

    // 时间与连续性检查
    constexpr double kMaxStepDist = 80.0; // 单步位移过大基本不合理

    // 曲率连续性检查：kappa 跳变过大视为不合理
    // 说明：这里既检查绝对跳变，也检查按距离归一化后的跳变率（|dkappa|/ds）。
    // 阈值偏“安全保守”，如误杀可按实际地图/轨迹密度调整。
    constexpr double kMinDsForKappaRate = 0.2;       // 增加最小距离阈值，避免极小距离下的数值爆炸
    constexpr double kMaxAbsDeltaKappa = 20.0;         // 放宽相邻点曲率绝对跳变上限
    constexpr double kMaxDeltaKappaPerMeter = 50.0;    // 放宽相邻点曲率变化率上限 (1/m^2)

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& p0 = trajectory[i - 1];
        const auto& p1 = trajectory[i];

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        const double dist = std::hypot(dx, dy);
        if (dist > kMaxStepDist) {
            return fail("step distance too large at index " + std::to_string(i));
        }

        const double dkappa = p1.kappa - p0.kappa;
        if (std::abs(dkappa) > kMaxAbsDeltaKappa) {
            return fail("kappa jump too large at index " + std::to_string(i) +
                        " (|dkappa|=" + std::to_string(std::abs(dkappa)) +
                        " > " + std::to_string(kMaxAbsDeltaKappa) + ")");
        }
        if (dist > kMinDsForKappaRate) {
            const double dkappa_ds = std::abs(dkappa) / dist;
            if (dkappa_ds > kMaxDeltaKappaPerMeter) {
                return fail("kappa jump rate too large at index " + std::to_string(i) +
                            " (|dkappa|/ds=" + std::to_string(dkappa_ds) +
                            " > " + std::to_string(kMaxDeltaKappaPerMeter) + ")");
            }
        }
    }

    // 额外：检查 jerk（对 a_tau 做有限差分）
    double max_jerk = max_jerk_;
    for (size_t i = 1; i + 1 < trajectory.size(); ++i) {
        double dt1 = trajectory[i].time_stamped - trajectory[i-1].time_stamped;
        double dt2 = trajectory[i+1].time_stamped - trajectory[i].time_stamped;
        if (dt1 <= 1e-6 || dt2 <= 1e-6) continue;
        double jerk1 = (trajectory[i].a_tau - trajectory[i-1].a_tau) / dt1;
        double jerk2 = (trajectory[i+1].a_tau - trajectory[i].a_tau) / dt2;
        double jerk = std::max(std::abs(jerk1), std::abs(jerk2));
        if (jerk > max_jerk) return fail("Jerk out of bound");
    }

    if (reason) {
        reason->clear();
    }
    // 更新上一周期轨迹（同样优化）
    previous_trajectory_.clear();
    previous_trajectory_.insert(previous_trajectory_.end(),
                                trajectory.begin(),
                                trajectory.end());
    return true;
}

void TrajectoryManager::setLimits(double max_speed, double max_acc, double max_curvature, double max_jerk) {
    max_speed_ = max_speed;
    max_acc_ = max_acc;
    max_curvature_ = max_curvature;
    max_jerk_ = max_jerk;
}

void TrajectoryManager::getLimits(double& max_speed, double& max_acc, double& max_curvature, double& max_jerk) const {
    max_speed = max_speed_;
    max_acc = max_acc_;
    max_curvature = max_curvature_;
    max_jerk = max_jerk_;
}

} // namespace general
} // namespace AD_algorithm
