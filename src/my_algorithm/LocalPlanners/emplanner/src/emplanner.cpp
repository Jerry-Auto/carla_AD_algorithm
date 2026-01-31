#include "emplanner/emplanner.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include "general_modules/math_tool.h"
#include "general_modules/tic_toc.h"

namespace AD_algorithm {
namespace planner {

using namespace general;

EMPlanner::EMPlanner() : PlannerBase("EMPlanner") {
    weights_ = WeightCoefficients();

    SpeedPlannerConfig speed_planner_config;
    trajectory_manager_ = std::make_unique<TrajectoryManager>(logger_);
    trajectory_manager_->setLimits(speed_planner_config.max_speed,
                                   speed_planner_config.max_acceleration,
                                   speed_planner_config.max_curvature,
                                   speed_planner_config.max_jerk);

    path_planner_ = std::make_unique<PathPlanner>(weights_, PathPlannerConfig(), logger_);
    speed_planner_ = std::make_unique<SpeedPlanner>(weights_, speed_planner_config, logger_);

    log("INFO", "EMPlanner initialized");
}

void EMPlanner::set_log_enable(bool enable) {
    if (logger_) logger_->set_enable(enable);
}

void EMPlanner::setWeights(const WeightCoefficients& weights) {
    weights_ = weights;
    // TODO: recreate planners to apply new weights
}

void EMPlanner::setPathPlannerConfig(const PathPlannerConfig& config) {
    path_planner_->set_config(config);
}

void EMPlanner::setSpeedPlannerConfig(const SpeedPlannerConfig& config) {
    speed_planner_->set_config(config);
}

bool EMPlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
    general::ReferenceLine ref_line(reference_line);
    global_frenet_frame_ = std::make_shared<general::FrenetFrame>(ref_line);
    return true;
}

bool EMPlanner::isTrajectoryValid(
    const std::vector<general::TrajectoryPoint>& trajectory,
    std::string* reason,
    size_t min_points) {
    if (!trajectory_manager_) {
        if (reason) *reason = "trajectory_manager_ is null";
        return false;
    }
    return trajectory_manager_->isTrajectoryValid(trajectory, reason, min_points);
}

std::vector<std::vector<general::TrajectoryPoint>> EMPlanner::GetExtralTraj() {
    std::vector<std::vector<general::TrajectoryPoint>> result;
    if (!path_planner_) {
        log("WARN", "GetExtralTraj: path_planner_ is null");
        return result;
    }
    if (!global_frenet_frame_) {
        log("WARN", "GetExtralTraj: global_frenet_frame_ is null");
        return result;
    }

    const auto& dp_path = path_planner_->getDPPath();
    if (dp_path.empty()) {
        log("INFO", "GetExtralTraj: dp_path is empty");
        return result;
    }

    std::vector<double> l_min, l_max;
    path_planner_->getLastConvexBounds(l_min, l_max);
    if (l_min.size() != dp_path.size() || l_max.size() != dp_path.size()) {
        log("WARN", "GetExtralTraj: convex bounds size mismatch with dp_path");
        return result;
    }

    const double offset_s = path_planner_->getLastOffsetS();

    std::vector<general::FrenetPoint> left_fp, right_fp;
    left_fp.reserve(dp_path.size());
    right_fp.reserve(dp_path.size());

    for (size_t i = 0; i < dp_path.size(); ++i) {
        general::FrenetPoint lp = dp_path[i];
        general::FrenetPoint rp = dp_path[i];
        lp.s += offset_s;
        rp.s += offset_s;
        lp.l = l_max[i];
        rp.l = l_min[i];
        left_fp.push_back(lp);
        right_fp.push_back(rp);
    }

    auto left_traj = global_frenet_frame_->frenet_to_cartesian(left_fp);
    auto right_traj = global_frenet_frame_->frenet_to_cartesian(right_fp);

    result.emplace_back(std::move(left_traj));
    result.emplace_back(std::move(right_traj));
    return result;
}

std::vector<TrajectoryPoint> EMPlanner::plan(
    const std::shared_ptr<VehicleState>& ego_state,
    const std::vector<Obstacle>& obstacles,
    double reference_speed,
    double current_time) {

    general::TicToc timer;
    log("INFO", "Starting planning cycle at time: " + std::to_string(current_time));

    trajectory_manager_->setCurrentTime(current_time);
    trajectory_manager_->updateEgoState(ego_state);

    log("INFO", "Step 1: Classifying obstacles...");
    std::vector<Obstacle> static_obstacles, dynamic_obstacles;
    trajectory_manager_->classifyObstacles(ego_state, obstacles, static_obstacles, dynamic_obstacles);

    if (static_obstacles.empty() && dynamic_obstacles.empty()) {
        log("WARN", "No relevant obstacles detected");
    }

    log("INFO", "Step 2: Calculating planning start point...");
    TrajectoryPoint planning_start_point =
        trajectory_manager_->calculatePlanningStartPoint(ego_state, weights_.forward_predict_time);

    log("INFO", "Step 3: Projecting static obstacles to Frenet frame...");
    std::vector<std::vector<FrenetPoint>> static_frenet_obstacles;
    for (const auto& obs : static_obstacles) {
        log("INFO", "Raw Static Obstacle: id=" + std::to_string(obs.id) +
                        " x=" + std::to_string(obs.x) +
                        " y=" + std::to_string(obs.y));
        auto frenet_corners = global_frenet_frame_->project_obstacle_to_frenet(obs);
        static_frenet_obstacles.push_back(frenet_corners);
        log("INFO", "Static obstacle projected with " + std::to_string(frenet_corners.size()) + " corners");
    }

    log("INFO", "Step 4: Path planning...");
    FrenetPoint start_frenet = global_frenet_frame_->cartesian_to_frenet(planning_start_point);
    start_frenet.l_prime_prime = 0.0;
    start_frenet.l_prime = clamp(start_frenet.l_prime, -2.0, 2.0);

    auto ego_frenet = global_frenet_frame_->cartesian_to_frenet(*ego_state);
    path_planner_->set_road_width(ego_state, start_frenet.s - ego_frenet.s);

    const size_t kMaxPathRetry = 3;
    std::vector<TrajectoryPoint> path_trajectory;
    std::string path_invalid_reason;
    bool path_valid = false;

    for (size_t attempt = 0; attempt < kMaxPathRetry; ++attempt) {
        path_trajectory = path_planner_->planPath(global_frenet_frame_, start_frenet, static_frenet_obstacles);

        if (path_trajectory.empty()) {
            log("WARN", "Path planning attempt " + std::to_string(attempt + 1) + " returned empty trajectory");
            continue;
        }
        if (!trajectory_manager_->isPathValid(path_trajectory, &path_invalid_reason, 10)) {
            std::string reason = path_invalid_reason.empty() ? "unknown" : path_invalid_reason;
            log("WARN", "Path planning attempt " + std::to_string(attempt + 1) + " produced invalid trajectory: " + reason);
            log("WARN", "路径校验失败: %s", reason.c_str());
            continue;
        }
        path_valid = true;
        break;
    }

    if (!path_valid) {
        log("Path planning failed after retries, returning empty trajectory", "ERROR");
        return {};
    }

    log("Path planning completed, generated " + std::to_string(path_trajectory.size()) + " points");
    std::shared_ptr<FrenetFrame> path_frame_ptr = std::make_shared<FrenetFrame>(path_trajectory, false);

    // ===== Step 5: Speed planning =====
    log("INFO", "Step 5: Speed planning...");
    std::vector<std::vector<FrenetPoint>> dynamic_frenet_obstacles;
    for (const auto& obs : dynamic_obstacles) {
        dynamic_frenet_obstacles.push_back(path_frame_ptr->project_dynamic_obstacle_to_frenet(obs));
    }

    // Use a stabilized start point for speed planning:
    // if the stitched/planning start speed is near zero but ego_state has valid speed,
    // use ego_state speed to avoid huge acceleration spikes.
    TrajectoryPoint speed_start_point = planning_start_point;
    if (ego_state) {
        constexpr double kMinStartSpeed = 0.5; // m/s
        if (speed_start_point.v < kMinStartSpeed && ego_state->v > kMinStartSpeed) {
            speed_start_point.v = ego_state->v;
            // also sync accel projection inputs
            speed_start_point.ax = ego_state->ax;
            speed_start_point.ay = ego_state->ay;
            speed_start_point.heading = ego_state->heading;
        }
    }

    const size_t kMaxSpeedRetry = 3;
    std::vector<FrenetPoint> speed_profile;
    std::string speed_invalid_reason;
    bool speed_valid = false;

    for (size_t attempt = 0; attempt < kMaxSpeedRetry; ++attempt) {
        speed_profile = speed_planner_->planSpeed(*path_frame_ptr, speed_start_point, reference_speed, dynamic_frenet_obstacles);
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

    log("INFO", "Step 6: Generating final trajectory...");
    auto final_trajectory = generateTrajectory(speed_profile, path_trajectory, planning_start_point.time_stamped);

    log("INFO", "Step 7: Stitching trajectory...");
    final_trajectory = trajectory_manager_->stitchTrajectory(final_trajectory);

    double elapsed_ms = timer.toc();

    log("INFO", "Planning completed in " + std::to_string(static_cast<int>(elapsed_ms)) + " ms");
    log("INFO", "Final trajectory has " + std::to_string(final_trajectory.size()) + " points");

    return final_trajectory;
}

std::vector<TrajectoryPoint> EMPlanner::generateTrajectory(
    const std::vector<FrenetPoint>& speed_profile,
    const std::vector<TrajectoryPoint>& path_trajectory,
    double start_time) {

    if (speed_profile.empty() || path_trajectory.empty()) {
        log("ERROR", "Cannot generate trajectory: empty input");
        return {};
    }
    if (path_trajectory.size() < 2) {
        log("ERROR", "Path trajectory too short for FrenetFrame");
        return {};
    }

    FrenetFrame path_frenet(path_trajectory, false);
    const auto& ref_path = path_frenet.get_reference_path();
    if (ref_path.empty()) {
        log("ERROR", "Reference path is empty");
        return {};
    }

    log("INFO", "起点信息(x,y,heading，s)：", path_trajectory[0].x, ",", path_trajectory[0].y, ",",
        path_trajectory[0].heading, ",", ref_path[0].accumulated_s);
    log("INFO", "st起点信息(t,s,v)：", speed_profile[0].t, ",", speed_profile[0].s, ",", speed_profile[0].s_dot);

    const double max_s = ref_path.back().accumulated_s;

    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(speed_profile.size());

    std::vector<double> s_values;
    s_values.reserve(ref_path.size());
    for (const auto& point : ref_path) s_values.push_back(point.accumulated_s);

    for (const auto& st_point : speed_profile) {
        TrajectoryPoint traj_point;
        traj_point.time_stamped = start_time + st_point.t;
        traj_point.v = st_point.s_dot;
        traj_point.a_tau = st_point.s_dot_dot;

        const double cur_s = general::clamp(st_point.s, 0.0, max_s);

        auto it = std::lower_bound(s_values.begin(), s_values.end(), cur_s);
        size_t idx;
        if (it == s_values.end()) idx = ref_path.size() - 1;
        else if (it == s_values.begin()) idx = 0;
        else idx = static_cast<size_t>(std::distance(s_values.begin(), it) - 1);

        double alpha = 0.0;
        if (idx < ref_path.size() - 1 && ref_path[idx + 1].accumulated_s > ref_path[idx].accumulated_s) {
            const double ds = ref_path[idx + 1].accumulated_s - ref_path[idx].accumulated_s;
            alpha = general::clamp((cur_s - ref_path[idx].accumulated_s) / ds, 0.0, 1.0);
        }

        const auto& p0 = ref_path[idx];
        const auto& p1 = (idx < ref_path.size() - 1) ? ref_path[idx + 1] : p0;

        const auto& tp0 = path_trajectory[idx];
        const auto& tp1 = (idx < path_trajectory.size() - 1) ? path_trajectory[idx + 1] : tp0;

        traj_point.x = p0.x + alpha * (p1.x - p0.x);
        traj_point.y = p0.y + alpha * (p1.y - p0.y);
        traj_point.heading = normalize_angle(tp0.heading + alpha * (tp1.heading - tp0.heading));
        traj_point.kappa = tp0.kappa + alpha * (tp1.kappa - tp0.kappa);

        const double a_n = traj_point.v * traj_point.v * traj_point.kappa;
        traj_point.ax = traj_point.a_tau * std::cos(traj_point.heading) - a_n * std::sin(traj_point.heading);
        traj_point.ay = traj_point.a_tau * std::sin(traj_point.heading) + a_n * std::cos(traj_point.heading);

        trajectory.push_back(std::move(traj_point));
    }

    log("INFO", "Generated " + std::to_string(trajectory.size()) + " trajectory points");
    return trajectory;
}

} // namespace planner
} // namespace AD_algorithm