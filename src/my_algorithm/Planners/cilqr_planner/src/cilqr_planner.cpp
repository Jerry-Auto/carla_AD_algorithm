#include <iostream>
#include "cilqr_planner/cilqr_planner.h"

namespace AD_algorithm {
namespace planner {

cilqrPlanner::cilqrPlanner() : PlannerBase("CilqrPlanner") {
    log("INFO", "[DEBUG] cilqrPlanner constructor start");
    // 初始化problem和solver
    log("INFO", "[DEBUG] Creating problem...");
    cilqr_problem_ = std::make_shared<problem>();
    log("INFO", "[DEBUG] Creating solver...");
    cilqr_solver_ = std::make_shared<general::cilqr>(cilqr_problem_);
    log("INFO", "[DEBUG] cilqrPlanner constructor end");
}

std::vector<general::TrajectoryPoint> cilqrPlanner::plan(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const std::vector<general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {
    // 设置初始状态
    Eigen::VectorXd x0(4);
    x0 << ego_state->x, ego_state->y, ego_state->heading, ego_state->v;
    cilqr_solver_->set_initial_state(x0);

    // 生成参考轨迹（基于全局参考线）
    std::vector<general::TrajectoryPoint> ref_trajectory;
    // 假设全局参考线已设置，生成未来N步的参考点
    if (global_frenet_frame_) {
        general::FrenetPoint ego_frenet_point = global_frenet_frame_->cartesian_to_frenet(*ego_state);
        double s0 = ego_frenet_point.s;
        log("INFO", "轨迹点数：", s0);
        for (int i = 0; i <= cilqr_solver_->get_parameters().N; ++i) {
            double t = current_time + i * cilqr_solver_->get_parameters().dt;
            double s = s0 + reference_speed * i * cilqr_solver_->get_parameters().dt;
            
            general::FrenetPoint ref_frenet;
            ref_frenet.s = s;
            ref_frenet.l = 0.0;
            ref_frenet.s_dot = reference_speed;
            ref_frenet.s_dot_dot = 0.0;
            ref_frenet.l_dot = 0.0;
            ref_frenet.l_dot_dot = 0.0;
            ref_frenet.l_prime = 0.0;
            ref_frenet.l_prime_prime = 0.0;

            general::TrajectoryPoint point = global_frenet_frame_->frenet_to_cartesian(ref_frenet);
            point.v = reference_speed;
            point.time_stamped = t;
            ref_trajectory.push_back(point);
        }
    }

    // 设置参考轨迹和障碍物到problem
    cilqr_problem_->set_reference_trajectory(ref_trajectory);
    cilqr_problem_->set_obstacle_list(obstacles);

    // 求解
    cilqr_solver_->solve();

    // 获取结果
    const Eigen::MatrixXd& u = cilqr_solver_->get_u();
    const Eigen::MatrixXd& x = cilqr_solver_->get_x();

    // 转换为TrajectoryPoint
    std::vector<general::TrajectoryPoint> trajectory;
    for (int i = 0; i < x.rows(); ++i) {
        general::TrajectoryPoint point;
        point.x = x(i, 0);
        point.y = x(i, 1);
        point.heading = x(i, 2);
        point.v = x(i, 3);
        point.time_stamped = current_time + i * cilqr_solver_->get_parameters().dt;
        if (i < u.rows()) {
            point.a_tau = u(i, 1);  // 加速度
        }
        trajectory.push_back(point);
    }

    return trajectory;
}

bool cilqrPlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
  if (reference_line.size() < 2) {
    log("Failed to set reference line: too few points");
    return false;
  }

  // 缓存参考路径并构建 FrenetFrame（参考 EMPlanner 的实现）
  try {
    global_frenet_frame_ = std::make_shared<AD_algorithm::general::FrenetFrame>(reference_line);
    // cache the reference line for later use in Combine/visualization
    log("Global reference line set (", reference_line.size(), " points)");
    return true;
  } catch (const std::exception& e) {
    log("Exception while constructing FrenetFrame: ", e.what());
    global_frenet_frame_.reset();
    return false;
  }
}

void cilqrPlanner::setPlannerParams(const AD_algorithm::general::cilqr_params& params) {
    // 其他参数...
    auto params_ptr = std::make_shared<AD_algorithm::general::cilqr_params>(params);
    cilqr_problem_->set_params(params_ptr);
    if (cilqr_solver_) {
        cilqr_solver_->set_params(params_ptr);
    }
}

bool cilqrPlanner::isTrajectoryValid(
    const std::vector<general::TrajectoryPoint>& trajectory,
    std::string* reason,
    size_t min_points) {
    if (trajectory.size() < min_points) {
        if (reason) *reason = "Trajectory too short";
        return false;
    }
    // 添加更多验证逻辑，如速度、加速度限制等
    return true;
}

void cilqrPlanner::set_log_enable(bool enable) {
    PlannerBase::set_log_enable(enable);
    // 如果 solver 支持日志，也可以在这里开启
    // if (cilqr_solver_) cilqr_solver_->set_log_enable(enable);
}

std::vector<std::vector<general::TrajectoryPoint>> cilqrPlanner::GetExtralTraj() {
    // 返回额外轨迹（如果有）
    return {};
}

} // namespace planner
} // namespace AD_algorithm