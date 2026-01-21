#include <iostream>
#include "cilqr_planner/cilqr_planner.h"

namespace AD_algorithm {
namespace planner {

cilqrPlanner::cilqrPlanner() : PlannerBase("CilqrPlanner") {
    // 初始化problem和solver
    planner_params_ = std::make_shared<CILQRPlannerparams>();
    log("INFO","Creating problem...");
    cilqr_problem_ = std::make_shared<planner_problem>(planner_params_);
    cilqr_solver_ = std::make_shared<general::ILQR>(cilqr_problem_);
    log("INFO","cilqrPlanner constructed");
}
void cilqrPlanner::set_initial_state(Eigen::VectorXd x0){
    cilqr_solver_->set_initial_state(x0);
}
void cilqrPlanner::set_obstacles(const std::vector<general::Obstacle>& obstacles){
    cilqr_problem_->set_obstacles(obstacles);
}
void cilqrPlanner::set_reference_speed(double reference_speed){
    cilqr_problem_->set_reference_speed(reference_speed);
}
void cilqrPlanner::set_road_bounds(const double& lower_bound, const double& upper_bound){
    cilqr_problem_->set_road_bounds(lower_bound, upper_bound);
}
void cilqrPlanner::set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf){
    cilqr_problem_->set_cost_weights(Q, R, Qf);
}
void cilqrPlanner::solve(){
    cilqr_solver_->solve();
}
std::vector<general::TrajectoryPoint>  cilqrPlanner::get_trajectory(double current_time){
    const Eigen::MatrixXd& x = cilqr_solver_->get_x();
    const Eigen::MatrixXd& u = cilqr_solver_->get_u();
    std::vector<general::TrajectoryPoint> trajectory;
    for (int i = 0; i < x.rows(); ++i) {
        general::TrajectoryPoint point;
        point.x = x(i, 0);
        point.y = x(i, 1);
        point.heading = x(i, 2);
        point.v = x(i, 3);
        point.time_stamped = current_time + i * planner_params_->ilqr_params->dt;
        if (i < u.rows()) {
            point.a_tau = u(i, 1);  // 加速度
        }
        trajectory.push_back(point);
    }
    return trajectory;
}

std::vector<general::TrajectoryPoint> cilqrPlanner::plan(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const std::vector<general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {
    // 设置初始状态
    Eigen::VectorXd x0(4);
    x0 << ego_state->x, ego_state->y, ego_state->heading, ego_state->v;
    log("INFO","设置初始状态: x=", x0[0], ", y=", x0[1], ", heading=", x0[2], ", v=", x0[3]);
    set_initial_state(x0);
    log("INFO","设置障碍物数量:", obstacles.size());
    set_obstacles(obstacles);
    log("INFO","设置参考速度:", reference_speed);
    set_reference_speed(reference_speed);
    // 分别是位置误差权重，速度误差权重，航向误差权重
    Eigen::MatrixXd Q = Eigen::Vector4d(1.0, 1.0, 10000, 1000).asDiagonal();
    Eigen::MatrixXd R = Eigen::Vector2d(0.01, 0.1).asDiagonal();
    Eigen::MatrixXd Qf = Eigen::Vector4d(1.0, 1.0, 10.0, 1000.0).asDiagonal();
    set_cost_weights(Q, R, Qf);
    log("INFO","开始求解CILQR问题...");
    // 求解
    solve();
    // 转换为TrajectoryPoint
    std::vector<general::TrajectoryPoint> trajectory=get_trajectory(current_time);
    log("INFO","规划完成，轨迹点数量:", trajectory.size());
    last_trajectory_=trajectory;
    return trajectory;
}

bool cilqrPlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
  if (reference_line.size() < 2) {
    log("INFO","Failed to set reference line: too few points");
    return false;
  }

  // 缓存参考路径并构建 FrenetFrame（参考 EMPlanner 的实现）
  try {
    global_frenet_frame_ = std::make_shared<AD_algorithm::general::FrenetFrame>(reference_line);
    cilqr_problem_->set_frenet_frame(global_frenet_frame_);
    // cache the reference line for later use in Combine/visualization
    log("INFO","Global reference line set (", reference_line.size(), " points)");
    return true;
  } catch (const std::exception& e) {
    log("INFO","Exception while constructing FrenetFrame: ", e.what());
    global_frenet_frame_.reset();
    return false;
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
    if (cilqr_solver_) {
        cilqr_solver_->set_log_enable(enable);
    }
}

std::vector<std::vector<general::TrajectoryPoint>> cilqrPlanner::GetExtralTraj() {
    // 返回额外轨迹（如果有）
    return {};
}

} // namespace planner
} // namespace AD_algorithm