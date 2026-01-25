#include <iostream>
#include "cilqr_planner/cilqr_planner.h"
#include "general_modules/math_tool.h"

namespace AD_algorithm {
namespace planner {

cilqrPlanner::cilqrPlanner() : PlannerBase("CilqrPlanner") {
    // 初始化problem和solver
    planner_params_ = std::make_shared<CILQRPlannerparams>();
    log("INFO","Creating problem...");
    // 在这里可以变动不同的cost_factory实现不同的CILQR变体
    // cilqr_cost_ = std::make_shared<cilqr_problem>(planner_params_);
    cilqr_cost_ = std::make_shared<cilqr_eigen>(planner_params_);
    cilqr_solver_ = std::make_shared<general::ILQR>(cilqr_cost_);
    traj_manager_ = std::make_shared<general::TrajectoryManager>();
    log("INFO","cilqrPlanner constructed");
}

void cilqrPlanner::set_initial_state(const AD_algorithm::general::TrajectoryPoint& x0){
    general::VehicleState init_state;
    init_state.x = x0.x;
    init_state.y = x0.y;
    init_state.heading = x0.heading;
    init_state.v = x0.v;
    cilqr_cost_->set_initial_state(init_state);
}
void cilqrPlanner::set_initial_state(const general::VehicleState& x0){
    cilqr_cost_->set_initial_state(x0);
}


void cilqrPlanner::set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles){
    cilqr_cost_->set_obstacles(obstacles);
}
void cilqrPlanner::set_reference_speed(double reference_speed){
    cilqr_cost_->set_reference_speed(reference_speed);
}
void cilqrPlanner::set_road_bounds(const double& lower_bound, const double& upper_bound){
    cilqr_cost_->set_road_bounds(lower_bound, upper_bound);
}
void cilqrPlanner::set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf){
    cilqr_cost_->set_cost_weights(Q, R, Qf);
}
void cilqrPlanner::solve(){
    cilqr_solver_->solve();
}

std::vector<general::TrajectoryPoint> cilqrPlanner::traj_densify(const std::vector<general::TrajectoryPoint>& raw_traj, double time_resolution){
    // 先使用线性插值对轨迹进行时间上的加密
    if (raw_traj.size() < 2) {
        return raw_traj;
    }
    std::vector<general::TrajectoryPoint> dense_traj;
    for (size_t i = 0; i < raw_traj.size() - 1; ++i) {
        const auto& p0 = raw_traj[i];
        const auto& p1 = raw_traj[i + 1];
        dense_traj.push_back(p0);
        double dt = p1.time_stamped - p0.time_stamped;
        int num_segments = static_cast<int>(std::ceil(dt / time_resolution));
        for (int j = 1; j < num_segments; ++j) {
            double ratio = static_cast<double>(j) / num_segments;
            general::TrajectoryPoint interp_point;
            interp_point.x = p0.x + ratio * (p1.x - p0.x);
            interp_point.y = p0.y + ratio * (p1.y - p0.y);
            interp_point.heading = p0.heading + ratio * (p1.heading - p0.heading);
            interp_point.v = p0.v + ratio * (p1.v - p0.v);
            interp_point.time_stamped = p0.time_stamped + ratio * dt;
            dense_traj.push_back(interp_point);
        }
    }
    dense_traj.push_back(raw_traj.back());
    return dense_traj;
}

std::vector<general::TrajectoryPoint>  cilqrPlanner::get_trajectory(double current_time){
    const Eigen::MatrixXd& x = cilqr_solver_->get_x();
    const Eigen::MatrixXd& u = cilqr_solver_->get_u();
    // std::cout<<"状态结果"<<std::endl;
    // std::cout<<x<<std::endl;
    // std::cout<<"控制结果"<<std::endl;
    // std::cout<<u<<std::endl;
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
    // 轨迹加密
    trajectory=traj_densify(trajectory,0.01);
    return trajectory;
}


std::vector<general::TrajectoryPoint> cilqrPlanner::plan(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const std::vector<general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {
    traj_manager_->setCurrentTime(current_time);
    traj_manager_->updateEgoState(ego_state);
    // 设置障碍物
    log("INFO","step1:设置障碍物,数量:", obstacles.size());
    std::vector<std::vector<general::Obstacle>> predicted_obstacles;
    predict_obstacles(current_time, obstacles,predicted_obstacles);
    set_obstacles(predicted_obstacles);
    // // 计算规划起点,时间是绝对时间
    // AD_algorithm::general::TrajectoryPoint plan_start_point = traj_manager_ -> calculatePlanningStartPoint(ego_state,planner_params_->ilqr_params->dt);
    // // 将规划起点的状态设置为初始状态
    // log("INFO","设置初始状态: x=", plan_start_point.x, ", y=", plan_start_point.y, ", heading=", plan_start_point.heading, ", v=", plan_start_point.v);
    // set_initial_state(plan_start_point);
    set_initial_state(*ego_state);
    // 设置道路边界
    log("INFO","设置参考速度:", reference_speed);
    set_reference_speed(reference_speed);
    // 分别是位置误差权重，航向误差权重，速度误差权重
    Eigen::MatrixXd Q = Eigen::Vector4d(1, 1, 20, 1000).asDiagonal();
    Eigen::MatrixXd R = Eigen::Vector2d(30, 0.5).asDiagonal();
    Eigen::MatrixXd Qf = Eigen::Vector4d(0.001, 0.001, 100.0, 10000.0).asDiagonal();
    set_cost_weights(Q, R, Qf);
    log("INFO","开始求解CILQR问题...");
    // 求解
    solve();
    // 转换为TrajectoryPoint
    std::vector<general::TrajectoryPoint> trajectory=get_trajectory(current_time);
    // 轨迹拼接
    // auto final_trj = traj_manager_->stitchTrajectory(trajectory);
    // log("INFO","规划完成，轨迹点数量:", final_trj.size());
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
    cilqr_cost_->set_frenet_frame(global_frenet_frame_);
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

void cilqrPlanner::predict_obstacles(double current_time, const std::vector<general::Obstacle>& obstacles,std::vector<std::vector<general::Obstacle>>& predicted_obstacles) {
    // 预测障碍物未来轨迹的逻辑，匀速直线模型
    double dt = planner_params_->ilqr_params->dt;
    int N = planner_params_->ilqr_params->N+1;  // 包括终端状态
    predicted_obstacles.clear();
    for (const auto& obs : obstacles) {
        std::vector<general::Obstacle> traj;;
        for (int i = 0; i < N; ++i) {
            general::Obstacle pred_obs = obs;
            double t = i * dt;
            pred_obs.x += obs.vx * t;
            pred_obs.y += obs.vy * t;
            pred_obs.time_stamp += t + current_time;
            traj.push_back(pred_obs);
        }
        predicted_obstacles.push_back(traj);
    }
}

} // namespace planner
} // namespace AD_algorithm