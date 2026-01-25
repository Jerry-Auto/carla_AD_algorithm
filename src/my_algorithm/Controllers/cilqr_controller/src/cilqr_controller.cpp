#include "cilqr_controller/cilqr_controller.h"


namespace AD_algorithm {
namespace controller {

CilqrController::CilqrController(){
    controller_params_ = std::make_shared<CilqrControllerParams>();
    cilqr_cost_ =  std::make_shared<controller_cost>(controller_params_);
    cilqr_solver_ = std::make_shared<general::ILQR>(cilqr_cost_);
}


// 设置控制器跟踪轨迹
bool CilqrController::set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory){
    cilqr_cost_->set_reference_trajectory(trajectory);
    return true;
}

bool CilqrController::compute_control_cmd(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const double dt,
    const double cur_t,
    general::ControlCMD & cmd) {
    // 设置初始状态
    cilqr_cost_->set_initial_state(*ego_state, cur_t);
    // 设置权重矩阵 [位置，位置，航向，速度] 和 [转角，加速度]
    Eigen::MatrixXd Q = Eigen::Vector4d(1, 1, 20, 200).asDiagonal();
    Eigen::MatrixXd R = Eigen::Vector2d(30, 0.5).asDiagonal();
    Eigen::MatrixXd Qf = Eigen::Vector4d(10, 10, 200.0, 2000.0).asDiagonal();
    cilqr_cost_->set_cost_weights(Q, R, Qf);
    // 求解CILQR问题
    cilqr_solver_->solve();
    // 获取控制指令
    const Eigen::MatrixXd& u = cilqr_solver_->get_u();
    auto x = cilqr_solver_->get_x();
    if (u.rows() == 0) {
        log("ERROR", "CILQR solver returned empty control sequence.");
        return false;
    }
    // 由于规划周期大概是控制周期的10倍，所以这里取第0个控制量作为当前控制指令
    double accel_cmd = u(0, 1);  // 加速度
    double steer_cmd = u(0, 0);  // 转角
    cmd.set_acceleration(accel_cmd, x(0,3));
    cmd.set_steer(steer_cmd);
    return true;
}
    
}}