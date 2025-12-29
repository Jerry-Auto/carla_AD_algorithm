#include "pidlqr/pid_lqr_controller.h"

namespace AD_algorithm {
namespace controller {

PidLqrController::PidLqrController() : ControllerBase("PidLqrController") {
    _lon_controller = std::make_shared<LonCascadePIDController>(logger_);
    _lat_controller = std::make_shared<LateralLQRController>(logger_);

    // 默认配置 (可以根据需要调整，或者提供配置接口)
    _lon_controller->set_station_controller(0.1, 0.0, 0.0);
    _lon_controller->set_speed_controller(2.0, 1.0, 0.0);
    _lon_controller->set_speed_integral_saturation_boundary(0.5, -0.5);

    _lat_controller->set_r_matrix(5.0);
    std::vector<double> q_vector = {0.05, 0.1, 1.0, 5.0};
    _lat_controller->set_q_matrix(q_vector);
}

bool PidLqrController::set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) {
    bool ret = true;
    ret &= _lon_controller->set_trajectory(trajectory);
    ret &= _lat_controller->set_trajectory(trajectory);
    return ret;
}

bool PidLqrController::compute_control_cmd(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const double dt,
    const double cur_t,
    general::ControlCMD & cmd) 
{
    bool ret = true;
    ret &= _lon_controller->compute_control_cmd(ego_state, dt, cur_t, cmd);
    ret &= _lat_controller->compute_control_cmd(ego_state, dt, cur_t, cmd);
    return ret;
}

void PidLqrController::set_log_enable(bool enable) {
    ControllerBase::set_log_enable(enable);
    _lon_controller->set_log_enable(enable);
    _lat_controller->set_log_enable(enable);
}

} // namespace controller
} // namespace AD_algorithm
