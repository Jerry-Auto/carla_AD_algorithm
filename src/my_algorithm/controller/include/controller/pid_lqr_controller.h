#pragma once
#include "general_modules/controller_base.h"
#include "controller/lon_cascade_pid_controller.h"
#include "controller/lateral_lqr_controller.h"

namespace AD_algorithm {
namespace controller {

class PidLqrController : public ControllerBase {
public:
    PidLqrController();
    ~PidLqrController() = default;

    bool set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) override;
    
    bool compute_control_cmd(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const double dt,
        const double cur_t,
        general::ControlCMD & cmd) override;

    void set_log_enable(bool enable) override;

    // 暴露内部控制器以供配置（如果需要）
    std::shared_ptr<LonCascadePIDController> get_lon_controller() { return _lon_controller; }
    std::shared_ptr<LateralLQRController> get_lat_controller() { return _lat_controller; }

private:
    std::shared_ptr<LonCascadePIDController> _lon_controller;
    std::shared_ptr<LateralLQRController> _lat_controller;
};

} // namespace controller
} // namespace AD_algorithm
