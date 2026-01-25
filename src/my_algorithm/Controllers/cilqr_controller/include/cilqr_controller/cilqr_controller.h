#pragma once
#include "general_modules/controller_base.h"
#include "general_modules/cilqr.h"
#include "cost_define.h"
#include <memory>
#include <vector>
#include "general_modules/common_types.h"

namespace AD_algorithm {
namespace controller {

class CilqrController : public ControllerBase {
public:
    CilqrController();
    ~CilqrController() = default;

    // 设置控制器跟踪轨迹
    bool set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) override;
    
    bool compute_control_cmd(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const double dt,
        const double cur_t,
        general::ControlCMD & cmd) override;
        
    void set_log_enable(bool enable) override {
        ControllerBase::set_log_enable(enable);
        if (cilqr_solver_) cilqr_solver_->set_log_enable(enable);
    }

private:
    std::shared_ptr<controller_cost> cilqr_cost_;
    std::shared_ptr<general::ILQR> cilqr_solver_;
    std::shared_ptr<CilqrControllerParams> controller_params_;
    std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame_;
};

} // namespace controller
} // namespace AD_algorithm
