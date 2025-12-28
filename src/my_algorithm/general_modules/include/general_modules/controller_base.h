#pragma once
#include <vector>
#include <memory>
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace controller {

class ControllerBase {
public:
    ControllerBase(const std::string& name = "Controller") {
        logger_ = std::make_shared<general::Logger>(name);
    }
    virtual ~ControllerBase() = default;

    /**
     * @brief 设置日志开关
     */
    virtual void set_log_enable(bool enable) {
        enable_logging_ = enable;
        if (logger_) logger_->set_enable(enable);
    }

protected:
    /**
     * @brief 辅助函数，方便子类调用
     */
    template<typename... Args>
    void log(Args&&... args) const {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

    std::shared_ptr<general::Logger> logger_;
    bool enable_logging_ = true;

public:
    /**
     * @brief 设置参考轨迹
     * @param trajectory 轨迹点列表
     * @return 是否设置成功
     */
    virtual bool set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) = 0;
    
    /**
     * @brief 计算控制指令
     * @param ego_state 车辆状态
     * @param dt 控制周期
     * @param cur_t 当前时间
     * @param cmd 输出控制指令
     * @return 是否计算成功
     */
    virtual bool compute_control_cmd(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const double dt,
        const double cur_t,
        general::ControlCMD & cmd) = 0;
};

} // namespace controller
} // namespace AD_algorithm
