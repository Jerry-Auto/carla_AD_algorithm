#pragma once
#include <vector>
#include <memory>
#include "pid_controller.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/Vehicle.h"
#include "general_modules/Trajectory.h"
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace controller {
class LonCascadePIDController
{
public:
    // 构造函数,初始化默认控制器参数
    LonCascadePIDController(std::shared_ptr<general::Logger> logger = nullptr);

    // 规划出的轨迹拿来建立控制器的frenet坐标系
    bool set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) {
        _frenet_frame = std::make_shared<general::FrenetFrame>(trajectory,false);
        double delta=_frenet_frame->traj_point(_frenet_frame->size()-1).time_stamped
        -_frenet_frame->traj_point(0).time_stamped;
        log("INFO", "轨迹时长(s):", delta);
        return true;   
    };

    // 计算控制指令
    bool compute_control_cmd(const std::shared_ptr<general::VehicleState>& ego_state,
                             const double dt,double cur_t,general::ControlCMD & cmd);

    void set_log_enable(bool enable) {
        enable_logging_ = enable;
        if (logger_) logger_->set_enable(enable);
    }

    template<typename... Args>
    void log(Args&&... args) {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

    // 设置位置PID参数
    void set_station_controller(const double k_p, const double k_i, const double k_d);

    // 设置速度PID参数
    void set_speed_controller(const double k_p, const double k_i, const double k_d);

    // 设置位置PID积分限幅边界
    void set_station_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
    );

    // 设置速度PID积分限幅边界
    void set_speed_integral_saturation_boundary(
        const double high_boundary,
        const double low_boundary
    );
    
    // 重置积分项
    void reset_integral();

private:
    std::unique_ptr<PIDController> _station_controller;// 位置PID
    std::unique_ptr<PIDController> _speed_controller;// 速度PID

    double _preview_window = 0.0;// 预瞄点步长
    double _dt = 0.01;
    std::shared_ptr<general::FrenetFrame> _frenet_frame;

    std::shared_ptr<general::Logger> logger_;
    bool enable_logging_ = true;
};

}}