#include "pidlqr/lon_cascade_pid_controller.h"

using namespace AD_algorithm::general;


namespace AD_algorithm {
namespace controller {
// 构造函数,初始化默认控制器参数
LonCascadePIDController::LonCascadePIDController(std::shared_ptr<general::Logger> logger)
    : logger_(logger)
{
    if (!logger_) {
        logger_ = std::make_shared<general::Logger>("LonCascadePIDController");
    }
    _station_controller = std::make_unique<PIDController>();
    _speed_controller = std::make_unique<PIDController>();
    // 设置控制器参数
    _station_controller->set_controller(1.0, 0.0, 0.0);
    _speed_controller->set_controller(4.0, 0.0, 0.0);
    _preview_window = 10.0; // 10 * 0.01s = 0.1s 预瞄
}

// 计算控制指令
bool LonCascadePIDController::compute_control_cmd
    (const std::shared_ptr<VehicleState>& ego_state,const double dt,
     double cur_t,ControlCMD & cmd)
{
    (void)cur_t; // unused
    _dt = dt;

    // 1. 计算目标点 (基于当前位置的预瞄)
    // 纵向控制的目标点应该是去跟随某一个时间戳对应的轨迹点
    // 为了鲁棒性，我们基于当前位置在轨迹上的投影时间进行预瞄，而不是绝对时间
    auto current_msg = _frenet_frame->get_matched_trj_point(ego_state->x, ego_state->y, ego_state->heading);
    double current_trj_time = current_msg.first.time_stamped;

    double target_time = current_trj_time + _preview_window * _dt;
    auto target_msg = _frenet_frame->get_matched_trj_point(target_time);
    const TrajectoryPoint& target_point = target_msg.first;
    double target_point_s = target_msg.second;

    // 2. 计算自车在Frenet坐标系下的位置
    auto ego_pro = _frenet_frame->cartesian_to_frenet(*ego_state);

    // 3. 串级PID控制
    // 3.1 位置PID (外环)
    // 输入：位置误差 (目标s - 实际s)
    // 输出：速度补偿量
    double station_error = target_point_s - ego_pro.s;
    double speed_offset = _station_controller->computer_control_cmd(station_error, dt);

    // 3.2 速度PID (内环)
    // 动态限速：基于横向误差
    double lat_error = ego_pro.l;
    double error_scale = 1.0;
    if (std::abs(lat_error) > 0.7) {
        // 误差大于 0.7m 时，开始线性降低目标速度，最大降低到 30%
        error_scale = std::max(0.3, 1.0 - 1.0 * (std::abs(lat_error) - 0.7));
    }
    double final_target_v = target_point.v * error_scale;

    // 输入：速度误差 (目标v - 实际v_s) + 速度补偿量
    // 输出：加速度指令
    double speed_error = final_target_v - ego_pro.s_dot;
    double speed_controller_input = speed_error + speed_offset;
    
    // 获取规划的参考加速度
    double ref_acc = target_point.a_tau; 

    // PID 计算出的只是 "误差修正量"
    double pid_output = _speed_controller->computer_control_cmd(speed_controller_input, dt);

    // 最终指令 = 参考加速度 + PID修正量
    double acceleration_cmd = ref_acc + pid_output;

    // 4. 设置控制指令
    cmd.set_acceleration(acceleration_cmd, ego_state->v);
    
    log("INFO", "S_err:", station_error, "V_err:", speed_error, "V_target:", final_target_v, "Acc_cmd:", acceleration_cmd, "Lat_err:", lat_error);

    return true;
}



// 设置位置PID参数
void LonCascadePIDController::set_station_controller
    (const double k_p, const double k_i, const double k_d)
{
    _station_controller->set_controller(k_p, k_i, k_d);
}

// 设置速度PID参数
void LonCascadePIDController::set_speed_controller
     (const double k_p, const double k_i, const double k_d)
{
    _speed_controller->set_controller(k_p, k_i, k_d);
}

// 设置位置PID积分限幅边界
void LonCascadePIDController::set_station_integral_saturation_boundary
    (const double high_boundary,const double low_boundary)
{
    _station_controller->set_integral_saturation_boundary(high_boundary, low_boundary);
}

// 设置速度PID积分限幅边界
void LonCascadePIDController::set_speed_integral_saturation_boundary
     (const double high_boundary,const double low_boundary)
{
    _speed_controller->set_integral_saturation_boundary(high_boundary, low_boundary);
}

// 重置积分项
void LonCascadePIDController::reset_integral()
{
    _station_controller->reset_integral();
    _speed_controller->reset_integral();
}

}}