#pragma once

#ifndef CONTROL_AGENT_H
#define CONTROL_AGENT_H

#include <memory>
#include <string>
#include <vector>
#include <mutex>  // <<< 新增
#include <fstream>
#include <cstdint>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_info.hpp"
#include "pnc_msgs/msg/trajectory.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"

#include "general_modules/FrenetFrame.h"
#include "general_modules/Vehicle.h"
#include "general_modules/Trajectory.h"
#include "general_modules/ReferenceLine.h"
#include "general_modules/controller_base.h"
#include "controller/pid_lqr_controller.h"
#include "controller/mpc_controller.h"

namespace AD_algorithm {
namespace agent {

class ControlAgent : public rclcpp::Node
{
public:
    ControlAgent();

private:
    void control_run_step();
    void odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void ego_info_cb(const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info);
    void trajectory_cb(const pnc_msgs::msg::Trajectory::SharedPtr trajectory);
    void emergency_stop();
    std::shared_ptr<AD_algorithm::controller::ControllerBase> createController(const std::string& type);

    std::string _role_name = "ego_vehicle";

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;
    rclcpp::Subscription<pnc_msgs::msg::Trajectory>::SharedPtr _trajectory_subscriber;

    std::shared_ptr<AD_algorithm::general::VehicleState> _ego_state;
    std::vector<AD_algorithm::general::TrajectoryPoint> _trajectory;
    bool _trj_flag = false;

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _control_cmd_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _real_speed_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _real_heading_publisher;

    std::shared_ptr<AD_algorithm::controller::ControllerBase> _controller;
    std::string _controller_type = "pid_lqr";

    rclcpp::TimerBase::SharedPtr _control_timer;
    double _control_time_step = 0.02;
    std::unique_ptr<AD_algorithm::general::ControlCMD> _cmd;
    
    // 日志开关
    bool _enable_controller_log = false;

    // 文件日志（CSV）
    bool _enable_file_log = false;
    std::string _file_log_dir = "log";

    // <<< 新增互斥锁
    mutable std::mutex _ego_state_mutex;
    mutable std::mutex _trajectory_mutex;
};

} // namespace agent
} // namespace AD_algorithm

#endif // CONTROL_AGENT_H