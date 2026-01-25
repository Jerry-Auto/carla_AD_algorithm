#include "pnc_agent/control_agent.h"
#include "general_modules/csv_logger.h"
#include "rclcpp/qos.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iomanip>
#include <memory>
#include "cilqr_controller/cilqr_controller.h"
#include <sstream>
#include <ctime>

using namespace AD_algorithm::general;
using namespace AD_algorithm::controller;
using namespace AD_algorithm::agent;

namespace AD_algorithm {
namespace agent {

ControlAgent::ControlAgent() : Node("control_agent")
{
    // 1. 参数声明与获取
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    this->get_parameter("role_name", _role_name);
    this->declare_parameter<std::string>("controller_type", "cilqr");  //mpc cilqr pidlqr
    this->get_parameter("controller_type", _controller_type);
    
    this->declare_parameter<bool>("enable_controller_log", false);
    this->get_parameter("enable_controller_log", _enable_controller_log);

    this->declare_parameter<bool>("enable_file_log", true);
    this->get_parameter("enable_file_log", _enable_file_log);
    this->declare_parameter<std::string>("file_log_dir", "log");
    this->get_parameter("file_log_dir", _file_log_dir);

    // 2. 车辆状态初始化
    _ego_state = std::make_shared<VehicleState>();
    _ego_state->flag_ode = true;
    _ego_state->flag_imu = true;
    _ego_state->flag_info = true;
    _ego_state->v = 0.0;
    _ego_state->x = 127.4;
    _ego_state->y = -195.4;
    _ego_state->heading = -3.08;

    RCLCPP_INFO(this->get_logger(), "控制节点启动，角色名称: %s, 控制器日志: %s", 
                _role_name.c_str(), _enable_controller_log ? "开启" : "关闭");

    if (_enable_file_log) {
        AD_algorithm::general::CSVLogger::getInstance().init(_file_log_dir, _role_name);
        RCLCPP_INFO(this->get_logger(), "CSVLogger 已初始化, 目录: %s", _file_log_dir.c_str());
    }
    
    // 3. 加载控制查找表
    std::string general_modules_path = ament_index_cpp::get_package_share_directory("general_modules");
    std::string lut_bin = general_modules_path + "/lut/control_lut.bin";
    std::string lut_json = general_modules_path + "/lut/control_lut.json";
    _cmd = std::make_unique<ControlCMD>(lut_bin, lut_json);

    // 4. ROS通信初始化
    rclcpp::QoS qos_profile(10);
    qos_profile.reliable();
    qos_profile.durability_volatile();

    std::string odom_topic = "/carla/" + _role_name + "/odometry";
    std::string imu_topic = "/carla/" + _role_name + "/imu";
    std::string trajectory_topic = "/carla/" + _role_name + "/planning_trajectory";
    std::string control_topic = "/carla/" + _role_name + "/vehicle_control_cmd";

    _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_profile,
        std::bind(&ControlAgent::odometry_cb, this, std::placeholders::_1));
    _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_profile,
        std::bind(&ControlAgent::imu_cb, this, std::placeholders::_1));
    _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
        "/carla/ego_vehicle/vehicle_info", qos_profile,
        std::bind(&ControlAgent::ego_info_cb, this, std::placeholders::_1));
    _trajectory_subscriber = this->create_subscription<pnc_msgs::msg::Trajectory>(
        trajectory_topic, qos_profile,
        std::bind(&ControlAgent::trajectory_cb, this, std::placeholders::_1));

    _control_cmd_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        control_topic, 10);
    _real_speed_publisher = this->create_publisher<std_msgs::msg::Float64>("real_speed", 10);
    _real_heading_publisher = this->create_publisher<std_msgs::msg::Float64>("real_heading", 10);

    _control_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(_control_time_step * 1000)),
        std::bind(&ControlAgent::control_run_step, this));

    // 5. 控制器初始化
    _controller = createController(_controller_type);
    if (_controller) {
        _controller->set_log_enable(_enable_controller_log);
        RCLCPP_INFO(this->get_logger(), "控制器初始化完成: %s", _controller_type.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法创建控制器: %s", _controller_type.c_str());
    }
}

std::shared_ptr<AD_algorithm::controller::ControllerBase> ControlAgent::createController(const std::string& type)
{
    if (type == "pid_lqr") {
        return std::make_shared<PidLqrController>();
    } else if (type == "mpc") {
        auto mpc = std::make_shared<MPCController>();
        std::vector<double> mpc_q_vector = {50.0, 0.0, 220.0, 0.0, 70.0, 20.0};
        std::vector<double> r_vector = {70.0, 10.0}; 
        mpc->set_matrix_Q(mpc_q_vector);
        mpc->set_matrix_R(r_vector);
        return mpc;
    }
    else if (type == "cilqr") {
        return std::make_shared<CilqrController>();
    }
    
    RCLCPP_ERROR(this->get_logger(), "未知控制器类型: %s, 默认使用 PidLqrController", type.c_str());
    return std::make_shared<PidLqrController>();
}

void ControlAgent::control_run_step()
{
    static int cycle_count = 0;
    cycle_count++;

    // 1. 检查状态
    bool ego_ready;
    {
        std::lock_guard<std::mutex> lock(_ego_state_mutex);
        ego_ready = _ego_state && _ego_state->flag_ode && _ego_state->flag_imu && _ego_state->flag_info;
    }

    bool traj_ready;
    {
        std::lock_guard<std::mutex> lock(_trajectory_mutex);
        traj_ready = !_trajectory.empty() && _trj_flag;
    }

    carla_msgs::msg::CarlaEgoVehicleControl cmd_msg;
    cmd_msg.hand_brake = false;
    cmd_msg.manual_gear_shift = false;

    // 2. 异常状态处理
    if (!ego_ready) {
        cmd_msg.throttle = 0.10; 
        cmd_msg.brake = 0.0;
        cmd_msg.steer = 0.0;
        _control_cmd_publisher->publish(cmd_msg);
        if (cycle_count % 100 == 0)
            RCLCPP_WARN(this->get_logger(), "车辆状态未就绪，输出测试油门: %.2f", cmd_msg.throttle);
        return;
    }

    if (!traj_ready) {
        cmd_msg.throttle = 0.10;
        cmd_msg.brake = 0.0;
        cmd_msg.steer = 0.0;
        _control_cmd_publisher->publish(cmd_msg);
        if (cycle_count % 100 == 0)
            RCLCPP_WARN(this->get_logger(), "等待轨迹，输出测试油门: %.2f", cmd_msg.throttle);
        return;
    }

    // 3. 拷贝数据 (减少持锁时间)
    auto ego_copy = std::make_shared<VehicleState>();
    std::vector<TrajectoryPoint> traj_copy;
    {
        std::lock_guard<std::mutex> lock1(_ego_state_mutex);
        std::lock_guard<std::mutex> lock2(_trajectory_mutex);
        *ego_copy = *_ego_state;
        traj_copy = _trajectory;
    }
    
    // 4. 执行控制计算
    double cur_t = this->now().seconds();
    try {
        if (_controller) {
            // 设置参考轨迹
            _controller->set_trajectory(traj_copy);

            // 计算控制量
            _controller->compute_control_cmd(ego_copy, _control_time_step, cur_t, *_cmd);
        }

        // 发布控制指令
        cmd_msg.throttle = _cmd->get_throttle();
        cmd_msg.brake = _cmd->get_brake();
        cmd_msg.steer = _cmd->get_steer();
        _control_cmd_publisher->publish(cmd_msg);

        if (_enable_file_log) {
            AD_algorithm::general::CSVLogger::getInstance().logControlCmd(
                cur_t,
                cmd_msg.throttle, cmd_msg.brake, cmd_msg.steer,
                ego_copy->x, ego_copy->y, ego_copy->heading, ego_copy->v);
        }

        if (_enable_controller_log && cycle_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "正常控制输出 -> 油门: %.3f, 刹车: %.3f, 转向: %.3f",
                       _cmd->get_throttle(), _cmd->get_brake(), _cmd->get_steer());
        }
    } catch (const std::exception& e) {
        // 异常处理：轻微制动或保持低速
        cmd_msg.throttle = 0.0;
        cmd_msg.brake = 0.5; // 增加制动以确保安全
        cmd_msg.steer = 0.0;
        _control_cmd_publisher->publish(cmd_msg);
        RCLCPP_ERROR(this->get_logger(), "控制计算异常: %s", e.what());
    }

    // 5. 发布调试信息
    if (cycle_count % 200 == 0) {
        std_msgs::msg::Float64 speed_msg, heading_msg;
        {
            std::lock_guard<std::mutex> lock(_ego_state_mutex);
            speed_msg.data = _ego_state->v;
            heading_msg.data = _ego_state->heading;
        }
        _real_speed_publisher->publish(speed_msg);
        _real_heading_publisher->publish(heading_msg);
    }
}

void ControlAgent::odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _ego_state->x = msg->pose.pose.position.x;
    _ego_state->y = msg->pose.pose.position.y;
    _ego_state->z = msg->pose.pose.position.z;

    _ego_state->v = std::sqrt(
        std::pow(msg->twist.twist.linear.x, 2) +
        std::pow(msg->twist.twist.linear.y, 2) +
        std::pow(msg->twist.twist.linear.z, 2));

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw = fmod(yaw + M_PI, 2 * M_PI);
    if (yaw < 0) yaw += 2 * M_PI;
    yaw -= M_PI;
    _ego_state->heading = yaw;

    _ego_state->flag_ode = true;
}

void ControlAgent::imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _ego_state->ax = imu_msg->linear_acceleration.x;
    _ego_state->ay = imu_msg->linear_acceleration.y;
    _ego_state->flag_imu = true;
}

void ControlAgent::ego_info_cb(const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
{
    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _ego_state->id = ego_info->id;
    _ego_state->flag_info = true;
}

void ControlAgent::trajectory_cb(const pnc_msgs::msg::Trajectory::SharedPtr trajectory)
{
    if (trajectory->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "轨迹点数为空！");
        return;
    }
    if (trajectory->points.size() < 5) {
        RCLCPP_WARN(this->get_logger(), "轨迹点数过少 (%zu)，忽略", trajectory->points.size());
        return;
    }

    std::vector<TrajectoryPoint> new_traj;
    for (const auto& point : trajectory->points) {
        TrajectoryPoint tp;
        tp.x = point.x;
        tp.y = point.y;
        tp.heading = point.heading;
        tp.kappa = point.kappa;
        tp.v = point.v;
        tp.ax = point.ax;
        tp.ay = point.ay;
        tp.a_tau = point.a_tau;
        tp.time_stamped = point.time_stamped;
        new_traj.push_back(tp);
    }

    if (_enable_file_log) {
        const double cur_t = this->now().seconds();
        AD_algorithm::general::CSVLogger::getInstance().logControlTrajectory(cur_t, new_traj);
    }

    {
        std::lock_guard<std::mutex> lock(_trajectory_mutex);
        _trajectory = std::move(new_traj);
        _trj_flag = true;
    }

    if (_enable_controller_log) {
        RCLCPP_INFO(this->get_logger(), "轨迹已更新，点数: %zu", _trajectory.size());
    }
}

void ControlAgent::emergency_stop()
{
    carla_msgs::msg::CarlaEgoVehicleControl control_msg;
    control_msg.throttle = 0.0;
    control_msg.brake = 1.0;
    control_msg.steer = 0.0;
    control_msg.reverse = false;
    control_msg.hand_brake = false;
    control_msg.manual_gear_shift = false;
    _control_cmd_publisher->publish(control_msg);
}

} // namespace agent
} // namespace AD_algorithm

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AD_algorithm::agent::ControlAgent>());
    rclcpp::shutdown();
    return 0;
}