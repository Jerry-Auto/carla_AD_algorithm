#include "pnc_agent/planning_agent.h"
#include "general_modules/ReferenceLine.h"
#include "general_modules/csv_logger.h"
#include "general_modules/math_tool.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "rclcpp/qos.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <chrono>
#include <cmath>
#include <limits>

using namespace AD_algorithm::general;
using namespace AD_algorithm::agent;
using namespace AD_algorithm::planner;

namespace AD_algorithm {
namespace agent {

PlanningAgent::PlanningAgent() : Node("planning_agent")
{
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    this->get_parameter("role_name", _role_name);
    this->declare_parameter<std::string>("planner_type", "emplanner");
    this->get_parameter("planner_type", _planner_type);
    this->declare_parameter<bool>("enable_planner_log", false);
    this->get_parameter("enable_planner_log", _enable_planner_log);

    this->declare_parameter<bool>("enable_file_log", false);
    this->get_parameter("enable_file_log", _enable_file_log);
    this->declare_parameter<std::string>("file_log_dir", "log");
    this->get_parameter("file_log_dir", _file_log_dir);

    _current_ego_state = std::make_shared<VehicleState>();
    RCLCPP_INFO(this->get_logger(), "规划节点启动，角色名称: %s", _role_name.c_str());
    RCLCPP_INFO(this->get_logger(), "规划器日志开关: %s", _enable_planner_log ? "ON" : "OFF");

    if (_enable_file_log) {
        AD_algorithm::general::CSVLogger::getInstance().init(_file_log_dir, _role_name);
        RCLCPP_INFO(this->get_logger(), "CSVLogger 已初始化, 目录: %s", _file_log_dir.c_str());
    }

    rclcpp::QoS qos_profile(10);
    qos_profile.reliable();
    qos_profile.durability_volatile();

    std::string odom_topic = "/carla/" + _role_name + "/odometry";
    std::string imu_topic = "/carla/" + _role_name + "/imu";
    std::string speed_topic = "/carla/" + _role_name + "/target_speed";
    std::string path_topic = "/carla/" + _role_name + "/waypoints";

    _odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_profile,
        std::bind(&PlanningAgent::odometry_cb, this, std::placeholders::_1));
    _imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos_profile,
        std::bind(&PlanningAgent::imu_cb, this, std::placeholders::_1));
    _ego_info_subscriber = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
        "/carla/ego_vehicle/vehicle_info", qos_profile,
        std::bind(&PlanningAgent::ego_info_cb, this, std::placeholders::_1));
    _target_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        speed_topic, qos_profile,
        std::bind(&PlanningAgent::target_speed_cb, this, std::placeholders::_1));
        
    _path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
        path_topic, qos_profile,
        std::bind(&PlanningAgent::path_cb, this, std::placeholders::_1));

    _object_array_subscriber = this->create_subscription<derived_object_msgs::msg::ObjectArray>(
        "/carla/ego_vehicle/objects", qos_profile,
        std::bind(&PlanningAgent::objects_cb, this, std::placeholders::_1));

    std::string trajectory_topic = "/carla/" + _role_name + "/planning_trajectory";
    _trajectory_publisher = this->create_publisher<pnc_msgs::msg::Trajectory>(
        trajectory_topic, qos_profile);

    _planner = createPlanner(_planner_type);
    if (_planner) {
        _planner->set_log_enable(_enable_planner_log);
    } else {
        RCLCPP_ERROR(this->get_logger(), "无法创建规划器: %s", _planner_type.c_str());
    }

    _planning_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(_planning_time_step * 1000)),
        std::bind(&PlanningAgent::planning_run_step, this));

    _viz_tool = std::make_shared<AD_algorithm::general::VisualizationTool>("planning_visualizer");
    RCLCPP_INFO(this->get_logger(), "可视化工具已初始化");
    RCLCPP_INFO(this->get_logger(), "规划定时器启动，周期: %.3f秒", _planning_time_step);
}

void PlanningAgent::odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _current_ego_state->x = msg->pose.pose.position.x;
    _current_ego_state->y = msg->pose.pose.position.y;
    _current_ego_state->z = msg->pose.pose.position.z;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    _current_ego_state->v = std::sqrt(vx * vx + vy * vy);
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    _current_ego_state->heading = normalize_angle(yaw);
    _current_ego_state->flag_ode = true;
}

void PlanningAgent::imu_cb(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    if (std::abs(imu_msg->linear_acceleration.x) > 10.0 ||
        std::abs(imu_msg->linear_acceleration.y) > 10.0) return;

    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _current_ego_state->ax = imu_msg->linear_acceleration.x;
    _current_ego_state->ay = imu_msg->linear_acceleration.y;
    _current_ego_state->flag_imu = true;
}

void PlanningAgent::ego_info_cb(const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr ego_info)
{
    std::lock_guard<std::mutex> lock(_ego_state_mutex);
    _current_ego_state->id = ego_info->id;
    _current_ego_state->flag_info = true;
}

void PlanningAgent::target_speed_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_reference_speed_mutex);
    _reference_speed = msg->data / 3.6;
    if (_enable_planner_log) {
        RCLCPP_INFO(this->get_logger(), "收到目标速度: %.2f km/h", msg->data);
    }
}

void PlanningAgent::path_cb(const nav_msgs::msg::Path::SharedPtr waypoints)
{
    RCLCPP_INFO(this->get_logger(), "收到路径消息，包含 %zu 个航点", waypoints->poses.size());
    if (_enable_planner_log) {
        RCLCPP_INFO(this->get_logger(), "=== PATH CALLBACK CALLED ===");
    }
    std::vector<PathPoint> new_path;
    for (const auto& pose : waypoints->poses) {
        PathPoint pt;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        new_path.push_back(pt);
    }

    {
        std::lock_guard<std::mutex> lock(_global_path_mutex);
        _global_path = std::move(new_path);
    }

    // 收到全局 path：直接重建 planner 并设置参考线（避免历史状态影响）。
    // 注意：与 planning_run_step 并发访问 _planner 的问题，通过 _planner_mutex 串行化。
    {
        std::lock_guard<std::mutex> planner_lock(_planner_mutex);
        auto new_planner = createPlanner(_planner_type);
        new_planner->set_log_enable(_enable_planner_log);
        if (!new_planner->setGlobalReferenceLine(_global_path)) {
            RCLCPP_ERROR(this->get_logger(), "收到新 global_path，但设置参考线失败（points=%zu），保留原 planner", _global_path.size());
        } else {
            _planner = std::move(new_planner);
            if (_enable_planner_log) {
                RCLCPP_INFO(this->get_logger(), "已重建 planner 并更新参考线（points=%zu）", _global_path.size());
            }
        }
    }
}

void PlanningAgent::objects_cb(const derived_object_msgs::msg::ObjectArray::SharedPtr object_array)
{
    auto obstacles = convertToPlannerObstacles(object_array->objects);
    {
        std::lock_guard<std::mutex> lock(_objects_mutex);
        _object_arrry = std::move(obstacles);
    }
}

void PlanningAgent::planning_run_step()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    bool has_path;
    {
        std::lock_guard<std::mutex> lock(_global_path_mutex);
        has_path = !_global_path.empty();
    }
    if (!has_path) {
        RCLCPP_ERROR(this->get_logger(), "全局路径缺失，等待路径输入！");
        return;
    }

    bool ego_ready;
    {
        std::lock_guard<std::mutex> lock(_ego_state_mutex);
        ego_ready = _current_ego_state->flag_ode &&
                    _current_ego_state->flag_imu &&
                    _current_ego_state->flag_info;
    }
    if (!ego_ready) {
        RCLCPP_ERROR(this->get_logger(), "车辆状态数据不完整，等待传感器数据！");
        if(!_current_ego_state->flag_ode) {
            RCLCPP_ERROR(this->get_logger(), "缺少里程计数据");
        }
        if(!_current_ego_state->flag_imu) {
            RCLCPP_ERROR(this->get_logger(), "缺少IMU数据");
        }
        if(!_current_ego_state->flag_info) {
            RCLCPP_ERROR(this->get_logger(), "缺少车辆信息数据");
        }
        return;
    }

    // 拷贝所有数据
    auto ego_copy = std::make_shared<VehicleState>();
    std::vector<general::Obstacle> objects_copy;
    std::vector<general::PathPoint> global_path_copy;
    double ref_speed;
    {
        std::lock_guard<std::mutex> lock1(_ego_state_mutex);
        std::lock_guard<std::mutex> lock2(_global_path_mutex);
        std::lock_guard<std::mutex> lock3(_objects_mutex);
        std::lock_guard<std::mutex> lock4(_reference_speed_mutex);

        *ego_copy = *_current_ego_state;
        global_path_copy = _global_path;
        objects_copy = _object_arrry;
        ref_speed = _reference_speed;
    }

    bool is_valid = false;
    std::vector<general::TrajectoryPoint> trajectory;
    int max_replan_attempts = 5;
    int attempt = 0;
    while (!is_valid) {
        attempt++;
        if (attempt > max_replan_attempts) {
            RCLCPP_WARN(this->get_logger(), "规划多次失败，仍无法得到合理轨迹，使用最后一次规划结果");
            break;
        }
        trajectory.clear();
        {
            std::lock_guard<std::mutex> planner_lock(_planner_mutex);
            if (_planner) {
                trajectory = _planner->plan(ego_copy, objects_copy, ref_speed, this->now().seconds());
            }
        }
        // 若本次规划失败就重规划
        std::string invalid_reason;
        {
            std::lock_guard<std::mutex> planner_lock(_planner_mutex);
            if (_planner) {
                is_valid = _planner->isTrajectoryValid(trajectory, &invalid_reason);
            }
        }
        if (!is_valid && _enable_planner_log) {
            RCLCPP_WARN(this->get_logger(), "第 %d 次规划轨迹校验失败: %s", attempt, invalid_reason.c_str());
        }
        // 每次 plan() 之后都记录：包括不合理轨迹
        if (_enable_file_log) {
            AD_algorithm::general::CSVLogger::getInstance().logPlanningTrajectory(
                trajectory, ref_speed, "plan", is_valid, invalid_reason);
        }
    }

    // 发布
    pnc_msgs::msg::Trajectory pub_msg;
    for (const auto& point : trajectory) {
        pnc_msgs::msg::TrajectoryPoint tp;
        tp.x = point.x;
        tp.y = point.y;
        tp.heading = point.heading;
        tp.kappa = point.kappa;
        tp.v = point.v;
        tp.ax = point.ax;
        tp.ay = point.ay;
        tp.a_tau = point.a_tau;
        tp.time_stamped = point.time_stamped;
        pub_msg.points.push_back(tp);
    }
    _trajectory_publisher->publish(pub_msg);


    _viz_tool->FinalPathVisualization(trajectory); 
    _viz_tool->ObstacleVisualization(objects_copy);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    if (_enable_planner_log) {
        RCLCPP_INFO(this->get_logger(), "成功发布 %zu 个轨迹点, planning time: %ld ms",
                    pub_msg.points.size(), duration.count());
    }
}

std::vector<general::Obstacle> PlanningAgent::convertToPlannerObstacles(
    const std::vector<derived_object_msgs::msg::Object>& objects)
{
    std::vector<general::Obstacle> obstacles;
    std::shared_ptr<VehicleState> ego_state;
    {
        std::lock_guard<std::mutex> lock(_ego_state_mutex);
        ego_state = _current_ego_state; // 用于距离过滤
    }

    for (const auto& obj : objects) {
        if (obj.id == 0) continue;

        general::Obstacle obstacle;
        obstacle.id = obj.id;
        obstacle.x = obj.pose.position.x;
        obstacle.y = obj.pose.position.y;
        obstacle.z = obj.pose.position.z;
        obstacle.vx = obj.twist.linear.x;
        obstacle.vy = obj.twist.linear.y;
        obstacle.vz = obj.twist.linear.z;

        if (obj.shape.dimensions.size() >= 3) {
            obstacle.length = obj.shape.dimensions[0];
            obstacle.width = obj.shape.dimensions[1];
            obstacle.height = obj.shape.dimensions[2];
        } else {
            obstacle.length = 5.0;
            obstacle.width = 2.0;
            obstacle.height = 1.5;
        }

        double dx = obstacle.x - ego_state->x;
        double dy = obstacle.y - ego_state->y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < 100.0) {
            obstacles.push_back(obstacle);
        }
    }
    return obstacles;
}

std::shared_ptr<AD_algorithm::planner::PlannerBase> PlanningAgent::createPlanner(const std::string& type)
{
    if (type == "emplanner") {
        return std::make_shared<EMPlanner>();
    }
    // Future: Add other planners here
    // if (type == "lattice") return std::make_shared<LatticePlanner>();
    
    RCLCPP_ERROR(this->get_logger(), "未知规划器类型: %s, 默认使用 EMPlanner", type.c_str());
    return std::make_shared<EMPlanner>();
}


} // namespace agent
} // namespace AD_algorithm

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AD_algorithm::agent::PlanningAgent>());
    rclcpp::shutdown();
    return 0;
}