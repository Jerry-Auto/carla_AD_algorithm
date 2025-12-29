#pragma once

#ifndef PLANNING_AGENT_H
#define PLANNING_AGENT_H

#include <memory>
#include <string>
#include <vector>
#include <mutex>  // <<< 新增
#include <atomic>
#include <fstream>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_info.hpp"
#include "derived_object_msgs/msg/object_array.hpp"
#include "pnc_msgs/msg/trajectory.hpp"

#include "general_modules/FrenetFrame.h"
#include "general_modules/Vehicle.h"
#include "general_modules/Trajectory.h"
#include "general_modules/ReferenceLine.h"
#include "general_modules/planner_base.h"
#include "emplanner/emplanner.h"
#include "general_modules/visualization_tool.h"


namespace AD_algorithm {
namespace agent {

class PlanningAgent : public rclcpp::Node
{
public:
    PlanningAgent();

private:
    void odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
    void ego_info_cb(const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr msg);
    void target_speed_cb(const std_msgs::msg::Float64::SharedPtr msg);
    void road_boundaries_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void path_cb(const nav_msgs::msg::Path::SharedPtr msg);
    void objects_cb(const derived_object_msgs::msg::ObjectArray::SharedPtr msg);
    void planning_run_step();
    std::shared_ptr<AD_algorithm::planner::PlannerBase> createPlanner(const std::string& type);

    std::vector<AD_algorithm::general::Obstacle> convertToPlannerObstacles(
        const std::vector<derived_object_msgs::msg::Object>& objects);

    std::string _role_name = "ego_vehicle";

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscriber;
    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr _ego_info_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _target_speed_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _road_boundaries_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscriber;
    rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr _object_array_subscriber;

    std::shared_ptr<AD_algorithm::general::VehicleState> _current_ego_state;
    double _reference_speed = 0.0;
    std::vector<AD_algorithm::general::PathPoint> _global_path;
    std::vector<AD_algorithm::general::Obstacle> _object_arrry;

    rclcpp::Publisher<pnc_msgs::msg::Trajectory>::SharedPtr _trajectory_publisher;

    double _planning_time_step = 0.2;
    rclcpp::TimerBase::SharedPtr _planning_timer;
    std::shared_ptr<AD_algorithm::planner::PlannerBase> _planner;
    std::string _planner_type = "emplanner";
        
    // 可视化轨迹
    std::shared_ptr<AD_algorithm::general::VisualizationTool> _viz_tool;
    // <<< 新增互斥锁
    mutable std::mutex _ego_state_mutex;
    mutable std::mutex _global_path_mutex;
    mutable std::mutex _objects_mutex;
    mutable std::mutex _reference_speed_mutex;
    mutable std::mutex _planner_mutex;

    bool _enable_planner_log = false;

    // 文件日志（CSV）
    bool _enable_file_log = true;
    std::string _file_log_dir = "log";

};

} // namespace agent
} // namespace AD_algorithm

#endif // PLANNING_AGENT_H