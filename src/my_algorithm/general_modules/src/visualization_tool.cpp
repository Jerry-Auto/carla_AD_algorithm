/* Copyright 2025 <Your Name> */

#include "general_modules/visualization_tool.h" 

namespace AD_algorithm {
namespace general {


VisualizationTool::VisualizationTool(const std::string & node_name)
: Node(node_name) {
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/reference_line/ref_path", 10);
    final_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/final_trajectory", 10);
    // 新增：用于带颜色的 final path
    final_path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/final_trajectory_marker", 10);
    sample_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/sample_trajectories", 10);
    history_paths_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/history_trajectories", 10);
    obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planning/obstacles", 10); // 新增
}

// ================== 通用转换模板 ==================
template<>
nav_msgs::msg::Path VisualizationTool::toNavPath<PathPoint>(const std::vector<PathPoint>& points) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (const auto& pt : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pt.heading);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
    }
    return path;
}

template<>
nav_msgs::msg::Path VisualizationTool::toNavPath<TrajectoryPoint>(const std::vector<TrajectoryPoint>& points) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (const auto& pt : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, pt.heading);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
    }
    return path;
}

// ================== 具体可视化函数 ==================

void VisualizationTool::RefPathVisualization(const std::vector<PathPoint>& ref_path) {
    if (ref_path.empty()) return;
    auto path_msg = toNavPath(ref_path);
    ref_path_pub_->publish(path_msg);
}

void VisualizationTool::FinalPathVisualization(const std::vector<TrajectoryPoint>& trajectory) {
    // 发布标准的 Path（兼容旧订阅者）
    if (!trajectory.empty()) {
        auto path_msg = toNavPath(trajectory);
        final_path_pub_->publish(path_msg);
    } else {
        // 为空时仍发布空 path 以便清除显示（或选择不发布）
        nav_msgs::msg::Path empty_path;
        empty_path.header.frame_id = "map";
        empty_path.header.stamp = this->now();
        final_path_pub_->publish(empty_path);
    }

    // 额外发布带颜色的 Line Marker（绿色，不透明）用于在 RViz 中高亮最终轨迹
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    if (trajectory.empty()) {
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "final_trajectory";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        final_path_marker_pub_->publish(marker_array);
        return;
    }

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "final_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.18;  // 线宽（更粗，增强可见性）

    // 品红（醒目），不透明
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.0f;

    marker.points.reserve(trajectory.size());
    for (const auto& pt : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    // 生命周期设置为略大于规划周期（例如0.5s），确保显示稳定
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker_array.markers.push_back(marker);
    final_path_marker_pub_->publish(marker_array);
}

void VisualizationTool::SamplePathsVisualization(
    const std::vector<std::vector<TrajectoryPoint>>& sample_trajectories) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(sample_trajectories.size());

    for (size_t i = 0; i < sample_trajectories.size(); ++i) {
        const auto& traj = sample_trajectories[i];
        if (traj.empty()) continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "sample_trajectories";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.08;  // 线宽

        // 蓝色半透明
        marker.color.r = 0.0f;
        marker.color.g = 0.3f;
        marker.color.b = 1.0f;
        marker.color.a = 0.4f;

        marker.points.reserve(traj.size());
        for (const auto& pt : traj) {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_array.markers.push_back(marker);
    }

    // 清除旧 marker（可选）
    if (marker_array.markers.empty()) {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = this->now();
        clear_marker.ns = "sample_trajectories";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
    }

    sample_paths_pub_->publish(marker_array);
}

void VisualizationTool::HistoryPathVisualization(
    const std::vector<std::vector<TrajectoryPoint>>& history_trajectories) {
    // 为避免 RViz 拥塞，只发布最后 N 条
    const size_t MAX_HISTORY = 5;
    size_t start = (history_trajectories.size() > MAX_HISTORY) 
                   ? history_trajectories.size() - MAX_HISTORY 
                   : 0;

    for (size_t i = start; i < history_trajectories.size(); ++i) {
        if (!history_trajectories[i].empty()) {
            auto path_msg = toNavPath(history_trajectories[i]);
            // 可选：修改 frame_id 或添加 offset 避免重叠
            history_paths_pub_->publish(path_msg);
        }
    }
}


void VisualizationTool::ObstacleVisualization(
    const std::vector<Obstacle>& obstacles)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(obstacles.size());

    for (const auto& obs : obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";          // 与轨迹坐标系统一
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = static_cast<int>(obs.id);    // 使用唯一 ID 避免闪烁
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置：立方体底部中心在 (x, y, z)，所以抬高 half height
        marker.pose.position.x = obs.x;
        marker.pose.position.y = obs.y;
        marker.pose.position.z = obs.z + obs.height / 2.0;

        // 朝向：由速度方向决定（若速度接近0，则朝向默认为0）
        double speed = obs.getSpeed();
        double yaw = (speed > 0.1) ? std::atan2(obs.vy, obs.vx) : 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        marker.pose.orientation = tf2::toMsg(q);

        // 尺寸
        marker.scale.x = obs.length;
        marker.scale.y = obs.width;
        marker.scale.z = obs.height;

        // 颜色：红色半透明
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6f;  // 半透明，便于看穿

        // 生命周期：略大于规划周期（如 0.1s 规划 → 0.2s lifetime）
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(marker);
    }

    // 发布
    obstacle_pub_->publish(marker_array);
}

}}
