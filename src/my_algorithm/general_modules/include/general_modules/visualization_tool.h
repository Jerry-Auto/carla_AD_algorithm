#ifndef AUTO_DRIVE_VISUALIZATION_TOOL_HPP_
#define AUTO_DRIVE_VISUALIZATION_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "general_modules/common_types.h"
#include <vector>
#include <sstream>
#include <iomanip>

namespace AD_algorithm {
namespace general {

    
class VisualizationTool : public rclcpp::Node {
public:
    explicit VisualizationTool(const std::string & node_name = "visualization_tool");
    // 参考路径（PathPoint）
    void RefPathVisualization(const std::vector<PathPoint>& ref_path);
    // 最终轨迹（TrajectoryPoint）
    void FinalPathVisualization(const std::vector<TrajectoryPoint>& trajectory);
    // 候选轨迹集合（多条）
    void SamplePathsVisualization(const std::vector<std::vector<TrajectoryPoint>>& sample_trajectories);
    // 历史轨迹（多条）
    void HistoryPathVisualization(const std::vector<std::vector<TrajectoryPoint>>& history_trajectories);
    // 障碍物可视化（Obstacle）
    void ObstacleVisualization(const std::vector<Obstacle>& obstacles);

private:
    // 通用转换函数：支持 PathPoint 和 TrajectoryPoint
    template<typename PointT>
    nav_msgs::msg::Path toNavPath(const std::vector<PointT>& points) const;

    // 发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_path_pub_;
    // Marker 发布器，用于带颜色的 final path 可视化
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr final_path_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_paths_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_paths_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
};
}}




#endif