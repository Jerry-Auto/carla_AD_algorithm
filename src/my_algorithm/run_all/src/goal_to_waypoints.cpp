#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class GoalToWaypointsNode : public rclcpp::Node {
public:
  GoalToWaypointsNode() : rclcpp::Node("goal_to_waypoints") {
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    this->declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");

    this->get_parameter("role_name", role_name_);
    this->get_parameter("goal_topic", goal_topic_);

    const std::string carla_goal_topic = "/carla/" + role_name_ + "/goal";

    rclcpp::QoS qos_profile(10);
    qos_profile.reliable();
    qos_profile.durability_volatile();

    carla_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(carla_goal_topic, qos_profile);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, qos_profile,
        [this, carla_goal_topic](const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
          geometry_msgs::msg::PoseStamped out = *goal;
          out.header.stamp = this->now();
          if (out.header.frame_id.empty()) {
            out.header.frame_id = "map";
          }
          carla_goal_pub_->publish(out);
          RCLCPP_INFO(this->get_logger(), "已转发 RViz goal -> %s (x=%.2f,y=%.2f)",
                      carla_goal_topic.c_str(), out.pose.position.x, out.pose.position.y);
        });

    RCLCPP_INFO(this->get_logger(),
                "GoalToWaypoints 启动: role_name=%s, subscribe=%s, publish=%s (依赖 carla_waypoint_publisher 生成车道中心线路径)",
                role_name_.c_str(), goal_topic_.c_str(), carla_goal_topic.c_str());
  }

private:
  std::string role_name_;
  std::string goal_topic_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carla_goal_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalToWaypointsNode>());
  rclcpp::shutdown();
  return 0;
}
