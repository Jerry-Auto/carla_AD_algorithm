#include <rclcpp/rclcpp.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class VehicleInfoChecker : public rclcpp::Node
{
public:
  VehicleInfoChecker() : Node("vehicle_info_checker")
  {
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    this->declare_parameter<double>("timeout", 10.0);
    
    this->get_parameter("role_name", role_name_);
    this->get_parameter("timeout", timeout_);
    
    std::string topic_name = "/carla/" + role_name_ + "/vehicle_info";
    
    RCLCPP_INFO(this->get_logger(), "等待车辆信息话题: %s", topic_name.c_str());
    
    // 创建订阅者来检查话题是否存在
    auto callback = [this, topic_name](const carla_msgs::msg::CarlaEgoVehicleInfo::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "成功接收到车辆信息!");
      RCLCPP_INFO(this->get_logger(), "车辆ID: %u", msg->id);
      RCLCPP_INFO(this->get_logger(), "车辆类型: %s", msg->type.c_str());
      RCLCPP_INFO(this->get_logger(), "车辆角色名: %s", msg->rolename.c_str());
      RCLCPP_INFO(this->get_logger(), "车辆质量: %.2f kg", msg->mass);
      RCLCPP_INFO(this->get_logger(), "车辆质心: (%.2f, %.2f, %.2f)", 
                  msg->center_of_mass.x, msg->center_of_mass.y, msg->center_of_mass.z);
      
      if (msg->wheels.size() >= 4) {
        RCLCPP_INFO(this->get_logger(), "车辆有 %zu 个车轮", msg->wheels.size());
      }
      
      // 接收到消息后，可以关闭这个节点
      rclcpp::shutdown();
    };
    
    subscriber_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleInfo>(
      topic_name, 10, callback);
    
    // 设置超时定时器
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timeout_),
      [this, topic_name]() {
        RCLCPP_WARN(this->get_logger(), "超时: 未接收到车辆信息话题 %s", topic_name.c_str());
        RCLCPP_WARN(this->get_logger(), "尝试列出所有carla话题...");
        
        // 执行命令来列出所有话题
        std::system("ros2 topic list | grep carla");
        
        rclcpp::shutdown();
      }
    );
  }

private:
  std::string role_name_;
  double timeout_;
  rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInfoChecker>());
  rclcpp::shutdown();
  return 0;
}