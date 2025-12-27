#include <rclcpp/rclcpp.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_info_wheel.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;

class VehicleInfoPublisher : public rclcpp::Node
{
public:
  VehicleInfoPublisher() : Node("vehicle_info_publisher")
  {
    this->declare_parameter<std::string>("role_name", "ego_vehicle");
    this->declare_parameter<double>("publish_frequency", 1.0);
    
    this->get_parameter("role_name", role_name_);
    double frequency;
    this->get_parameter("publish_frequency", frequency);
    
    std::string topic_name = "/carla/" + role_name_ + "/vehicle_info";
    
    publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleInfo>(
      topic_name, rclcpp::QoS(10).reliable().durability_volatile());
    
    // 创建一个定时器来定期发布车辆信息
    auto timer_period = std::chrono::duration<double>(1.0 / frequency);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&VehicleInfoPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "车辆信息发布器启动，发布话题: %s，频率: %.1f Hz", 
                topic_name.c_str(), frequency);
  }

private:
  void timer_callback()
  {
    auto msg = carla_msgs::msg::CarlaEgoVehicleInfo();
    
    // 设置车辆基本信息
    msg.id = 1;  // 车辆ID
    msg.type = "vehicle.tesla.model3";  // 车辆类型
    msg.rolename = role_name_;  // 角色名
    msg.mass = 1500.0;  // 车辆质量 (kg)
    
    // 设置车辆参数
    msg.max_rpm = 7000.0;
    msg.moi = 1.0;  // 转动惯量
    msg.damping_rate_full_throttle = 0.15;
    msg.damping_rate_zero_throttle_clutch_engaged = 2.0;
    msg.damping_rate_zero_throttle_clutch_disengaged = 0.35;
    msg.use_gear_autobox = true;
    msg.gear_switch_time = 0.5;
    msg.clutch_strength = 10.0;
    msg.drag_coefficient = 0.3;
    
    // 设置质心位置
    msg.center_of_mass.x = 0.0;
    msg.center_of_mass.y = 0.0;
    msg.center_of_mass.z = 0.5;
    
    // 设置车轮信息（4个车轮）
    for (int i = 0; i < 4; ++i) {
      carla_msgs::msg::CarlaEgoVehicleInfoWheel wheel;
      
      // 设置车轮位置
      if (i == 0) {  // 前左轮
        wheel.position.x = 1.5;
        wheel.position.y = 0.75;
        wheel.position.z = 0.5;
      } else if (i == 1) {  // 前右轮
        wheel.position.x = 1.5;
        wheel.position.y = -0.75;
        wheel.position.z = 0.5;
      } else if (i == 2) {  // 后左轮
        wheel.position.x = -1.5;
        wheel.position.y = 0.75;
        wheel.position.z = 0.5;
      } else {  // 后右轮
        wheel.position.x = -1.5;
        wheel.position.y = -0.75;
        wheel.position.z = 0.5;
      }
      
      msg.wheels.push_back(wheel);
    }
    
    publisher_->publish(msg);
  }
  
  std::string role_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleInfo>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}