// vehicle_longitudinal_model.cpp
#include "pidlqr/Vehicle_model.h"

namespace AD_algorithm {
namespace controller {
VehicleLongitudinalModel::VehicleLongitudinalModel():VehicleLongitudinalModel(Parameters{}) {}

VehicleLongitudinalModel::VehicleLongitudinalModel(const Parameters& params) 
    : params_(params) {
    reset();
}

void VehicleLongitudinalModel::reset(double init_position, double init_velocity) {
    current_state_.position = init_position;
    current_state_.velocity = init_velocity;
    current_state_.acceleration = 0.0;
    current_state_.throttle = 0.0;
    current_state_.brake = 0.0;
    current_state_.engine_torque = 0.0;
    current_state_.brake_torque = 0.0;
}

void VehicleLongitudinalModel::update(double throttle_cmd, double brake_cmd, double dt) {
    // 限制输入范围
    throttle_cmd = std::max(0.0, std::min(1.0, throttle_cmd));
    brake_cmd = std::max(0.0, std::min(1.0, brake_cmd));
    
    // 更新油门和刹车
    current_state_.throttle = throttle_cmd;
    current_state_.brake = brake_cmd;
    
    // 计算发动机扭矩（线性模型）
    current_state_.engine_torque = params_.min_engine_torque + 
                                  throttle_cmd * (params_.max_engine_torque - params_.min_engine_torque);
    
    // 计算制动力矩
    current_state_.brake_torque = brake_cmd * params_.max_brake_torque;
    
    // 计算总驱动力（车轮上的力）
    double wheel_torque = current_state_.engine_torque * params_.gear_ratio - 
                         current_state_.brake_torque;
    double driving_force = wheel_torque / params_.wheel_radius;
    
    // 计算各种阻力
    double rolling_resistance = calculate_rolling_resistance();
    double aerodynamic_drag = calculate_aerodynamic_drag();
    
    // 总阻力（假设路面平坦）
    double total_resistance = rolling_resistance + aerodynamic_drag;
    
    // 计算净力
    double net_force = driving_force - total_resistance;
    
    // 计算加速度
    current_state_.acceleration = net_force / params_.mass;
    
    // 更新速度和位置（使用欧拉积分）
    current_state_.velocity += current_state_.acceleration * dt;
    
    // 防止速度变为负值
    if (current_state_.velocity < 0.0) {
        current_state_.velocity = 0.0;
        current_state_.acceleration = 0.0;
    }
    
    current_state_.position += current_state_.velocity * dt;
}

double VehicleLongitudinalModel::calculate_rolling_resistance() const {
    return params_.rolling_resistance * params_.mass * 9.81;
}

double VehicleLongitudinalModel::calculate_aerodynamic_drag() const {
    return 0.5 * params_.air_density * params_.frontal_area * 
           params_.drag_coefficient * current_state_.velocity * current_state_.velocity;
}

double VehicleLongitudinalModel::calculate_grade_resistance(double road_grade) const {
    return params_.mass * 9.81 * std::sin(road_grade);
}

void VehicleLongitudinalModel::print_state() const {
    std::cout << "位置: " << current_state_.position << " m, "
              << "速度: " << current_state_.velocity * 3.6 << " km/h, "
              << "加速度: " << current_state_.acceleration << " m/s², "
              << "油门: " << current_state_.throttle << ", "
              << "刹车: " << current_state_.brake << std::endl;
}

} // namespace simulation
} // namespace AD_algorithm