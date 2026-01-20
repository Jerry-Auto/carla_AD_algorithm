#pragma once
#include <vector>
#include <memory>
#include <string>
#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/planner_base.h"
#include "general_modules/common_types.h"

namespace AD_algorithm {
namespace planner {

class problem : public AD_algorithm::general::cost_base {
private:
    AD_algorithm::general::VehicleParams vehicle_params_;
    std::vector<AD_algorithm::general::TrajectoryPoint> reference_trajectory_;
    std::vector<casadi::SX> ref_states_;
    std::vector<AD_algorithm::general::Obstacle> obstacles_;    
    casadi::SX control_limits_;
public:
    problem();
    ~problem()=default;
    // 状态转移方程，使用运动学模型，控制量：[steering_angle，a],状态量：[x, y,heading,v]
    casadi::SX dynamics(casadi::SX& state, casadi::SX& control) override;
    // 定义原始每一时间步的代价函数,输入是原始笛卡尔状态，内部实现自己与参考轨迹的误差计算
    casadi::SX cost_function(const casadi::SX& state,const casadi::SX& control,casadi::SX index) override;
    // 定义终端代价函数
    casadi::SX terminal_cost(const casadi::SX& state) override;
    // 定义不等式约束函数，内部实现障碍物安全裕度代价，状态、控制数值边界约束(导数为常数)
    casadi::SX constraints(const casadi::SX& state,const casadi::SX& control,casadi::SX index) override;

    void set_reference_trajectory(const std::vector<AD_algorithm::general::TrajectoryPoint>& reference_trajectory) {
        reference_trajectory_ = reference_trajectory;
        ref_states_.clear();
        for (const auto& point : reference_trajectory_) {
            ref_states_.push_back(casadi::SX::vertcat({point.x, point.y, point.heading, point.v}));
        }
    }
    void set_obstacle_list(const std::vector<AD_algorithm::general::Obstacle>& obstacles) {
        obstacles_ = obstacles;
    }
    void set_state_limits(const AD_algorithm::general::VehicleState& state_limits) {
        // 在这里可以将状态限制存储为成员变量，以便在约束函数中使用
    }
    void set_control_limits(double max_steering_angle, double max_acceleration,double min_acceleration) {
        control_limits_ = casadi::SX::vertcat({max_steering_angle, max_acceleration, min_acceleration});
    }

private:
    casadi::SX state_constraints(const casadi::SX& state,casadi::SX index) ;
    casadi::SX control_constraints(const casadi::SX& control) ;
    casadi::SX obstacle_avoidance_constraints(const casadi::SX& state, casadi::SX index) ;
};

}}