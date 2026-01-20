#include "cilqr_planner/cost_define.h"

namespace AD_algorithm {
namespace planner {

problem::problem() {
    // initialize control limits: delta_max, a_min, a_max
    control_limits_ = casadi::SX::vertcat({0.5, -10.0, 10.0});
    // initialize vehicle params if needed
    vehicle_params_= AD_algorithm::general::VehicleParams();
}

// 定义系统单步前向离散动力学
casadi::SX problem::dynamics(casadi::SX& state, casadi::SX& control) {
    // 使用运动学模型，控制量：[steering_angle，a],状态量：[x, y,heading,v] 
    casadi::SX x = state(0);
    casadi::SX y = state(1);
    casadi::SX heading = state(2);
    casadi::SX v = state(3); 
    casadi::SX delta = control(0);
    casadi::SX a = control(1);
    casadi::SX dt = get_params()->dt;
    casadi::SX x_next = x + v * casadi::SX::cos(heading) * dt;
    casadi::SX y_next = y + v * casadi::SX::sin(heading) * dt;
    casadi::SX heading_next = heading + v * casadi::SX::tan(delta) / (vehicle_params_.lf + vehicle_params_.lr) * dt; //车辆轴距
    casadi::SX v_next = v + a * dt;
    casadi::SX next_state = casadi::SX::vertcat({x_next, y_next, heading_next, v_next});
    return next_state;  
}

// 定义原始每一时间步的代价函数,输入是原始笛卡尔状态，内部实现自己与参考轨迹的误差计算
casadi::SX problem::cost_function(const casadi::SX& state,const casadi::SX& control,casadi::SX index) {
    // 这里以简单的二次代价函数为例
    casadi::SX Q = casadi::SX::diag(casadi::SX::vertcat({100, 100, 1, 1})); // 状态权重
    casadi::SX R = casadi::SX::diag(casadi::SX::vertcat({1, 1}));       // 控制权重
    // 使用设置的成员变量 reference_trajectory_ 来获取参考轨迹点
    casadi::SX ref_state = casadi::SX::zeros(4); 

    casadi::SX state_error = state - ref_state;
    casadi::SX cost = 0.5 * casadi::SX::mtimes({state_error.T(), Q, state_error}) + 0.5 * casadi::SX::mtimes({control.T(), R, control});
    return cost;
}

// 定义终端代价函数
casadi::SX problem::terminal_cost(const casadi::SX& state) {
    // 这里以简单的终端二次代价函数为例
    casadi::SX Qf = casadi::SX::diag(casadi::SX::vertcat({20, 20, 5, 5})); // 终端状态权重
    casadi::SX state_ref = casadi::SX::zeros(state.size1()); // 参考状态
    casadi::SX state_error = state - state_ref;
    casadi::SX cost = 0.5 * casadi::SX::mtimes({state_error.T(), Qf, state_error});
    return cost;    
}
// 定义不等式约束函数，内部实现障碍物安全裕度代价，状态、控制数值边界约束(导数为常数)
casadi::SX problem::constraints(const casadi::SX& state,const casadi::SX& control,casadi::SX index) {
    return casadi::SX::vertcat({state_constraints(state, index),
                             control_constraints(control),
                             obstacle_avoidance_constraints(state, index)
    });
}

casadi::SX problem::state_constraints(const casadi::SX& state,casadi::SX index) {
    casadi::SX x = state(0);
    casadi::SX y = state(1);
    // 状态边界
    casadi::SX x_min = -10.0;
    casadi::SX x_max = 10.0;
    casadi::SX y_min = -10.0;
    casadi::SX y_max = 10.0;
    
    return casadi::SX::vertcat({
            x - x_max,  
            x_min - x,  
            y - y_max,  
            y_min - y  
        });
}

casadi::SX problem::control_constraints(const casadi::SX& control) {
    casadi::SX delta = control(0);
    casadi::SX a = control(1);
    return casadi::SX::vertcat({
            delta - control_limits_(0),  
            -control_limits_(0) - delta,  
            a - control_limits_(1),  
            control_limits_(2) - a  
        });
}

casadi::SX problem::obstacle_avoidance_constraints(const casadi::SX& state, casadi::SX index) {
    casadi::SX constraints = casadi::SX::zeros(0, 1);  // 初始化为空向量

    // 车辆状态
    casadi::SX x = state(0);
    casadi::SX y = state(1);
    casadi::SX heading = state(2);
    casadi::SX v = state(3);

    // 车辆前后点（假设参考点是重心，轴距 lf + lr）
    casadi::SX wheelbase = vehicle_params_.lf + vehicle_params_.lr;
    casadi::SX front_x = x + (vehicle_params_.lf) * casadi::SX::cos(heading);
    casadi::SX front_y = y + (vehicle_params_.lf) * casadi::SX::sin(heading);
    casadi::SX rear_x = x - (vehicle_params_.lr) * casadi::SX::cos(heading);
    casadi::SX rear_y = y - (vehicle_params_.lr) * casadi::SX::sin(heading);

    // 遍历障碍物
    for (const auto& obs : obstacles_) {
        // 障碍物位置（假设静态，x, y, heading, width, length）
        casadi::SX obs_x = obs.x;
        casadi::SX obs_y = obs.y;
        casadi::SX obs_heading = obs.heading;
        casadi::SX obs_width = obs.width;
        casadi::SX obs_length = obs.length;

        // 椭圆半轴（a: 沿heading方向，b: 垂直）
        casadi::SX a = obs_length / 2 + vehicle_params_.width / 2;  // 安全裕度
        casadi::SX b = obs_width / 2 + vehicle_params_.width / 2;

        // 计算前后点的安全裕度（椭圆内为负，外为正）
        // 变换到障碍物局部坐标
        casadi::SX cos_obs = casadi::SX::cos(obs_heading);
        casadi::SX sin_obs = casadi::SX::sin(obs_heading);
        casadi::SX dx_front = (front_x - obs_x) * cos_obs + (front_y - obs_y) * sin_obs;
        casadi::SX dy_front = -(front_x - obs_x) * sin_obs + (front_y - obs_y) * cos_obs;
        casadi::SX safety_front = 1 - (dx_front / a) * (dx_front / a) - (dy_front / b) * (dy_front / b);

        casadi::SX dx_rear = (rear_x - obs_x) * cos_obs + (rear_y - obs_y) * sin_obs;
        casadi::SX dy_rear = -(rear_x - obs_x) * sin_obs + (rear_y - obs_y) * cos_obs;
        casadi::SX safety_rear = 1 - (dx_rear / a) * (dx_rear / a) - (dy_rear / b) * (dy_rear / b);

        // 约束：safety >= 0，即 -safety <= 0
        constraints = casadi::SX::vertcat({constraints, -safety_front, -safety_rear});
    }

    return constraints;
}

}}

