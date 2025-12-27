#include <iostream>
#include <Eigen/Dense>
#include "general_modules/Vehicle.h"
using namespace Eigen;
namespace AD_algorithm {
namespace controller {
/// @brief 这些模型用于测试控制算法的可行性

class Vehicle2DOF {
public:
    Vehicle2DOF(double mass, double Iz, double lf, double lr, double Cf, double Cr, double vx, double dt)
        : m(mass), Iz(Iz), lf(lf), lr(lr), Cf(Cf), Cr(Cr), vx(vx), dt(dt) 
    {
        state = Vector2d::Zero(); // [vy, r]
        x = y = psi = 0.0;
    }

    // 设置内部状态vy, r
    void set_state(Vector2d vy_r){
        state = vy_r;
    }

    // 设置全局状态（自动转换为2DOF内部状态）
    void set_state(const AD_algorithm::general::VehicleState& veh_state){
        x = veh_state.x;
        y = veh_state.y;
        psi = veh_state.heading;
        vx = veh_state.v;
        state(0) = 0.0;       // 横向速度 vy 默认0，如果你有测量值可赋值
        state(1) = veh_state.omega; // 偏航角速度
    }

    // 更新车辆状态
    void update(double delta) {
        Vector2d x_dot = A() * state + B() * delta;
        state += x_dot * dt;

        double vy = state(0);
        double r  = state(1);

        // 更新全局位置和航向
        x += (vx * cos(psi) - vy * sin(psi)) * dt;
        y += (vx * sin(psi) + vy * cos(psi)) * dt;
        psi += r * dt;
    }

    // 获取内部状态 [vy, r]
    Vector2d getState() const { return state; }

    // 获取全局状态结构体
    AD_algorithm::general::VehicleState getVehicleState() const {
        AD_algorithm::general::VehicleState vs;
        vs.x = x;
        vs.y = y;
        vs.z = 0.0;
        vs.heading = psi;
        vs.v = vx;
        vs.omega = state(1);
        return vs;
    }

private:
    double m, Iz, lf, lr, Cf, Cr, vx, dt;
    Vector2d state;  // [vy, r]
    double x, y, psi;

    Matrix2d A() const {
        Matrix2d mat;
        mat(0,0) = -(Cf + Cr)/(m*vx);
        mat(0,1) = -vx - (Cf*lf - Cr*lr)/(m*vx);
        mat(1,0) = -(Cf*lf - Cr*lr)/(Iz*vx);
        mat(1,1) = -(Cf*lf*lf + Cr*lr*lr)/(Iz*vx);
        return mat;
    }

    Vector2d B() const {
        Vector2d mat;
        mat(0) = Cf/m;
        mat(1) = Cf*lf/Iz;
        return mat;
    }
};


class VehicleLongitudinalModel {
public:
    struct Parameters {
        double mass = 1500.0;           // 车辆质量 (kg)
        double wheel_radius = 0.3;      // 车轮半径 (m)
        double gear_ratio = 4.0;        // 传动比
        double max_engine_torque = 200.0;  // 最大发动机扭矩 (Nm)
        double min_engine_torque = -20.0;  // 发动机制动扭矩 (Nm)
        double max_brake_torque = 1000.0;  // 最大制动力矩 (Nm)
        double rolling_resistance = 0.015; // 滚动阻力系数
        double air_density = 1.225;     // 空气密度 (kg/m³)
        double frontal_area = 2.5;      // 迎风面积 (m²)
        double drag_coefficient = 0.3;  // 风阻系数
        double moment_of_inertia = 1.0; // 转动惯量 (kg·m²)
        Parameters() = default;
    };

    struct State {
        double position = 0.0;          // 位置 (m)
        double velocity = 0.0;          // 速度 (m/s)
        double acceleration = 0.0;      // 加速度 (m/s²)
        double throttle = 0.0;          // 油门开度 (0-1)
        double brake = 0.0;             // 刹车开度 (0-1)
        double engine_torque = 0.0;     // 发动机扭矩 (Nm)
        double brake_torque = 0.0;      // 制动力矩 (Nm)
        State()=default;
    };

    // 声明两个构造函数
    VehicleLongitudinalModel();
    VehicleLongitudinalModel(const Parameters& params);
    
    // 更新车辆状态
    void update(double throttle_cmd, double brake_cmd, double dt);
    
    // 重置状态
    void reset(double init_position = 0.0, double init_velocity = 0.0);
    
    // 获取当前状态
    const State& get_state() const { return current_state_; }
    
    // 计算各种阻力
    double calculate_rolling_resistance() const;
    double calculate_aerodynamic_drag() const;
    double calculate_grade_resistance(double road_grade = 0.0) const;
    
    // 设置参数
    void set_parameters(const Parameters& params) { params_ = params; }
    
    // 打印当前状态
    void print_state() const;

private:
    Parameters params_;
    State current_state_;
};


}}
