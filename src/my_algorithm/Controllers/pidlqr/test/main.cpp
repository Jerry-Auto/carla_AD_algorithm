#include <memory>
#include <vector>
#include <cmath>
#include <iostream>

#include "pidlqr/lateral_lqr_controller.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include <Eigen/Dense>
#include "pidlqr/Vehicle_model.h"
#include "pidlqr/lon_cascade_pid_controller.h"


using namespace AD_algorithm::controller;
using namespace AD_algorithm::general;

/*********************************
 * 构造直线路径 y = 0
 *********************************/
std::vector<TrajectoryPoint> buildStraightTrajectory()
{
    std::vector<TrajectoryPoint> traj;

    double x = 0.0;
    double step = 0.5;
    double time = 0.0;

    for (int i = 0; i < 200; ++i)
    {
        TrajectoryPoint p;
        p.x = x;
        p.y = 0.0;
        p.heading = 0.0;
        p.kappa = 0.0;

        p.v = 10.0;           // 给定参考速度
        p.ax = 0.0;
        p.ay = 0.0;
        p.a_tau = 0.0;
        p.time_stamped = time;

        traj.push_back(p);
        x += step;
        time += step / p.v;
    }
    return traj;
}


void test_vehicle(){
    // 车辆参数示例
    double mass = 1500.0;   // kg
    double Iz   = 3000.0;   // kg*m^2
    double lf   = 1.2;      // 前轴到质心距离
    double lr   = 1.6;      // 后轴到质心距离
    double Cf   = 19000.0;  // 前轮侧偏刚度
    double Cr   = 33000.0;  // 后轮侧偏刚度
    double vx   = 15.0;     // m/s, 车辆纵向速度
    double dt   = 0.01;     // 时间步长 0.01s

    Vehicle2DOF vehicle(mass, Iz, lf, lr, Cf, Cr, vx, dt);

    // 设置初始状态（可选）
    VehicleState vs;
    vs.x = 0.0;
    vs.y = 0.0;
    vs.heading = 0.0;
    vs.v = vx;
    vs.omega = 0.0;
    vehicle.set_state(vs);

    // 模拟 5 秒钟，方向盘恒定偏转 5 度（转为弧度）
    double delta_deg = 5.0;
    double delta_rad = delta_deg * M_PI / 180.0;

    int steps = 5.0 / dt;

    for (int i = 0; i < steps; ++i) {
        vehicle.update(delta_rad);
        VehicleState cur = vehicle.getVehicleState();

        // 每 0.1 秒打印一次
        if (i % 10 == 0) {
            std::cout << "t=" << i*dt << "s, "
                      << "x=" << cur.x << ", "
                      << "y=" << cur.y << ", "
                      << "psi=" << cur.heading << ", "
                      << "vy=" << vehicle.getState()(0) << ", "
                      << "r=" << vehicle.getState()(1) << std::endl;
        }
    }

}

void test_lqr(){
    std::cout << "========== Lateral LQR Test ==========" << std::endl;
    // 创建被控车辆
    // ==== 参数 ====
    double Cf = 155494.663;
    double Cr = 155494.663;
    double m = 1845.0;
    double a = 2.852 / 2.0;
    double b = 2.852 / 2.0;
    double Iz = a*a*(m/2.0) + b*b*(m/2.0); // 按你公式计算
    double vx = 15.0;  // m/s
    double dt = 0.01;  // s

    Vehicle2DOF vehicle(m, Iz, a, b, Cf, Cr, vx, dt);

    // 1. 创建控制器
    LateralLQRController controller;

    // 2. 设置轨迹
    auto trajectory = buildStraightTrajectory();
    controller.set_trajectory(trajectory);

    // 3. 初始化车辆状态（带误差）
    // 使用二自由度模型
    auto ego_state = std::make_shared<VehicleState>();
    ego_state->x = 0.0;
    ego_state->y = 1.0;                         // 横向误差 1m
    ego_state->heading = 10*M_PI/180;    // 航向误差 5°，这里的单位是rad
    ego_state->v = 2.0;
    ego_state->omega = 0.0;
    ego_state->time_stamp = 0.0;

    vehicle.set_state(*ego_state);


    // 4. 闭环仿真 5 秒
    for (int i = 0; i < 2000; ++i)
    {
        ControlCMD cmd;
        bool ok = controller.compute_control_cmd(ego_state, dt, 0.0, cmd);
        if (!ok)
        {
            std::cerr << "[ERROR] control failed!" << std::endl;
        }

        double steer = cmd.get_steer();  // rad

        std::cout << "Step " << i
                  << " | y=" << ego_state->y
                  << " | heading(°)=" << ego_state->heading * 180.0 / M_PI
                  << " | 前轮转角(°)=" << steer* 180.0 / M_PI
                  << std::endl;

        vehicle.update(steer);
        *ego_state=vehicle.getVehicleState();
    }

    std::cout << "========== Test Finished ==========" << std::endl;
}

void test(){
    std::cout << "\n========== 最小LQR测试 ==========" << std::endl;
    
    // 1. 创建直线参考线
    std::vector<TrajectoryPoint> traj;
    for (int i = 0; i < 100; i++) {
        TrajectoryPoint pt;
        pt.x = i * 0.1;
        pt.y = 0.0;
        pt.heading = 0.0;
        pt.kappa = 0.0;
        traj.push_back(pt);
    }
    
    // 2. 创建控制器
    LateralLQRController controller;
    controller.set_trajectory(traj);
    
    // 3. 只测试一次控制计算
    auto state = std::make_shared<VehicleState>();
    state->x = 0.0;
    state->y = 0.1;      // 0.1米误差
    state->heading = 0.0;
    state->v = 20.0;      // 60m/s低速
    state->omega = 0.0;
    
    ControlCMD cmd;
    
    std::cout << "初始状态: y=" << state->y << "m, v=" << state->v << "m/s" << std::endl;
    
    if (controller.compute_control_cmd(state, 0.01, 0.0, cmd)) {
        std::cout << "控制量计算成功!" << std::endl;
        std::cout << "前轮转角: " << cmd.get_steer() * 180.0 / M_PI << "度" << std::endl;
    }
    
    std::cout << "========== 测试结束 ==========\n" << std::endl; 
}

void test_pid(){
    // === 1. 初始化 LonCascadePIDController 并设置控制器参数 ===
    LonCascadePIDController pid_controller;
    
    // 设置位置PID参数 (针对s)
    pid_controller.set_station_controller(1.0, 0.1, 0.05); // Kp, Ki, Kd
    
    // 设置速度PID参数 (针对v)
    pid_controller.set_speed_controller(0.5, 0.1, 0.01); // Kp, Ki, Kd
    
    // 设置积分限幅边界
    pid_controller.set_station_integral_saturation_boundary(1.0, -1.0);
    pid_controller.set_speed_integral_saturation_boundary(1.0, -1.0);

    // === 2. 创建参考轨迹 ===
    std::vector<TrajectoryPoint> ref_traj;
    double dt = 0.01; // 时间步长
    double total_time = 20.0; // 总仿真时间
    int N = static_cast<int>(total_time / dt);
    for (int i = 0; i <= N; ++i) {
        TrajectoryPoint pt;
        pt.time_stamped = i * dt;
        // 示例：线性加速到10m/s然后保持
        if (pt.time_stamped <= 10.0) {
            pt.v = pt.time_stamped * (10.0 / 10.0); // 加速阶段
        } else {
            pt.v = 10.0; // 匀速行驶
        }
        ref_traj.push_back(pt);
    }

    // 设置轨迹给控制器
    pid_controller.set_trajectory(ref_traj);

    // === 3. 初始化车辆模型 ===
    VehicleLongitudinalModel vehicle_model;
    VehicleLongitudinalModel::State vehicle_state;
    vehicle_state.velocity = 0.0; // 初始速度为0

    ControlCMD cmd;
    std::cout<<"开始仿真"<<std::endl;
    // === 4. 仿真循环 ===
    for (int i = 0; i < N; ++i) {
        // 获取当前时间
        double cur_t = i * dt;
        
        // 构建当前车辆状态
        VehicleState ego_state;
        ego_state.v = vehicle_state.velocity;
        ego_state.time_stamp = cur_t;
        
        // 计算控制指令
        if (!pid_controller.compute_control_cmd(
                std::make_shared<VehicleState>(ego_state), dt, cur_t, cmd)) {
            std::cerr << "PID control failed at step " << i << std::endl;
            break;
        }
        
        // 更新车辆状态
        vehicle_model.update(cmd.get_throttle(), cmd.get_brake(), dt);
        vehicle_state = vehicle_model.get_state();

        // 打印信息
        if (i % 10 == 0) {
            std::cout << "t=" << cur_t << "s | v=" << vehicle_state.velocity 
                      << ", throttle=" << cmd.get_throttle()
                      << ", brake=" << cmd.get_brake() << std::endl;
        }
    }

    std::cout << "Simulation finished." << std::endl;

}





int main(int argc, char** argv)
{
    // test_vehicle();
    // test_lqr();
    // test();
    test_pid();
    return 0;
}