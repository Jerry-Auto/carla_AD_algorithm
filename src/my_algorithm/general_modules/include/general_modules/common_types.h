#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <memory>
#include <utility>
#include "general_modules/geometry.h"

namespace AD_algorithm {
namespace general {

struct PathPoint {
    PathPoint() : x(0.0), y(0.0), heading(0.0), accumulated_s(0.0), kappa_rate(0.0), kappa(0.0) {}
    double x;
    double y;
    double heading;
    double accumulated_s;
    double kappa_rate;
    double kappa;
};

struct TrajectoryPoint {
    TrajectoryPoint() : x(0.0), y(0.0), heading(0.0), kappa(0.0), v(0.0), ax(0.0), ay(0.0), a_tau(0.0), time_stamped(0.0) {}
    double x;
    double y;
    double heading;
    double kappa;
    double v;
    double ax;
    double ay;
    double a_tau; // 切向加速度
    double time_stamped;
};

struct FrenetPoint {
    FrenetPoint() : t(0.0), s(0.0), s_dot(0.0), s_dot_dot(0.0), l(0.0), l_dot(0.0), l_dot_dot(0.0), l_prime(0.0), l_prime_prime(0.0) {}
    double t;
    double s;
    double s_dot;
    double s_dot_dot;
    double l;
    double l_dot;
    double l_dot_dot;
    double l_prime;
    double l_prime_prime;
};




// 车辆参数结构体
struct VehicleParams {
    VehicleParams() : mass(1845.0), lf(1.426), lr(1.426), iz(3751.6), cf(155494.663),
                      cr(155494.663), max_steer(1.0), max_accel(4.0), max_decel(6.0) {}
    double mass;
    double lf;
    double lr;
    double iz;
    double cf;
    double cr;
    double max_steer;
    double max_accel;
    double max_decel;
    double width;
};

struct VehicleState {
    VehicleState()
        : x(0.0), y(0.0), z(0.0), heading(0.0), v(0.0), ax(0.0), ay(0.0), omega(0.0), alpha(0.0),
          time_stamp(0.0), id(0), flag_imu(false), flag_ode(false), flag_info(false) {}

    double x;
    double y;
    double z;
    double heading;
    double v;
    double ax;
    double ay;
    double omega;
    double alpha;
    double time_stamp;
    std::uint32_t id;
    bool flag_imu;
    bool flag_ode;
    bool flag_info;
    
    // 道路边界信息（相对于车辆当前位置/车道）
    // 存储前方道路宽度的采样点，resolution为采样间隔
    std::vector<double> road_width_left_vec;
    std::vector<double> road_width_right_vec;
    double road_width_resolution = 1.0; 

    VehicleState& operator=(const VehicleState& other);
};

struct Obstacle {
    std::uint32_t id;
    double x;
    double y;
    double z;
    double heading;
    double vx;
    double vy;
    double vz;
    double length;
    double width;
    double height;

    Obstacle()
        : id(0), x(0.0), y(0.0), z(0.0), heading(0.0), vx(0.0), vy(0.0), vz(0.0),
          length(5.0), width(2.0), height(1.5) {}

    double getSpeed() const {
        return std::sqrt(vx * vx + vy * vy + vz * vz);
    }
};

// SL障碍物表示（用于路径规划）
struct SLObstacle {
    std::shared_ptr<Polygon2d> polygon;
    double safety_margin;
    
    SLObstacle(const std::vector<FrenetPoint>& corners, double margin = 0.5);
    
    // 方便构造矩形障碍物 (s_center, l_center, length, width)
    SLObstacle(double s, double l, double length = 5.0, double width = 2.0, double margin = 0.5);
    
    // 检查(s,l)点是否在障碍物区域内
    bool contains(double s, double l, double safety_margin = 0.5) const;
    
    // 计算点到障碍物区域的最小距离
    double minDistanceTo(double s, double l) const;
};

// ST障碍物表示（用于速度规划）
struct STObstacle {
    std::shared_ptr<Polygon2d> polygon;
    double safety_margin;
    
    STObstacle(const std::vector<FrenetPoint>& points, double margin = 0.5);
    
    // 方便构造矩形障碍物
    STObstacle(double t_start, double t_end, double s_start, double s_end, double margin = 0.5);
    
    // 检查(t,s)点是否在障碍物区域内
    bool contains(double t, double s, double safety_margin = 0.5) const;
    
    // 计算点到障碍物区域的最小距离
    double minDistanceTo(double t, double s) const;
};


// 辅助函数：将FrenetPoint集合转换为SLObstacle
std::vector<SLObstacle> convertToSLObstacles(
    const std::vector<std::vector<FrenetPoint>>& frenet_obstacles,
    double safety_margin = 0.5);

// 辅助函数：将ST图节点转换为STObstacle
std::vector<STObstacle> convertToSTObstacles(
    const std::vector<std::vector<FrenetPoint>>& st_graph,
    double safety_margin = 0.5);

} // namespace general
} // namespace AD_algorithm
