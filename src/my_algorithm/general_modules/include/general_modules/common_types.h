#pragma once

#include <cstdint>
#include <cmath>
#include <utility>

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

struct FrenetPoint {
    FrenetPoint() : s(0.0), s_dot(0.0), s_dot_dot(0.0), l(0.0), l_dot(0.0), l_dot_dot(0.0), l_prime(0.0), l_prime_prime(0.0) {}
    double s;
    double s_dot;
    double s_dot_dot;
    double l;
    double l_dot;
    double l_dot_dot;
    double l_prime;
    double l_prime_prime;
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

struct STPoint {
    STPoint() : t(0.0), s(0.0), s_dot(0.0), s_dot_dot(0.0) {}
    double t;
    double s;
    double s_dot;
    double s_dot_dot;

    bool operator==(const STPoint& other) const {
        return (t == other.t) && (s == other.s) && (s_dot == other.s_dot) && (s_dot_dot == other.s_dot_dot);
    }
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

    VehicleState& operator=(const VehicleState& other);
};

struct Obstacle {
    std::uint32_t id;
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double length;
    double width;
    double height;

    Obstacle()
        : id(0), x(0.0), y(0.0), z(0.0), vx(0.0), vy(0.0), vz(0.0),
          length(5.0), width(2.0), height(1.5) {}

    double getSpeed() const {
        return std::sqrt(vx * vx + vy * vy + vz * vz);
    }
};

} // namespace general
} // namespace AD_algorithm
