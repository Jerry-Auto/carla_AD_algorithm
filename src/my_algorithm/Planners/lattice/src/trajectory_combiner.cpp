#include "lattice/trajectory_combiner.h"

#include <cmath>

namespace AD_algorithm {
namespace planner {

std::vector<AD_algorithm::general::TrajectoryPoint> TrajectoryCombiner::Combine(
    const Trajectory1DGenerator::LonCandidate& lon,
    const Trajectory1DGenerator::LatCandidate& lat,
    const std::vector<PathPoint>& ref_path,
    double s_offset) const {
  std::vector<AD_algorithm::general::TrajectoryPoint> trajectory;
  if (ref_path.empty()) return trajectory;

  // 使用参考路径的 XY 构造 FrenetFrame
  std::vector<std::pair<double, double>> xy_points;
  xy_points.reserve(ref_path.size());
  for (const auto& p : ref_path) xy_points.emplace_back(p.x, p.y);
  AD_algorithm::general::FrenetFrame frenet_frame(xy_points);

  double T = lon.T;
  int steps = std::max(1, static_cast<int>(T / 0.02));

  for (int i = 0; i <= steps; ++i) {
    double t = T * i / steps;
    double s = lon.curve->value_evaluation(t, 0);
    double s_dot = lon.curve->value_evaluation(t, 1);
    double s_ddot = lon.curve->value_evaluation(t, 2);

    // 横向以时间为参数进行评估（我们的生成器以时间参数化横向）
    double d = lat.curve->value_evaluation(t, 0);
    double d_dot = lat.curve->value_evaluation(t, 1);
    double d_ddot = lat.curve->value_evaluation(t, 2);

    // 构建 Frenet 点并计算 dl/ds 和 d2l/ds2 以便 frenet_to_cartesian 使用（空间导数）
    AD_algorithm::general::FrenetPoint fpt;
    fpt.t = t;
    // 先使用相对 s，然后在调用 frenet_to_cartesian 前加回 s_offset
    fpt.s = s + s_offset;
    fpt.s_dot = s_dot;
    fpt.s_dot_dot = s_ddot;
    fpt.l = d;
    fpt.l_dot = d_dot;
    fpt.l_dot_dot = d_ddot;

    const double eps = 1e-6;
    if (std::abs(s_dot) > eps) {
      fpt.l_prime = d_dot / s_dot; // dl/ds = (dl/dt)/(ds/dt)
      // 使用公式： l''_s = (d_ddot * s_dot - d_dot * s_ddot) / s_dot^3
      fpt.l_prime_prime = (d_ddot * s_dot - d_dot * s_ddot) / (s_dot * s_dot * s_dot);
    } else {
      fpt.l_prime = 0.0;
      fpt.l_prime_prime = 0.0;
    }

    auto traj_point = frenet_frame.frenet_to_cartesian(fpt);
    // 时间戳已由 frenet_to_cartesian（使用 fpt.t）填充（为相对时间，由调用者加 time_base）
    trajectory.push_back(traj_point);
  }

  return trajectory;
}

} // namespace planner
} // namespace AD_algorithm
