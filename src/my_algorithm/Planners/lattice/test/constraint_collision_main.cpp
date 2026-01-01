#include <iostream>
#include "lattice/trajectory1d_generator.h"
#include "lattice/trajectory_combiner.h"

#include "lattice/lattice_collision_detection.h"
#include "lattice/planner_weights.h"
#include "general_modules/Trajectory.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 3.0;
  params.sampling.sample_min_time = 1.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;
  params.limits.max_speed = 20.0;
  params.limits.max_acc = 12.0;
  params.limits.max_curvature = 1.0;
  params.limits.max_jerk = 100.0;

  AD_algorithm::planner::Trajectory1DGenerator gen(params);
  AD_algorithm::planner::TrajectoryCombiner comb;
  AD_algorithm::general::TrajectoryManager traj_manager;
  traj_manager.setLimits(params.limits.max_speed, params.limits.max_acc, params.limits.max_curvature, params.limits.max_jerk);

  // 暂无障碍物
  std::vector<AD_algorithm::planner::Obstacle> obs;
  auto collide_impl = std::make_shared<AD_algorithm::planner::latticeCollisionDetection>(obs, 0.5, std::vector<AD_algorithm::general::PathPoint>());
  // 直接使用 latticeCollisionDetection 进行轨迹级碰撞查询

  AD_algorithm::planner::latticeFrenetPoint init;
  init.s = 0.0; init.s_dot = 1.0; init.s_dot_dot = 0.0;
  init.l = 0.0; init.l_dot = 0.0; init.l_dot_dot = 0.0;

  auto lons = gen.GenerateLongitudinalCruising(init);
  auto lats = gen.GenerateLateralCandidates(init, 2.0);
  // 构建更长的直线参考路径
  std::vector<AD_algorithm::general::PathPoint> ref_path;
  ref_path.reserve(100);
  for (int i=0;i<100;i++) {
    AD_algorithm::general::PathPoint p;
    p.x = i * 0.5; p.y = 0.0; p.heading = 0.0; p.accumulated_s = i * 0.5; p.kappa = 0.0; p.kappa_rate = 0.0;
    ref_path.push_back(p);
  }
  // 使用相同参考路径初始化碰撞检测实现
  collide_impl->Update(obs, ref_path);

  auto traj = comb.Combine(lons.back(), lats.front(), ref_path);

  // 诊断：打印前几个点
  std::cout << "Trajectory size: " << traj.size() << "\n";
  for (size_t i = 0; i < traj.size() && i < 10; ++i) {
    const auto& p = traj[i];
    std::cout << "pt[" << i << "] t=" << p.time_stamped << " x=" << p.x << " y=" << p.y
              << " v=" << p.v << " ax=" << p.ax << " a_tau=" << p.a_tau << " kappa=" << p.kappa << "\n";
  }

  double max_ax = 0.0; size_t max_idx = 0;
  for (size_t i=0;i<traj.size();++i) {
    if (std::abs(traj[i].ax) > max_ax) { max_ax = std::abs(traj[i].ax); max_idx = i; }
  }
  std::cout << "Max |ax| = " << max_ax << " at idx=" << max_idx << " t=" << traj[max_idx].time_stamped << "\n";
  std::cout << "Max point: v=" << traj[max_idx].v << " ax=" << traj[max_idx].ax << " a_tau=" << traj[max_idx].a_tau << "\n";

  double max_jerk = 0.0;
  for (size_t i = 1; i + 1 < traj.size(); ++i) {
    double dt1 = traj[i].time_stamped - traj[i-1].time_stamped;
    double dt2 = traj[i+1].time_stamped - traj[i].time_stamped;
    if (dt1 <= 1e-6 || dt2 <= 1e-6) continue;
    double jerk1 = (traj[i].a_tau - traj[i-1].a_tau) / dt1;
    double jerk2 = (traj[i+1].a_tau - traj[i].a_tau) / dt2;
    double jerk = std::max(std::abs(jerk1), std::abs(jerk2));
    max_jerk = std::max(max_jerk, jerk);
  }
  std::cout << "Max jerk = " << max_jerk << "\n";

  std::string reason;
  bool ok = traj_manager.isTrajectoryValid(traj, &reason);
  std::cout << "Constraint check: " << ok << " reason=" << reason << "\n";

  bool coll = collide_impl->InCollision(traj);
  std::cout << "Collision check: " << coll << "\n";
  
  // 在第一个点附近添加一个障碍物
  AD_algorithm::planner::Obstacle o;
  o.x = traj.front().x; o.y = traj.front().y; o.length = 4.0; o.width = 2.0; o.vx = 0.0; o.vy = 0.0; o.heading = 0.0;
  obs.push_back(o);
  collide_impl->Update(obs, ref_path);
  bool coll2 = collide_impl->InCollision(traj);
  std::cout << "Collision check after add obstacle: " << coll2 << "\n";

  return 0;
}