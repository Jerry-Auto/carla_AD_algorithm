#include <iostream>
#include "lattice/lattice_planner.h"
#include "lattice/lattice_collision_detection.h"
#include "lattice/planner_weights.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 4.0;
  params.sampling.sample_min_time = 2.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;
  params.weights.weight_st_object = 1.0;
  params.weights.weight_st_jerk = 1.0;
  params.weights.weight_lt_offset = 1.0;
  params.weights.weight_lt_acc = 1.0;
  params.weights.weight_st_acc = 0.5;
  params.limits.max_speed = 20.0;
  params.limits.max_acc = 12.0;
  params.limits.max_jerk = 100.0;

  auto collision_impl = std::make_shared<AD_algorithm::planner::latticeCollisionDetection>(std::vector<AD_algorithm::planner::Obstacle>{}, 0.5, std::vector<AD_algorithm::general::PathPoint>());
  AD_algorithm::planner::latticePlanner planner;

  // 设置直线参考路径
  std::vector<AD_algorithm::general::PathPoint> ref;
  for (int i=0;i<200;i++) {
    AD_algorithm::general::PathPoint p; p.x = i*0.5; p.y = 0.0; p.accumulated_s = i*0.5; p.heading=0.0; p.kappa=0.0; p.kappa_rate=0.0; ref.push_back(p);
  }
  planner.setGlobalReferenceLine(ref);

  auto ego = std::make_shared<AD_algorithm::general::VehicleState>();
  ego->x = 0.0; ego->y = 0.0; ego->v = 1.0; ego->heading = 0.0;

  // 在前方 0.5m 处添加一个静态障碍物：应使规划器拒绝靠近的路径
  AD_algorithm::planner::Obstacle obs; obs.x = 0.5; obs.y = 0.0; obs.length = 4.0; obs.width = 2.0; obs.vx = 0.0; obs.vy=0.0; obs.heading=0.0;

  auto traj_ok = planner.plan(ego, {}, 3.0, 0.0);
  auto traj_coll = planner.plan(ego, {obs}, 3.0, 0.0);

  if (traj_ok.empty()) { std::cerr << "Collision edge: baseline planner failed" << std::endl; return 1; }
  if (!traj_coll.empty()) { std::cerr << "Collision edge: planner returned trajectory despite obstacle" << std::endl; return 1; }

  std::cout << "Collision edge tests passed" << std::endl;
  return 0;
}
