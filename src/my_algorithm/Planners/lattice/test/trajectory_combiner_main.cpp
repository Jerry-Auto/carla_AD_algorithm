#include <iostream>
#include "lattice/trajectory1d_generator.h"
#include "lattice/trajectory_combiner.h"
#include "lattice/planner_weights.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 3.0;
  params.sampling.sample_min_time = 1.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;

  AD_algorithm::planner::Trajectory1DGenerator gen(params);
  AD_algorithm::planner::TrajectoryCombiner comb;

  AD_algorithm::planner::latticeFrenetPoint init;
  init.s = 0.0; init.s_dot = 1.0; init.s_dot_dot = 0.0;
  init.l = 0.0; init.l_dot = 0.0; init.l_dot_dot = 0.0;

  auto lons = gen.GenerateLongitudinalCruising(init);
  auto lats = gen.GenerateLateralCandidates(init, 2.0);

  auto traj = comb.Combine(lons.back(), lats.front(), std::vector<AD_algorithm::planner::PathPoint>());
  if (traj.empty()) {
    std::cout << "Combiner correctly returned empty for empty ref_path\n";
  }

  // 构建一个简单的直线参考路径
  std::vector<AD_algorithm::planner::PathPoint> ref;
  for (int i=0;i<100;i++) {
    AD_algorithm::planner::PathPoint p;
    p.x = i*1.0; p.y = 0.0; p.accumulated_s = i*1.0;
    ref.push_back(p);
  }

  auto traj2 = comb.Combine(lons.back(), lats.front(), ref);
  std::cout << "Combined trajectory size: " << traj2.size() << "\n";
  if (!traj2.empty()) std::cout << "First point: (" << traj2.front().x << "," << traj2.front().y << ") t=" << traj2.front().time_stamped << "\n";
  return 0;
}