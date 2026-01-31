#include <iostream>
#include <memory>
#include "lattice/trajectory1d_generator.h"
#include "lattice/trajectory_evaluator.h"
#include "lattice/planner_weights.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 3.0;
  params.sampling.sample_min_time = 1.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;
  params.weights.weight_st_object = 1.0;
  params.weights.weight_st_jerk = 1.0;
  params.weights.weight_lt_offset = 1.0;
  params.weights.weight_lt_acc = 1.0;

  AD_algorithm::planner::Trajectory1DGenerator gen(params);
  auto collision = std::make_shared<AD_algorithm::planner::latticeCollisionDetection>();
  AD_algorithm::planner::TrajectoryEvaluator eval(params, collision);

  AD_algorithm::planner::latticeFrenetPoint init;
  init.s = 0.0; init.s_dot = 0.0; init.s_dot_dot = 0.0;
  init.l = 0.0; init.l_dot = 0.0; init.l_dot_dot = 0.0;

  auto lons = gen.GenerateLongitudinalCruising(init, params.cruise_speed);
  auto lats = gen.GenerateLateralCandidates(init, 2.0, params.cruise_speed);

  // no frenet frame available in this small unit test; pass nullptr and zero offsets
  std::shared_ptr<AD_algorithm::general::FrenetFrame> fr = nullptr;
  double s_offset = 0.0;
  double start_time = 0.0;
  eval.RankPairs(lons, lats, 8.0, fr, s_offset, start_time);

  std::cout << "Produced " << lons.size() * lats.size() << " pairs, queue size " << eval.num_of_pairs() << "\n";
  if (eval.has_more_pairs()) {
    auto top = eval.next_top_pair();
    std::cout << "Top pair cost: " << top.cost << " T_lon=" << top.lon->T << " T_lat=" << top.lat->T << "\n";
  }
  return 0;
}