#include <iostream>
#include "lattice/trajectory1d_generator.h"
#include "lattice/planner_weights.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 3.0;
  params.sampling.sample_min_time = 1.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;

  AD_algorithm::planner::Trajectory1DGenerator gen(params);

  AD_algorithm::planner::latticeFrenetPoint init;
  init.s = 0.0; init.s_dot = 0.0; init.s_dot_dot = 0.0;
  init.l = 0.0; init.l_dot = 0.0; init.l_dot_dot = 0.0;

  auto lon = gen.GenerateLongitudinalCruising(init, params.cruise_speed);
  if (lon.empty()) {
    std::cerr << "Longitudinal cruising generation failed\n";
    return 1;
  }

  auto lat = gen.GenerateLateralCandidates(init, 2.0, params.cruise_speed);
  if (lat.empty()) {
    std::cerr << "Lateral generation failed\n";
    return 1;
  }

  std::cout << "Generated " << lon.size() << " longitudinal and " << lat.size() << " lateral candidates\n";
  return 0;
}