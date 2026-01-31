#include <iostream>
#include <memory>
#include "lattice/lattice_collision_detection.h"
#include "lattice/planner_weights.h"
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"

int main() {
  AD_algorithm::planner::PlannerParams params;
  params.sampling.sample_max_time = 3.0;
  params.sampling.sample_min_time = 1.0;
  params.sampling.sample_time_step = 1.0;
  params.sampling.sample_lat_width = 1.0;
  params.sampling.sample_width_length = 0.5;

  AD_algorithm::planner::latticeCollisionDetection collision_detector;
  collision_detector.setParameters(params);

  // Create some dummy obstacles
  std::vector<AD_algorithm::general::Obstacle> obstacles;
  AD_algorithm::general::Obstacle obs1;
  obs1.id = 1;
  obs1.x = 10.0;
  obs1.y = 0.0;
  obs1.length = 4.0;
  obs1.width = 2.0;
  obstacles.push_back(obs1);

  // Create a dummy FrenetFrame
  std::vector<std::pair<double, double>> xy_points = {{0.0, 0.0}, {5.0, 0.0}};
  auto frenet_frame = std::make_shared<AD_algorithm::general::FrenetFrame>(xy_points);

  double start_time = 0.0;
  collision_detector.Update(obstacles, frenet_frame, start_time);

  // Create a dummy trajectory point
  AD_algorithm::general::TrajectoryPoint traj_point;
  traj_point.x = 5.0;
  traj_point.y = 0.0;
  traj_point.time_stamped = 1.0;

  double distance = collision_detector.DistanceToObstaclesAtTime(traj_point, 1.0);
  std::cout << "Distance to obstacles at time 1.0: " << distance << std::endl;

  // Create a dummy LatCandidate
  AD_algorithm::planner::LatCandidate lat_candidate;
  lat_candidate.T = 2.0;
  // Assume a simple polynomial curve
  lat_candidate.curve = std::make_shared<AD_algorithm::general::PolynomialCurve>();
  Eigen::VectorXd coeffs(5);
  coeffs << 0.0, 0.0, 0.0, 0.0, 0.0; // constant 0
  lat_candidate.curve->setCoefficients(coeffs);

  bool overlap = collision_detector.HasOverlapWithStaticObstacles(lat_candidate, 0.0);
  std::cout << "Has overlap with static obstacles: " << (overlap ? "Yes" : "No") << std::endl;

  return 0;
}