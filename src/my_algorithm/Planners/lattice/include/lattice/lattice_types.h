#ifndef LATTICE_TYPES_H_
#define LATTICE_TYPES_H_

#include <vector>
#include <cmath>
#include <limits>
#include "general_modules/common_types.h"

namespace AD_algorithm {
namespace planner {

using PathPoint = AD_algorithm::general::PathPoint;
using Obstacle = AD_algorithm::general::Obstacle;

struct latticeFrenetPoint : public AD_algorithm::general::FrenetPoint {
 public:
  // Inherited: s, s_dot, s_dot_dot, l, l_dot, l_dot_dot, l_prime, l_prime_prime
  // s_dot = s_d, l_prime = l_ds
  
  double s_d_d_d = 0.0;
  double l_d_d_d = 0.0;
  double l_d_ds = 0.0; // dl_dot / ds
  double l_d_d_ds = 0.0; // dl_dot_dot / ds
  double t = 0.0; // Time relative to start of path
  double v = 0.0; // Cartesian velocity (approx)
  // Cartesian coordinates filled during conversion
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double cur = 0.0;
  
  // Helper setters/getters for compatibility with original code
  double& s_d() { return s_dot; }
  const double& s_d() const { return s_dot; }
  double& s_d_d() { return s_dot_dot; }
  const double& s_d_d() const { return s_dot_dot; }
  double& l_d() { return l_dot; }
  const double& l_d() const { return l_dot; }
  double& l_d_d() { return l_dot_dot; }
  const double& l_d_d() const { return l_dot_dot; }
  double& l_ds() { return l_prime; }
  const double& l_ds() const { return l_prime; }
};

struct latticeFrenetPath {
 public:
  int size_ = 0;
  double cost = 0.0;
  std::vector<latticeFrenetPoint> frenet_points;
  double max_speed = 0.0;
  double max_acc = 0.0;
  double max_curvature = 0.0;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TYPES_H_
