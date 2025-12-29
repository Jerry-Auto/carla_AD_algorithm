#include "lattice/lattice_planner.h"
#include <iostream>

namespace AD_algorithm {
namespace planner {

latticePlanner::latticePlanner(const double& cruise_speed,
                 std::unordered_map<std::string, double>& lattice_params,
                 std::shared_ptr<latticeCollisionDetection> collision_detection_ptr) {
    this->sample_max_time_ = lattice_params["sample_max_time"];
    this->sample_min_time_ = lattice_params["sample_min_time"];
    this->sample_time_step_ = lattice_params["sample_time_step"];
    this->sample_lat_width_ = lattice_params["sample_lat_width"];
    this->sample_width_length_ = lattice_params["sample_width_length"];
    this->weight_st_object_ = lattice_params["weight_st_object"];
    this->weight_st_jerk_ = lattice_params["weight_st_jerk"];
    this->weight_lt_offset_ = lattice_params["weight_lt_offset"];
    this->weight_lt_acc_ = lattice_params["weight_lt_acc"];
    this->cruise_speed_ = cruise_speed;
    this->collision_detection_ptr_ = collision_detection_ptr;
}

void latticePlanner::GetStValues(latticeFrenetPoint& frenet_point, 
    const std::shared_ptr<AD_algorithm::general::PolynomialCurve> st_polynomial,
    double t) {
  frenet_point.s = st_polynomial->value_evaluation(t, 0);
  frenet_point.s_d() = st_polynomial->value_evaluation(t, 1);
  frenet_point.s_d_d() = st_polynomial->value_evaluation(t, 2);
  frenet_point.s_d_d_d = st_polynomial->value_evaluation(t, 3);
  frenet_point.v = desired_speed_;
}

void latticePlanner::GetLtValues(latticeFrenetPoint& frenet_point, 
    const std::shared_ptr<AD_algorithm::general::PolynomialCurve> lt_polynomial,
    double t) {
  frenet_point.l = lt_polynomial->value_evaluation(t, 0);
  frenet_point.l_d() = lt_polynomial->value_evaluation(t, 1);
  frenet_point.l_d_d() = lt_polynomial->value_evaluation(t, 2);
  frenet_point.l_d_d_d = lt_polynomial->value_evaluation(t, 3);
}

double latticePlanner::GetStObjectCost(const latticeFrenetPath& frenet_path, 
                                       const double& target_speed) {
  double object_cost = 0.0, speed_cost = 0.0;
  double time_square_sum = 0.1, dist_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    speed_cost += pow(frenet_point.t, 2) * fabs(target_speed - frenet_point.s_d());
    time_square_sum += pow(frenet_point.t, 2);
  }
  speed_cost = speed_cost / time_square_sum;
  if (!frenet_path.frenet_points.empty()) {
      dist_cost = 1.0 / (1.0 + frenet_path.frenet_points.back().s);
  }
  object_cost = (speed_cost + 10 * dist_cost) / 11;

  return object_cost;
}

double latticePlanner::GetStJerkCost(const latticeFrenetPath& frenet_path) {
  double st_jerk_cost = 0.0, jerk_square_sum = 0.0, jerk_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    jerk_square_sum += pow(frenet_point.s_d_d_d / 4.0, 2);
    jerk_abs_sum += fabs(frenet_point.s_d_d_d / 4.0);
  }
  if (jerk_abs_sum > 1e-6) {
      st_jerk_cost = jerk_square_sum / jerk_abs_sum;
  }
  
  return st_jerk_cost;
}

double latticePlanner::GetLtOffsetCost(const latticeFrenetPath& frenet_path, 
                                       const latticeFrenetPoint& initial_frenet_point) {
  double lt_offset_cost = 0.0, offset_square_sum = 0.0, offset_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    // 反向则放大代价值
    if (frenet_point.l * initial_frenet_point.l < 0.0) {
      offset_square_sum += pow(frenet_point.l / 3.5, 2) * 5;
      offset_abs_sum += fabs(frenet_point.l / 3.5) * 5;
    }
    else {
      offset_square_sum += pow(frenet_point.l / 3.5, 2);
      offset_abs_sum += fabs(frenet_point.l / 3.5);
    }
  }
  if (offset_abs_sum > 1e-6) {
      lt_offset_cost = offset_square_sum / offset_abs_sum;
  }
  
  return lt_offset_cost;
}

double latticePlanner::GetLtAccCost(const latticeFrenetPath& frenet_path) {
  double max_acc = 0.0, lt_acc_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    if (fabs(max_acc) < fabs(frenet_point.l_d_d())) {
      max_acc = frenet_point.l_d_d();
    }
  }
  lt_acc_cost = fabs(max_acc);

  return lt_acc_cost;
}

std::vector<latticeFrenetPath> latticePlanner::SamplingFollowingFrenetPaths(
    const latticeFrenetPoint& initial_frenet_point, 
    const latticeFrenetPoint& leader_frenet_point) {
  std::vector<latticeFrenetPath> frenet_paths;
  for (double t_i = sample_min_time_; t_i <= sample_max_time_;) {
    t_i += sample_time_step_;
    
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    st_polynomial->curve_fitting(
        0.0, initial_frenet_point.s, initial_frenet_point.s_d(), 0.0,
        t_i, leader_frenet_point.s - 8.0, std::min(cruise_speed_, leader_frenet_point.v), 0.0
    );
    
    for (double l_i = -0.5 * sample_lat_width_; l_i <= 0.5 * sample_lat_width_;) {
      l_i += sample_width_length_;
      
      auto lt_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
      lt_polynomial->curve_fitting(
          0.0, initial_frenet_point.l, initial_frenet_point.l_d(), initial_frenet_point.l_d_d(),
          t_i, l_i, 0.0, 0.0
      );
      
      latticeFrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      for (double t = 0; t < t_i; t += 0.02) {
        latticeFrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial, t);
        GetLtValues(frenet_point, lt_polynomial, t);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_d() > frenet_path.max_speed ? 
            frenet_point.s_d() : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_d_d() > frenet_path.max_acc ? 
            frenet_point.s_d_d() : frenet_path.max_acc;
      }
      double st_object_cost = GetStObjectCost(frenet_path, cruise_speed_);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = weight_st_object_ * st_object_cost +
                         weight_st_jerk_ * st_jerk_cost + 
                         weight_lt_offset_ * lt_offset_cost + 
                         weight_lt_acc_ * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}

std::vector<latticeFrenetPath> latticePlanner::SamplingCruisingFrenetPaths(
    const latticeFrenetPoint& initial_frenet_point) {
  std::vector<latticeFrenetPath> frenet_paths;
  for (double t_i = sample_min_time_; t_i <= sample_max_time_;) {
    t_i += sample_time_step_;

    // Quartic Polynomial for longitudinal (Cruising)
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // Use quintic-style fitting (8 params) for compatibility with installed PolynomialCurve.
    // estimate end position by s + v * t (simple linear prediction)
    double est_end_s = initial_frenet_point.s + cruise_speed_ * t_i;
    st_polynomial->curve_fitting(
      0.0, initial_frenet_point.s, initial_frenet_point.s_d(), 0.0,
      t_i, est_end_s, cruise_speed_, 0.0
    );

    for (double l_i = -1 * sample_lat_width_; l_i <= sample_lat_width_;) {
      l_i += sample_width_length_;
      
      // Quintic Polynomial for lateral
      auto lt_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
      lt_polynomial->curve_fitting(
          0.0, initial_frenet_point.l, initial_frenet_point.l_d(), initial_frenet_point.l_d_d(),
          t_i, l_i, 0.0, 0.0
      );

      latticeFrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      
      for (double t = 0; t <= t_i; t += 0.02) {
        latticeFrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial, t);
        GetLtValues(frenet_point, lt_polynomial, t);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_d() > frenet_path.max_speed ? 
            frenet_point.s_d() : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_d_d() > frenet_path.max_acc ? 
            frenet_point.s_d_d() : frenet_path.max_acc;
      }
      double st_object_cost = GetStObjectCost(frenet_path, cruise_speed_);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = weight_st_object_ * st_object_cost +
                         weight_st_jerk_ * st_jerk_cost + 
                         weight_lt_offset_ * lt_offset_cost + 
                         weight_lt_acc_ * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}

void latticePlanner::GetCartesianPaths(std::vector<latticeFrenetPath>& frenet_paths, 
                                       const std::vector<PathPoint>& ref_path) {
  if (ref_path.empty()) return;
  
  // Create FrenetFrame for conversion
  // Note: FrenetFrame expects std::vector<std::pair<double, double>> or std::vector<TrajectoryPoint>
  // But we have std::vector<PathPoint>.
  // We can convert PathPoint to pair<double, double>
  std::vector<std::pair<double, double>> xy_points;
  for (const auto& p : ref_path) {
      xy_points.push_back({p.x, p.y});
  }
  
  // Construct FrenetFrame. Note: This might be expensive if done every cycle.
  // Ideally, FrenetFrame should be passed in or cached if ref_path doesn't change often.
  // But for now, we follow the structure.
  AD_algorithm::general::FrenetFrame frenet_frame(xy_points);

  for (auto& frenet_path : frenet_paths) {
    frenet_path.size_ = 0;
    for (size_t i = 0; i < frenet_path.frenet_points.size(); i++) {
      if (frenet_path.frenet_points[i].s >= ref_path.back().accumulated_s) {
        break;
      }
      frenet_path.size_++;
    }
    for (int i = 0; i< frenet_path.size_; i++) {
      // Convert Frenet to Cartesian using FrenetFrame
      // FrenetFrame::frenet_to_cartesian takes general::FrenetPoint
      // Our latticeFrenetPoint inherits from general::FrenetPoint
      auto traj_point = frenet_frame.frenet_to_cartesian(frenet_path.frenet_points[i]);
      
      frenet_path.frenet_points[i].x = traj_point.x;
      frenet_path.frenet_points[i].y = traj_point.y;
      frenet_path.frenet_points[i].yaw = traj_point.heading;
      frenet_path.frenet_points[i].cur = traj_point.kappa;
      frenet_path.frenet_points[i].v = desired_speed_;
    }
  }
}

std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> 
    latticePlanner::GetValidPaths(std::vector<latticeFrenetPath>& frenet_paths, 
                                  const latticeFrenetPoint& leader_frenet_point, 
                                  const bool& is_car_followed) {
  std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> valid_paths;
  for (auto& frenet_path : frenet_paths) {
    if (frenet_path.max_speed < MAX_SPEED && 
        frenet_path.max_acc < MAX_ACCEL && 
        frenet_path.max_curvature < MAX_CURVATURE && 
        !collision_detection_ptr_->IsCollision(frenet_path, 
                                               leader_frenet_point,
                                               is_car_followed)) {
      valid_paths.push(frenet_path);
    }
  }
  return valid_paths;
}

std::vector<latticeFrenetPath> latticePlanner::GetCandidatePaths(
    const std::vector<PathPoint>& ref_path, const latticeFrenetPoint& initial_frenet_point,
    const latticeFrenetPoint& leader_frenet_point, const bool& is_car_followed) {
  std::vector<latticeFrenetPath> frenet_paths;
  if (is_car_followed) {
    desired_speed_ = std::min(cruise_speed_, leader_frenet_point.v);
    frenet_paths = std::move(SamplingFollowingFrenetPaths(initial_frenet_point, 
                                                          leader_frenet_point));
  } 
  else {
    desired_speed_ = cruise_speed_;
    frenet_paths = std::move(SamplingCruisingFrenetPaths(initial_frenet_point));
  }
  GetCartesianPaths(frenet_paths, ref_path);
  auto&& valid_paths = 
      GetValidPaths(frenet_paths, leader_frenet_point, is_car_followed);

  std::vector<latticeFrenetPath> planning_paths;
  while (!valid_paths.empty()) {
    planning_paths.push_back(valid_paths.top());
    valid_paths.pop();
  }
  return planning_paths;
}

} // namespace planner
} // namespace AD_algorithm
