#include "lattice/trajectory_evaluator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "lattice/lattice_types.h"

namespace AD_algorithm {
namespace planner {

namespace {

// clamp to [0,1]
inline double clamp01(double v) {
  if (v < 0.0) return 0.0;
  if (v > 1.0) return 1.0;
  return v;
}

inline double safe_div(double num, double den, double fallback = 0.0) {
  return (std::fabs(den) > 1e-12) ? (num / den) : fallback;
}

inline double positive_or(double v, double fallback) {
  return (v > 0.0 && std::isfinite(v)) ? v : fallback;
}

}  // namespace

TrajectoryEvaluator::TrajectoryEvaluator(const PlannerParams& params,
                                         const std::shared_ptr<latticeCollisionDetection>& CollisionDetection)
    : collision_detection_(CollisionDetection), planner_params_(params) {}

bool TrajectoryEvaluator::has_more_pairs() const { return !cost_queue_.empty(); }

size_t TrajectoryEvaluator::num_of_pairs() const { return cost_queue_.size(); }

CandidatePair TrajectoryEvaluator::next_top_pair() {
  if (cost_queue_.empty()) return CandidatePair{};
  CandidatePair top = cost_queue_.top();
  cost_queue_.pop();
  return top;
}

double TrajectoryEvaluator::top_pair_cost() const {
  if (cost_queue_.empty()) return 0.0;
  return cost_queue_.top().cost;
}

// Longitudinal objective: normalized mean |v_target - v| (0..1)
double TrajectoryEvaluator::LonObjectiveCost(const LonCandidate& lon, double target_speed) const {
  const double T = lon.T;
  if (T <= 1e-6) return 0.0;

  double v_scale = planner_params_.cost_scales.speed_error_scale;
  if (!(v_scale > 0.0)) v_scale = std::max(1.0, std::fabs(target_speed));

  double speed_err_sum = 0.0;
  double time_weight = 0.0;

  for (double t = 0.0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    const double v = lon.curve->value_evaluation(t, 1);
    const double w = t * t;  // keep original preference: later time weighted more
    speed_err_sum += w * std::fabs(target_speed - v);
    time_weight += w;
  }

  const double mean_err = safe_div(speed_err_sum, time_weight, 0.0);  // m/s
  return clamp01(mean_err / v_scale);
}

// Longitudinal comfort: normalized RMS(jerk) (0..1)
double TrajectoryEvaluator::LonComfortCost(const LonCandidate& lon) const {
  const double T = lon.T;
  if (T <= 1e-6) return 0.0;

  const double dt = planner_params_.rank_dt;
  const double jerk_scale = positive_or(planner_params_.cost_scales.jerk_scale, 2.0);

  double sum_sq = 0.0;
  double sum_t = 0.0;

  for (double t = 0.0; t <= T + 1e-9; t += dt) {
    const double jerk = lon.curve->value_evaluation(t, 3);  // m/s^3
    sum_sq += jerk * jerk * dt;
    sum_t += dt;
  }

  const double jerk_rms = std::sqrt(safe_div(sum_sq, sum_t, 0.0));
  return clamp01(jerk_rms / jerk_scale);
}

// Centripetal accel cost: normalized RMS(v^2 * kappa) (0..1)
double TrajectoryEvaluator::CentripetalAccelerationCost(
    const LonCandidate& lon, const std::shared_ptr<general::FrenetFrame> frenet_frame, double s_offset) const {
  if (!frenet_frame) return 0.0;
  const auto& path = frenet_frame->get_reference_path();
  if (path.empty()) return 0.0;

  const double T = lon.T;
  if (T <= 1e-6) return 0.0;

  const double dt = planner_params_.rank_dt;
  const double a_lat_scale = positive_or(planner_params_.cost_scales.lat_acc_scale, 2.0);

  double sum_sq = 0.0;
  double sum_t = 0.0;

  for (double t = 0.0; t <= T + 1e-9; t += dt) {
    const double s = lon.curve->value_evaluation(t, 0);
    const double v = lon.curve->value_evaluation(t, 1);
    const double s_query = s + s_offset;

    // Interpolate kappa by s_query
    double kappa = 0.0;
    if (s_query <= path.front().accumulated_s) {
      kappa = path.front().kappa;
    } else if (s_query >= path.back().accumulated_s) {
      kappa = path.back().kappa;
    } else {
      auto it = std::lower_bound(path.begin(), path.end(), s_query,
                                 [](const general::PathPoint& pt, double s_val) {
                                   return pt.accumulated_s < s_val;
                                 });
      const size_t idx = static_cast<size_t>(std::distance(path.begin(), it));
      if (idx == 0) {
        kappa = path.front().kappa;
      } else {
        const auto& p0 = path[idx - 1];
        const auto& p1 = path[idx];
        const double ds = p1.accumulated_s - p0.accumulated_s;
        const double t_s = (ds < 1e-12) ? 0.0 : (s_query - p0.accumulated_s) / ds;
        kappa = p0.kappa + t_s * (p1.kappa - p0.kappa);
      }
    }

    const double a_c = v * v * kappa;  // m/s^2
    sum_sq += a_c * a_c * dt;
    sum_t += dt;
  }

  const double a_c_rms = std::sqrt(safe_div(sum_sq, sum_t, 0.0));
  return clamp01(a_c_rms / a_lat_scale);
}

// Progress cost: normalized lack of progress (0..1)
double TrajectoryEvaluator::ProgressCost(const LonCandidate& lon, double target_speed) const {
  const double T = lon.T;
  if (T <= 1e-6) return 0.0;

  const double s_expected = std::max(1e-3, std::fabs(target_speed) * T);
  const double s_end = std::fabs(lon.curve->value_evaluation(T, 0));

  const double lack = std::max(0.0, s_expected - s_end);
  return clamp01(lack / s_expected);
}

// Lateral offset cost: normalized RMS(d) (0..1)
double TrajectoryEvaluator::LatOffsetCost(const LatCandidate& lat, double reference_speed) const {
  const double s_end = lat.param_s;
  if (s_end <= 1e-6) return 0.0;

  // If you have lane-change target, set d_target accordingly.
  const double d_target = 0.0;

  // scales (meters)
  const double d_scale = positive_or(planner_params_.cost_scales.lat_offset_scale, 1.75);
  const double d_end_scale = 0.5;  // tighten (0.3) if you want stronger centering at end

  // sampling step along s
  const double ds =
      std::max(0.1, planner_params_.sampling.sample_time_step * std::max(1.0, std::fabs(reference_speed)));

  double sum_d = 0.0;
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  double count = 0.0;

  for (double s = 0.0; s <= s_end + 1e-9; s += ds) {
    const double d = lat.curve->value_evaluation(s, 0) - d_target;
    sum_d += d;
    sum_abs += std::fabs(d);
    sum_sq += d * d;
    count += 1.0;
  }

  const double mean = safe_div(sum_d, count, 0.0);
  const double mean_abs = safe_div(sum_abs, count, 0.0);
  const double rms = std::sqrt(safe_div(sum_sq, count, 0.0));

  const double J_mean = clamp01(std::fabs(mean) / d_scale);
  const double J_abs = clamp01(mean_abs / d_scale);
  const double J_rms = clamp01(rms / d_scale);
  // endpoint alignment
  const double d_end = lat.curve->value_evaluation(s_end, 0) - d_target;
  const double J_end = clamp01(std::fabs(d_end) / d_end_scale);
  // Combine:
  // - J_abs / J_rms: penalize overall offset (no cancellation)
  // - J_mean: penalize biased offset (centering)
  // - J_end: ensure end close to center
  const double a = 0.0; // overall absolute offset
  const double b = 0.30; // bias (mean) toward center
  const double c = 0.0; // RMS (shape/overall energy)
  const double e = 0.00; // endpoint
  return clamp01(a * J_abs + b * J_mean + c * J_rms + e * J_end);
}

// Lateral comfort: normalized max approx lateral accel (0..1)
double TrajectoryEvaluator::LatComfortCost(const LonCandidate& lon, const LatCandidate& lat) const {
  const double T = lon.T;
  if (T <= 1e-6) return 0.0;

  const double a_lat_scale = positive_or(planner_params_.cost_scales.lat_acc_scale, 2.0);

  double max_a_lat = 0.0;
  for (double t = 0.0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    const double s = lon.curve->value_evaluation(t, 0);
    const double s_dot = lon.curve->value_evaluation(t, 1);
    const double s_ddot = lon.curve->value_evaluation(t, 2);

    const double l_prime = lat.curve->value_evaluation(s, 1);
    const double l_pprime = lat.curve->value_evaluation(s, 2);

    // approx lateral acceleration in Frenet (units ~ m/s^2)
    const double a_lat = std::fabs(l_pprime * s_dot * s_dot + l_prime * s_ddot);
    max_a_lat = std::max(max_a_lat, a_lat);
  }

  return clamp01(max_a_lat / a_lat_scale);
}

// Obstacle distance cost: time-averaged penalty (0..1)
double TrajectoryEvaluator::ObstacleDistanceCost(const std::vector<AD_algorithm::general::TrajectoryPoint>& traj,
                                                double /*start_time*/) const {
  if (!collision_detection_ || traj.size() < 2) return 0.0;

  const double safe = planner_params_.collision.cost_distance_threshold;
  if (!(safe > 0.0) || !std::isfinite(safe)) return 0.0;

  double integral = 0.0;

  for (size_t i = 0; i + 1 < traj.size(); ++i) {
    const auto& pt = traj[i];
    const auto& pt_next = traj[i + 1];

    const double t_abs = pt.time_stamped;
    const double dt = pt_next.time_stamped - pt.time_stamped;
    if (!(dt > 0.0) || !std::isfinite(dt)) continue;

    const double d = collision_detection_->DistanceToObstaclesAtTime(pt, t_abs);
    if (d < safe) {
      const double nd = (safe - d) / safe;  // 0..1
      integral += (nd * nd) * dt;
    }
  }

  const double duration = traj.back().time_stamped - traj.front().time_stamped;
  if (!(duration > 1e-6) || !std::isfinite(duration)) return 0.0;

  return integral / duration;
}

double TrajectoryEvaluator::EvaluatePair(const LonCandidate& lon, const LatCandidate& lat, double target_speed,
                                        const std::shared_ptr<general::FrenetFrame>& frenet_frame, double s_offset,
                                        double start_time) const {
  // all costs are normalized to ~0..1
  const double lon_obj = LonObjectiveCost(lon, target_speed);
  const double lon_jerk = LonComfortCost(lon);
  const double lon_progress = ProgressCost(lon, target_speed);

  // lon_acc: normalized max |a|
  double lon_acc = 0.0;
  {
    const double T = lon.T;
    const double a_scale = positive_or(planner_params_.cost_scales.lon_acc_scale, 2.0);
    double a_max = 0.0;
    for (double t = 0.0; t <= T + 1e-9; t += planner_params_.rank_dt) {
      const double acc = lon.curve->value_evaluation(t, 2);
      a_max = std::max(a_max, std::fabs(acc));
    }
    lon_acc = clamp01(a_max / a_scale);
  }

  const double lat_offset = LatOffsetCost(lat, target_speed);

  // lat_acc: shape term normalized by lat_dd_scale (empirical)
  double lat_acc = 0.0;
  {
    const double max_s = lat.param_s;
    if (max_s > 1e-6) {
      const double dd_scale = positive_or(planner_params_.cost_scales.lat_dd_scale, 0.5);
      const double ds =
          std::max(0.1, planner_params_.sampling.sample_time_step * std::max(1.0, std::fabs(target_speed)));

      double max_dd = 0.0;
      for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
        const double d_dd = lat.curve->value_evaluation(s, 2);
        max_dd = std::max(max_dd, std::fabs(d_dd));
      }
      lat_acc = clamp01(max_dd / dd_scale);
    }
  }

  const double lat_comfort = LatComfortCost(lon, lat);
  const double centripetal_cost = CentripetalAccelerationCost(lon, frenet_frame, s_offset);

  double obstacle_cost = 0.0;
  if (frenet_frame && collision_detection_) {
    auto traj = CombineToCartesian(lon, lat, planner_params_.rank_dt, s_offset, start_time, frenet_frame);
    if (!traj.empty()) {
      obstacle_cost = ObstacleDistanceCost(traj, start_time);
      // std::cout << "[Eval] EvaluatePair: computed obstacle_cost=" << obstacle_cost << " traj_pts=" << traj.size()
      //           << std::endl;
    }
  }

  // std::cout << "lon_obj:" << lon_obj << ", lon_jerk:" << lon_jerk << ", lon_acc:" << lon_acc
  //           << ", lon_progress:" << lon_progress << "\n, lat_offset:" << lat_offset << ", lat_acc:" << lat_acc
  //           << ", lat_comfort:" << lat_comfort << "\n, centripetal_cost:" << centripetal_cost
  //           << ", obstacle_cost:" << obstacle_cost << std::endl;

  const double cost = planner_params_.weights.weight_st_object * lon_obj +
                      planner_params_.weights.weight_st_jerk * lon_jerk +
                      planner_params_.weights.weight_st_acc * lon_acc +
                      planner_params_.weights.weight_Progress * lon_progress +
                      planner_params_.weights.weight_lt_offset * lat_offset +
                      planner_params_.weights.weight_lt_acc * (lat_acc + lat_comfort) +
                      planner_params_.weights.weight_centripetal * centripetal_cost +
                      planner_params_.weights.weight_obstacle * obstacle_cost;

  return cost;
}

void TrajectoryEvaluator::EvaluatePairComponents(const LonCandidate& lon, const LatCandidate& lat, double target_speed,
                                                std::vector<double>& components) const {
  components.clear();
  components.push_back(LonObjectiveCost(lon, target_speed));
  components.push_back(LonComfortCost(lon));

  // lon_acc normalized
  {
    const double T = lon.T;
    const double a_scale = positive_or(planner_params_.cost_scales.lon_acc_scale, 2.0);
    double a_max = 0.0;
    for (double t = 0.0; t <= T + 1e-9; t += planner_params_.rank_dt) {
      const double acc = lon.curve->value_evaluation(t, 2);
      a_max = std::max(a_max, std::fabs(acc));
    }
    components.push_back(clamp01(a_max / a_scale));
  }

  components.push_back(LatOffsetCost(lat, target_speed));

  // lat_acc (shape) normalized
  {
    double lat_acc = 0.0;
    const double max_s = lat.param_s;
    if (max_s > 1e-6) {
      const double dd_scale = positive_or(planner_params_.cost_scales.lat_dd_scale, 0.5);
      const double ds =
          std::max(0.1, planner_params_.sampling.sample_time_step * std::max(1.0, std::fabs(target_speed)));

      double max_dd = 0.0;
      for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
        const double d_dd = lat.curve->value_evaluation(s, 2);
        max_dd = std::max(max_dd, std::fabs(d_dd));
      }
      lat_acc = clamp01(max_dd / dd_scale);
    }
    components.push_back(lat_acc);
  }

  components.push_back(LatComfortCost(lon, lat));
}

std::vector<AD_algorithm::general::TrajectoryPoint> TrajectoryEvaluator::CombineToCartesian(
    const LonCandidate& lon, const LatCandidate& lat, double dt, double s_offset, double start_time,
    const std::shared_ptr<general::FrenetFrame>& frenet_frame) const {
  std::vector<AD_algorithm::general::TrajectoryPoint> trajectory;
  if (!frenet_frame) return trajectory;

  const double T = lon.T;
  if (T <= 0.0) return trajectory;

  for (double t = 0.0; t <= T + 1e-9; t += dt) {
    double s, s_dot, s_ddot, d, d_prime, d_prime_prime;
    try {
      s = lon.curve->value_evaluation(t, 0);
      s_dot = lon.curve->value_evaluation(t, 1);
      s_ddot = lon.curve->value_evaluation(t, 2);

      const double s_query = s;
      const double eps_s = 1e-9;
      if (s_query <= lat.param_s + eps_s) {
        d = lat.curve->value_evaluation(s_query, 0);
        d_prime = lat.curve->value_evaluation(s_query, 1);
        d_prime_prime = lat.curve->value_evaluation(s_query, 2);
      } else {
        const double s_max = lat.param_s;
        const double p = lat.curve->value_evaluation(s_max, 0);
        const double v = lat.curve->value_evaluation(s_max, 1);
        const double a = lat.curve->value_evaluation(s_max, 2);
        const double ds = s_query - s_max;
        d = p + v * ds + 0.5 * a * ds * ds;
        d_prime = v + a * ds;
        d_prime_prime = a;
      }
    } catch (const std::exception&) {
      return {};
    }

    AD_algorithm::general::FrenetPoint fpt;
    fpt.t = t + start_time;
    fpt.s = s + s_offset;
    fpt.s_dot = s_dot;
    fpt.s_dot_dot = s_ddot;
    fpt.l = d;
    fpt.l_dot = d_prime;
    fpt.l_dot_dot = d_prime_prime;

    const double eps = 1e-6;
    if (std::abs(s_dot) > eps) {
      fpt.l_prime = d_prime / s_dot;
      fpt.l_prime_prime = (d_prime_prime * s_dot - d_prime * s_ddot) / (s_dot * s_dot * s_dot);
    } else {
      fpt.l_prime = 0.0;
      fpt.l_prime_prime = 0.0;
    }

    try {
      auto traj_point = frenet_frame->frenet_to_cartesian(fpt);
      traj_point.time_stamped = t + start_time;
      trajectory.push_back(traj_point);
    } catch (const std::exception&) {
      return {};
    }
  }

  return trajectory;
}

void TrajectoryEvaluator::RankPairs(const std::vector<LonCandidate>& lon_candidates,
                                    const std::vector<LatCandidate>& lat_candidates, double target_speed,
                                    const std::shared_ptr<general::FrenetFrame>& frenet_frame, double s_offset,
                                    double start_time) {
  if (collision_detection_) collision_detection_->setParameters(planner_params_);

  while (!cost_queue_.empty()) cost_queue_.pop();

  for (const auto& lon : lon_candidates) {
    for (const auto& lat : lat_candidates) {
      CandidatePair p;
      p.lon = std::make_shared<LonCandidate>(lon);
      p.lat = std::make_shared<LatCandidate>(lat);
      p.cost = EvaluatePair(lon, lat, target_speed, frenet_frame, s_offset, start_time);
      cost_queue_.push(p);
    }
  }
}

}  // namespace planner
}  // namespace AD_algorithm