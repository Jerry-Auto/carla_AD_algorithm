
#include "lattice/trajectory_evaluator.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include "lattice/lattice_types.h"

namespace AD_algorithm {
namespace planner {

TrajectoryEvaluator::TrajectoryEvaluator(const PlannerParams& params,const std::shared_ptr<latticeCollisionDetection>& CollisionDetection)
    : collision_detection_(CollisionDetection),
      planner_params_(params) 
      {}

bool TrajectoryEvaluator::has_more_pairs() const {
  return !cost_queue_.empty();
}

size_t TrajectoryEvaluator::num_of_pairs() const {
  return cost_queue_.size();
}

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

// 纵向目标/速度偏差代价
double TrajectoryEvaluator::LonObjectiveCost(const LonCandidate& lon, double target_speed) const {
  double T = lon.T;
  double speed_err_sum = 0.0;
  double time_weight = 0.0;
  for (double t=0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    double v = lon.curve->value_evaluation(t, 1);
    double w = t * t;
    speed_err_sum += w * std::fabs(target_speed - v);
    time_weight += w;
  }
  return speed_err_sum / (time_weight + 1e-9);
}

// 纵向jerk舒适度
double TrajectoryEvaluator::LonComfortCost(const LonCandidate& lon) const {
  double T = lon.T;
  double jerk_sq_sum = 0.0;
  double jerk_abs_sum = 1e-6;
  for (double t = 0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    double jerk = lon.curve->value_evaluation(t, 3);
    jerk_sq_sum += std::pow(jerk / 4.0, 2);
    jerk_abs_sum += std::fabs(jerk / 4.0);
  }
  return jerk_sq_sum / jerk_abs_sum;
}

// 向心加速度代价：采样时间区间，使用参考线曲率（插值）和纵向速度计算 a_c = v^2 * kappa，
// 按官方实现风格返回 sum(a_c^2) / (sum(|a_c|) + eps)
double TrajectoryEvaluator::CentripetalAccelerationCost(const LonCandidate& lon,const std::shared_ptr<general::FrenetFrame> frenet_frame,double s_offset) const {
  if (!frenet_frame) return 0.0;
  const auto& path = frenet_frame->get_reference_path();
  if (path.empty()) return 0.0;

  double T = lon.T;
  const double dt = planner_params_.rank_dt;
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  const double eps = 1e-9;
  auto ref_line = frenet_frame->get_reference_line();
  for (double t = 0.0; t <= T + 1e-9; t += dt) {
    double s = lon.curve->value_evaluation(t, 0);
    double v = lon.curve->value_evaluation(t, 1);
    // interpolate kappa on reference path by s 
        double s_query = s + s_offset;
    double kappa = 0.0;
    if (s_query <= path.front().accumulated_s) {
      kappa = path.front().kappa;
    } else if (s_query >= path.back().accumulated_s) {
      kappa = path.back().kappa;
    } else {
      auto it = std::lower_bound(path.begin(), path.end(), s_query,
          [](const general::PathPoint& pt, double s_val) { return pt.accumulated_s < s_val; });
      size_t idx = std::distance(path.begin(), it);
      if (idx == 0) {
        kappa = path.front().kappa;
      } else {
        const auto& p0 = path[idx - 1];
        const auto& p1 = path[idx];
        double ds = p1.accumulated_s - p0.accumulated_s;
        double t_s = (ds < 1e-12) ? 0.0 : (s_query - p0.accumulated_s) / ds;
        kappa = p0.kappa + t_s * (p1.kappa - p0.kappa);
      }
    }

    double a_c = v * v * kappa;
    sum_abs += std::fabs(a_c);
    sum_sq += a_c * a_c;
  }
  return sum_sq / (sum_abs + eps);
}

double TrajectoryEvaluator::ProgressCost(const LonCandidate& lon, double target_speed) const {
  // 预期在时间 T 内走的距离
  double T = lon.T;
  if (T <= 1e-6) return 0.0;
  double s_expected = target_speed * T;
  double s_end = lon.curve->value_evaluation(T, 0);
  return 100/std::pow(s_end, 4);
}

// 横向偏移代价
double TrajectoryEvaluator::LatOffsetCost(const LatCandidate& lat, double reference_speed) const {
  double max_s = lat.param_s;
  double eps = 1e-6;
  if (max_s <= eps) return 0.0;
  double ds = std::max(0.1, planner_params_.sampling.sample_time_step * reference_speed);
  double offset_sq_sum = 0.0;
  double offset_abs_sum = eps;
  for (double s = 0.0; s <= max_s + eps; s += ds) {
    double d = lat.curve->value_evaluation(s, 0);
    offset_sq_sum += std::pow(d, 2);
  }
  return offset_sq_sum;
}

// 横向舒适度
double TrajectoryEvaluator::LatComfortCost(const LonCandidate& lon, const LatCandidate& lat) const {
  double T = lon.T;
  double max_cost = 0.0;
  for (double t = 0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    double s = lon.curve->value_evaluation(t, 0);
    double s_dot = lon.curve->value_evaluation(t, 1);
    double s_ddot = lon.curve->value_evaluation(t,2);
    double l_prime = lat.curve->value_evaluation(s, 1);
    double l_pprime = lat.curve->value_evaluation(s, 2);
    double comfort = std::fabs(l_pprime * s_dot * s_dot + l_prime * s_ddot);
    if (comfort > max_cost) max_cost = comfort;
  }
  return max_cost;
}

// 综合评价，包含所有分量
double TrajectoryEvaluator::EvaluatePair(const LonCandidate& lon, const LatCandidate& lat, double target_speed,
                                           const std::shared_ptr<general::FrenetFrame>& frenet_frame,
                                           double s_offset,
                                           double start_time) const {
  double lon_obj = LonObjectiveCost(lon, target_speed);
  double lon_jerk = LonComfortCost(lon);
  double lon_acc = 0.0;
  double lon_progress = ProgressCost(lon, target_speed);
  double T = lon.T;
  for (double t = 0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    double acc = lon.curve->value_evaluation(t, 2);
    if (std::fabs(acc) > std::fabs(lon_acc)) lon_acc = acc;
  }
  lon_acc = std::fabs(lon_acc);
  double lat_offset = LatOffsetCost(lat, target_speed);
  double lat_acc = 0.0;
  double max_s = lat.param_s;
  if (max_s > 1e-6) {
    double ds = std::max(0.1, planner_params_.sampling.sample_time_step * target_speed);
    for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
      double d_dd = lat.curve->value_evaluation(s, 2);
      if (std::fabs(d_dd) > std::fabs(lat_acc)) lat_acc = d_dd;
    }
    lat_acc = std::fabs(lat_acc);
  }
  double lat_comfort = LatComfortCost(lon, lat);
  double centripetal_cost = CentripetalAccelerationCost(lon, frenet_frame, s_offset);
  double obstacle_cost=0.0;
  // 额外：基于笛卡尔化轨迹的代价（障碍物距离、轨迹长度）和向心加速度
  if (frenet_frame && collision_detection_) {
    auto traj = CombineToCartesian(lon, lat, planner_params_.rank_dt, s_offset, start_time, frenet_frame);
    if (!traj.empty()) {
      obstacle_cost = ObstacleDistanceCost(traj, start_time);
      std::cout << "[Eval] EvaluatePair: computed obstacle_cost=" << obstacle_cost << " traj_pts=" << traj.size() << std::endl;
    }
  }
  std::cout<<"lon_obj:"<<lon_obj<<", lon_jerk:"<<lon_jerk<<", lon_acc:"<<lon_acc<<", lon_progress:"<<lon_progress
           <<"\n, lat_offset:"<<lat_offset<<", lat_acc:"<<lat_acc<<", lat_comfort:"<<lat_comfort
           <<"\n, centripetal_cost:"<<centripetal_cost<<", obstacle_cost:"<<obstacle_cost<<std::endl;
  // 基础代价
  double cost = planner_params_.weights.weight_st_object * lon_obj +
         planner_params_.weights.weight_st_jerk * lon_jerk +
         planner_params_.weights.weight_st_acc * lon_acc +
         planner_params_.weights.weight_lt_offset * lat_offset +
         planner_params_.weights.weight_lt_acc * (lat_acc + lat_comfort)+
         planner_params_.weights.weight_Progress * lon_progress+
         planner_params_.weights.weight_centripetal * centripetal_cost+
         planner_params_.weights.weight_obstacle * obstacle_cost;
        
  return cost;
}

// 可选：输出各分量代价
void TrajectoryEvaluator::EvaluatePairComponents(const LonCandidate& lon, const LatCandidate& lat, double target_speed, std::vector<double>& components) const {
  components.clear();
  components.push_back(LonObjectiveCost(lon, target_speed));
  components.push_back(LonComfortCost(lon));
  // 纵向最大加速度
  double T = lon.T;
  double lon_acc = 0.0;
  for (double t = 0; t <= T + 1e-9; t += planner_params_.rank_dt) {
    double acc = lon.curve->value_evaluation(t, 2);
    if (std::fabs(acc) > std::fabs(lon_acc)) lon_acc = acc;
  }
  components.push_back(std::fabs(lon_acc));
  components.push_back(LatOffsetCost(lat, target_speed));
  // 横向最大加速度（使用 target_speed 估计采样步长）
  double max_s = lat.param_s;
  double lat_acc = 0.0;
  if (max_s > 1e-6) {
    double ds = std::max(0.1, planner_params_.sampling.sample_time_step * target_speed);
    for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
      double d_dd = lat.curve->value_evaluation(s, 2);
      if (std::fabs(d_dd) > std::fabs(lat_acc)) lat_acc = d_dd;
    }
  }
  components.push_back(std::fabs(lat_acc));
  components.push_back(LatComfortCost(lon, lat));
}


std::vector<AD_algorithm::general::TrajectoryPoint> TrajectoryEvaluator::CombineToCartesian(const LonCandidate& lon, const LatCandidate& lat,
                                                                                             double dt, double s_offset,double start_time,
                                                                                             const std::shared_ptr<general::FrenetFrame>& frenet_frame) const {
  std::vector<AD_algorithm::general::TrajectoryPoint> trajectory;
  if (!frenet_frame) return trajectory;
  double T = lon.T;
  if (T <= 0.0) return trajectory;

  for (double t = 0.0; t <= T + 1e-9; t += dt) {
    double s, s_dot, s_ddot, d, d_prime, d_prime_prime;
    try {
      s = lon.curve->value_evaluation(t, 0);
      s_dot = lon.curve->value_evaluation(t, 1);
      s_ddot = lon.curve->value_evaluation(t, 2);

      double s_query = s;
      const double eps_s = 1e-9;
      if (s_query <= lat.param_s + eps_s) {
        d = lat.curve->value_evaluation(s_query, 0);
        d_prime = lat.curve->value_evaluation(s_query, 1);
        d_prime_prime = lat.curve->value_evaluation(s_query, 2);
      } else {
        double s_max = lat.param_s;
        double p = lat.curve->value_evaluation(s_max, 0);
        double v = lat.curve->value_evaluation(s_max, 1);
        double a = lat.curve->value_evaluation(s_max, 2);
        double ds = s_query - s_max;
        d = p + v * ds + 0.5 * a * ds * ds;
        d_prime = v + a * ds;
        d_prime_prime = a;
      }
    } catch (const std::exception& e) {
      return {};
    }

    AD_algorithm::general::FrenetPoint fpt;
    fpt.t = t+ start_time;
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
      traj_point.time_stamped = t + start_time; // 绝对时间
      trajectory.push_back(traj_point);
    } catch (const std::exception& e) {
      return {};
    }
  }

  return trajectory;
}


void TrajectoryEvaluator::RankPairs(const std::vector<LonCandidate>& lon_candidates,
                  const std::vector<LatCandidate>& lat_candidates,
                  double target_speed,
                  const std::shared_ptr<general::FrenetFrame>& frenet_frame,
                  double s_offset,
                  double start_time) {
  // Ensure collision detector uses current params if provided
  if (collision_detection_) collision_detection_->setParameters(planner_params_);

  // 清空队列
  while (!cost_queue_.empty()) cost_queue_.pop();
  for (const auto& lon : lon_candidates) {
    for (const auto& lat : lat_candidates) {
      CandidatePair p;
      p.lon = std::make_shared<LonCandidate>(lon);
      p.lat = std::make_shared<LatCandidate>(lat);

      // 基础代价（不包含障碍物）
      // 将所有代价合并到 EvaluatePair 中（包含障碍物距离 / 轨迹长度 / 向心加速度）
      p.cost = EvaluatePair(lon, lat, target_speed, frenet_frame, s_offset, start_time);
      cost_queue_.push(p);
    }
  }
}

double TrajectoryEvaluator::ObstacleDistanceCost(const std::vector<AD_algorithm::general::TrajectoryPoint>& traj, double start_time) const {
  if (!collision_detection_ || traj.empty()) return 0.0;
  const double safe = planner_params_.collision.cost_distance_threshold;
  double sum_nd_sq = 0.0;
  double count = 0.0;
  for (const auto& pt : traj) {
    count += 1.0;
    double t_abs = pt.time_stamped; // absolute time from combined trajectory
    double d = collision_detection_->DistanceToObstaclesAtTime(pt, t_abs);
    std::cout << "[Eval] ObstacleDistanceCost: pt.time=" << t_abs << ", pos=(" << pt.x << "," << pt.y << "), d=" << d << std::endl;
    if (d < safe) {
      double nd = (safe - d) / safe; // normalized (0..1)
      sum_nd_sq += nd * nd;
    }
  }
  if (sum_nd_sq <= 0.0) return 0.0;
  double duration = traj.back().time_stamped - traj.front().time_stamped;
  if (duration <= 1e-6) duration = static_cast<double>(traj.size()) * planner_params_.rank_dt;
  return sum_nd_sq / duration;
}

} // namespace planner
} // namespace AD_algorithm
