#include "lattice/trajectory_evaluator.h"
#include <cmath>

namespace AD_algorithm {
namespace planner {

TrajectoryEvaluator::TrajectoryEvaluator(const PlannerParams& params) {
  planner_params_ = params;
  weight_st_object_ = params.weights.weight_st_object;
  weight_st_jerk_ = params.weights.weight_st_jerk;
  weight_lt_offset_ = params.weights.weight_lt_offset;
  weight_lt_acc_ = params.weights.weight_lt_acc;
  weight_st_acc_ = params.weights.weight_st_acc;
  weight_obstacle_distance_ = params.weights.weight_obstacle_distance;
}

// 评估纵向代价：速度偏差 + jerk 舒适度 + 加速度惩罚
double TrajectoryEvaluator::EvaluateLon(const Trajectory1DGenerator::LonCandidate& lon, double target_speed) const {
  // 按时间采样以计算平均速度误差与 jerk
  double T = lon.T;
  int steps = std::max(1, static_cast<int>(T / 0.02));
  double speed_err_sum = 0.0;
  double time_weight = 0.0;
  double jerk_sq_sum = 0.0;
  double jerk_abs_sum = 1e-6;
  double max_acc = 0.0;
  for (int i = 0; i <= steps; ++i) {
    double t = T * i / steps;
    double v = lon.curve->value_evaluation(t, 1);
    double jerk = lon.curve->value_evaluation(t, 3);
    double acc = lon.curve->value_evaluation(t, 2);
    double w = t * t;
    speed_err_sum += w * fabs(target_speed - v);
    time_weight += w;
    jerk_sq_sum += pow(jerk / 4.0, 2);
    jerk_abs_sum += fabs(jerk / 4.0);
    if (fabs(acc) > fabs(max_acc)) max_acc = acc;
  }
  double speed_cost = speed_err_sum / (time_weight + 1e-9);
  double jerk_cost = 0.0;
  if (jerk_abs_sum > 1e-6) jerk_cost = jerk_sq_sum / jerk_abs_sum;
  double acc_cost = fabs(max_acc);
  return weight_st_object_ * speed_cost + weight_st_jerk_ * jerk_cost + weight_st_acc_ * acc_cost;
}

// 评估横向代价：偏离量（基于 s 采样）
double TrajectoryEvaluator::EvaluateLat(const Trajectory1DGenerator::LatCandidate& lat) const {
  double max_s = lat.param_s;
  if (max_s <= 1e-6) return 0.0;
  double ds = std::max(0.1, planner_params_.sampling.sample_dt * planner_params_.cruise_speed); // 与 Filter 中一致
  double offset_sq_sum = 0.0;
  double offset_abs_sum = 1e-6;
  double max_acc = 0.0;
  for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
    double d = lat.curve->value_evaluation(s, 0);
    double d_dd = lat.curve->value_evaluation(s, 2);
    offset_sq_sum += pow(d / 3.5, 2);
    offset_abs_sum += fabs(d / 3.5);
    if (fabs(d_dd) > fabs(max_acc)) max_acc = d_dd;
  }
  double offset_cost = offset_sq_sum / offset_abs_sum;
  double acc_cost = fabs(max_acc);
  return weight_lt_offset_ * offset_cost + weight_lt_acc_ * acc_cost;
}

// 联合评估：按 s 采样，计算 LatComfort 依赖于 lon 的 s_dot/s_ddot
// 返回联合代价（lon 部分未包含在此函数中）
double TrajectoryEvaluator::EvaluatePair(const Trajectory1DGenerator::LonCandidate& lon,
                                        const Trajectory1DGenerator::LatCandidate& lat,
                                        double target_speed) const {
  // 基础 lon/lat 单项代价
  double lon_cost = EvaluateLon(lon, target_speed);
  double lat_cost = EvaluateLat(lat);

  // 联合耦合项（LatComfort）——按照空间 s 采样并使用 lon 的 s_dot/s_ddot
  double max_s = std::min(lat.param_s, lon.curve->value_evaluation(lon.T, 0));
  if (max_s <= 1e-6) return lon_cost + lat_cost;
  double ds = std::max(planner_params_.sampling.sample_space_resolution, planner_params_.sampling.sample_dt * planner_params_.cruise_speed);

  double lat_comfort_sum = 0.0;
  int cnt = 0;
  for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
    // 求解 t 使得 lon.curve->value_evaluation(t,0) ~= s（使用二分法，精确映射）
    double t = FindTimeForS(lon, s, 1e-3);
    double s_dot = lon.curve->value_evaluation(t, 1);
    double s_ddot = lon.curve->value_evaluation(t, 2);

    double l_prime = lat.curve->value_evaluation(s, 1); // dl/ds
    double l_pprime = lat.curve->value_evaluation(s, 2); // d2l/ds2
    // LatComfort: |l'' * s_dot^2 + l' * s_ddot|
    double comfort = fabs(l_pprime * s_dot * s_dot + l_prime * s_ddot);
    lat_comfort_sum += comfort;
    ++cnt;
  }

  double lat_comfort = (cnt > 0) ? (lat_comfort_sum / cnt) : 0.0;

  return lon_cost + lat_cost + weight_lt_acc_ * lat_comfort;
}

// 在 lon 曲线上通过二分搜索求解 t 使 s(t) = s_target（假设 s(t) 单调递增）
double TrajectoryEvaluator::FindTimeForS(const Trajectory1DGenerator::LonCandidate& lon, double s_target, double tol) const {
  double lo = 0.0;
  double hi = lon.T;
  double s_lo = lon.curve->value_evaluation(lo, 0);
  double s_hi = lon.curve->value_evaluation(hi, 0);
  if (s_target <= s_lo) return lo;
  if (s_target >= s_hi) return hi;
  for (int it = 0; it < 50; ++it) {
    double mid = 0.5 * (lo + hi);
    double s_mid = lon.curve->value_evaluation(mid, 0);
    double err = s_mid - s_target;
    if (fabs(err) <= tol) return mid;
    if (err < 0) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

std::priority_queue<CandidatePair, std::vector<CandidatePair>, CmpPair>
TrajectoryEvaluator::RankPairs(
    const std::vector<Trajectory1DGenerator::LonCandidate>& lon_candidates,
    const std::vector<Trajectory1DGenerator::LatCandidate>& lat_candidates,
    double target_speed) const {
  std::priority_queue<CandidatePair, std::vector<CandidatePair>, CmpPair> q;
  // 使用 Beam 搜索：对每个 lon 仅评估 top-K 个 lat（按独立 lat_cost 排序）以减少计算量
  const size_t K = planner_params_.sampling.beam_size; // 从配置读取 beam 大小
  // 预计算 lat 单项代价并排序
  std::vector<std::pair<double, Trajectory1DGenerator::LatCandidate>> lat_costs;
  lat_costs.reserve(lat_candidates.size());
  for (const auto& lat : lat_candidates) {
    lat_costs.emplace_back(EvaluateLat(lat), lat);
  }
  std::sort(lat_costs.begin(), lat_costs.end(), [](const auto& a, const auto& b){ return a.first < b.first; });

  for (const auto& lon : lon_candidates) {
    // 对 lon，选择 top-K lat 做联合评估
    size_t limit = std::min(K, lat_costs.size());
    for (size_t i = 0; i < limit; ++i) {
      const auto& lat = lat_costs[i].second;
      CandidatePair p;
      p.lon = std::make_shared<Trajectory1DGenerator::LonCandidate>(lon);
      p.lat = std::make_shared<Trajectory1DGenerator::LatCandidate>(lat);
      p.cost = EvaluatePair(lon, lat, target_speed);
      q.push(p);
    }
  }
  return q;
}

} // namespace planner
} // namespace AD_algorithm
