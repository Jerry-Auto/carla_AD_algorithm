#include "lattice/trajectory1d_generator.h"

#include <algorithm>
#include <cmath>

namespace AD_algorithm {
namespace planner {

Trajectory1DGenerator::Trajectory1DGenerator(const PlannerParams& params) {
  cruise_speed_ = params.cruise_speed;
  sample_max_time_ = params.sampling.sample_max_time;
  sample_min_time_ = params.sampling.sample_min_time;
  sample_time_step_ = params.sampling.sample_time_step;
  sample_lat_width_ = params.sampling.sample_lat_width;
  sample_width_length_ = params.sampling.sample_width_length;
}

std::vector<Trajectory1DGenerator::LonCandidate>
Trajectory1DGenerator::GenerateLongitudinalCruising(const latticeFrenetPoint& init) const {
  std::vector<LonCandidate> candidates;
  for (double T = sample_min_time_; T <= sample_max_time_; T += sample_time_step_) {
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // 巡航情形：不强制终点 s，而是约束终点速度（四次多项式），更自然地表达“到达目标速度”的目标
    bool ok = st_polynomial->curve_fitting(
        0.0, init.s, init.s_dot, 0.0,
        T, cruise_speed_, 0.0);    if (!ok) continue;
    LonCandidate c;
    c.curve = st_polynomial;
    c.T = T;
    candidates.push_back(c);
  }
  return candidates;
}

std::vector<Trajectory1DGenerator::LonCandidate>
Trajectory1DGenerator::GenerateLongitudinalFollowing(const latticeFrenetPoint& init,
                                                      const latticeFrenetPoint& leader) const {
  std::vector<LonCandidate> candidates;
  for (double T = sample_min_time_; T <= sample_max_time_; T += sample_time_step_) {
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // 结束 s 取为 leader.s 减去缓冲距离（用于保持安全跟车距离）
    double target_s = leader.s - 8.0;
    double target_v = std::min(cruise_speed_, leader.v);
    bool ok = st_polynomial->curve_fitting(
        0.0, init.s, init.s_dot, 0.0,
        T, target_s, target_v, 0.0);
    if (!ok) continue;
    LonCandidate c;
    c.curve = st_polynomial;
    c.T = T;
    candidates.push_back(c);
  }
  return candidates;
}

std::vector<Trajectory1DGenerator::LatCandidate>
Trajectory1DGenerator::GenerateLateralCandidates(const latticeFrenetPoint& init, double T) const {
  std::vector<LatCandidate> candidates;
  // 参考 Apollo 的实现：横向曲线以纵向弧长 s 为参数（d(s)），因此在若干 end_s 候选上采样
  std::vector<double> end_s_candidates = {10.0, 20.0, 40.0, 80.0};
  // 同时加入基于预计巡航速度的估计终点 s
  double est_end_s = cruise_speed_ * T;

  auto sample_for_end_s = [&](double end_s) {
    for (double l = -sample_lat_width_; l <= sample_lat_width_; l += sample_width_length_) {
      auto lt_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
      // 注意：横向多项式以 s 为自变量，初始空间导数直接使用 init 中的 l_prime / l_prime_prime
      double l_prime = init.l_prime;
      double l_pprime = init.l_prime_prime;
      bool ok = lt_polynomial->curve_fitting(
          0.0, init.l, l_prime, l_pprime,
          end_s, l, 0.0, 0.0);
      if (!ok) continue;
      LatCandidate c;
      c.curve = lt_polynomial;
      c.T = T;           // 仍保留与纵向候选相匹配的持续时间
      c.param_s = end_s; // 横向多项式以 s 为参数，记录真实的 param 长度
      candidates.push_back(c);
    }
  };

  for (double end_s : end_s_candidates) sample_for_end_s(end_s);
  // 若估计的 end_s 不在默认集合中，则也进行采样
  bool found = false;
  for (double v : end_s_candidates) if (std::abs(v - est_end_s) < 1e-6) { found = true; break; }
  if (!found && est_end_s > 0.1) sample_for_end_s(est_end_s);
  return candidates;
}

} // namespace planner
} // namespace AD_algorithm
