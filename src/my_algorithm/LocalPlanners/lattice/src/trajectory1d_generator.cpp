#include "lattice/trajectory1d_generator.h"

#include <algorithm>
#include <cmath>

namespace AD_algorithm {
namespace planner {

Trajectory1DGenerator::Trajectory1DGenerator(const PlannerParams& params) : params_(params) {}

std::vector<LonCandidate>
Trajectory1DGenerator::GenerateLongitudinalCruising(const latticeFrenetPoint& init, double reference_speed) const {
  std::vector<LonCandidate> candidates;
  for (double T = params_.sampling.sample_min_time; T <= params_.sampling.sample_max_time; T += params_.sampling.sample_time_step) {
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // 巡航情形：不强制终点 s，而是约束终点速度（四次多项式），更自然地表达“到达目标速度”的目标
    double target_v = reference_speed;
    bool ok = st_polynomial->curve_fitting(
        0.0, init.s, init.s_dot, 0.0,
        T, target_v, 0.0);
    if (!ok) continue;
    LonCandidate c;
    c.curve = st_polynomial;
    c.T = T;
    candidates.push_back(c);
  }
  return candidates;
}

std::vector<LonCandidate>
Trajectory1DGenerator::GenerateLongitudinalFollowing(const latticeFrenetPoint& init,
                                                    const latticeFrenetPoint& leader,
                                                    double reference_speed) const {
  std::vector<LonCandidate> candidates;
  for (double T = params_.sampling.sample_min_time; T <= params_.sampling.sample_max_time; T += params_.sampling.sample_time_step) {
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // 结束 s 取为 leader.s 减去缓冲距离（用于保持安全跟车距离）
    double target_s = leader.s - 8.0;
    // 防止目标 s 为负，导致向后运动的速度曲线
    if (target_s < 0.0) {
      target_s = 0.0;
    }
    double target_v = std::min(reference_speed, leader.v);
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

std::vector<LatCandidate>
Trajectory1DGenerator::GenerateLateralCandidates(const latticeFrenetPoint& init, double T, double reference_speed) const {
  std::vector<LatCandidate> candidates;
  // 参考 Apollo 的实现：横向曲线以纵向弧长 s 为参数（d(s)），因此在若干 end_s 候选上采样
  std::vector<double> end_s_candidates = {10.0, 80.0};
  // 同时加入基于参考速度的估计终点 s（使用传入的 reference_speed）
  double est_end_s = reference_speed * T;

  auto sample_for_end_s = [&](double end_s) {
    for (double l = -params_.sampling.sample_lat_width; l <= params_.sampling.sample_lat_width; l += params_.sampling.sample_width_length) {
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
