#ifndef LATTICE_TRAJECTORY1D_GENERATOR_H_
#define LATTICE_TRAJECTORY1D_GENERATOR_H_

#include <vector>
#include <memory>
#include <unordered_map>

#include "lattice/lattice_types.h"
#include "general_modules/polynomial_curve.h"
#include "lattice/planner_weights.h"

namespace AD_algorithm {
namespace planner {

/**
 * @brief Trajectory1DGenerator
 *
 * 生成一维纵向与横向候选轨迹。
 * - 纵向候选以时间为参数：t -> s(t)
 * - 横向候选以时间或参数为参数：d(t)（在某些用法中可视为 d(s)）
 *
 * 本生成器使用已有的 `general::PolynomialCurve` 进行四阶/五阶多项式拟合，
 * 并提供 lattice规划器所需的简单采样策略。
 */
class Trajectory1DGenerator {
 public:
  /**
   * LonCandidate
   * - curve: 映射时间到s的多项式
   * - T: 候选轨迹的持续时间
   */
  struct LonCandidate {
    std::shared_ptr<AD_algorithm::general::PolynomialCurve> curve;
    double T = 0.0; // 持续时间
  };

  struct LatCandidate {
    std::shared_ptr<AD_algorithm::general::PolynomialCurve> curve;
    double T = 0.0; // 与纵向候选匹配的持续时间
    double param_s = 0.0; // 横向多项式的参数长度（近似）
  };

  Trajectory1DGenerator() = delete;
  explicit Trajectory1DGenerator(const PlannerParams& params);

  // 纵向候选（巡航）：无严格终点位置
  std::vector<LonCandidate> GenerateLongitudinalCruising(const latticeFrenetPoint& init) const;
  // 纵向候选（跟车）：终点接近 leader.s（带缓冲）
  std::vector<LonCandidate> GenerateLongitudinalFollowing(const latticeFrenetPoint& init,
                                                           const latticeFrenetPoint& leader) const;

  // 横向候选：横向曲线以纵向弧长 s 为参数（d(s)），在若干 end_s 候选与横偏采样组合生成候选集
  std::vector<LatCandidate> GenerateLateralCandidates(const latticeFrenetPoint& init, double T) const;

 private:
  // 采样参数（与 lattice_planner 的风格一致）
  double sample_max_time_;
  double sample_min_time_;
  double sample_time_step_;
  double sample_lat_width_;
  double sample_width_length_;
  double cruise_speed_;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TRAJECTORY1D_GENERATOR_H_
