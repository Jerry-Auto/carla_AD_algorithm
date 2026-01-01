#ifndef LATTICE_TRAJECTORY_EVALUATOR_H_
#define LATTICE_TRAJECTORY_EVALUATOR_H_

#include <vector>
#include <memory>
#include <queue>
#include <functional>

#include "lattice/trajectory1d_generator.h"
#include "lattice/lattice_types.h"
#include "lattice/planner_weights.h"

namespace AD_algorithm {
namespace planner {

struct CandidatePair {
  std::shared_ptr<Trajectory1DGenerator::LonCandidate> lon;
  std::shared_ptr<Trajectory1DGenerator::LatCandidate> lat;
  double cost = 0.0;
};

class CmpPair {
 public:
  bool operator()(const CandidatePair& a, const CandidatePair& b) const {
    return a.cost > b.cost;
  }
};

/**
 * @brief TrajectoryEvaluator（轨迹评估器）
 *
 * 为一维候选轨迹提供轻量级的代价函数与排序。代价项包括：
 * - 纵向速度/目标偏差代价
 * - 纵向 jerk 舒适度
 * - 纵向加速度惩罚（可配置）
 * - 横向偏移惩罚与横向“加速度”代理项
 *
 * 评估器提供 `RankPairs`，返回以最小代价为首的优先队列，用于后续的组合与约束检查。
 */
class TrajectoryEvaluator {
 public:
  TrajectoryEvaluator() = delete;
  /**
   * @param params 包含权重与可调项的结构（例如 weight_st_object、weight_st_jerk、weight_lt_offset 等）
   */
  TrajectoryEvaluator(const PlannerParams& params);

  /**
   * 评估纵向候选代价，返回非负标量。
   */
  double EvaluateLon(const Trajectory1DGenerator::LonCandidate& lon, double target_speed) const;

  /**
   * 评估横向候选代价，返回非负标量（按 s 采样时可作为局部度量）。
   */
  double EvaluateLat(const Trajectory1DGenerator::LatCandidate& lat) const;

  /**
   * 联合评估：根据 lon 与 lat 在空间 s 上的联合代价（包含 LatComfort 依赖 lon）
   */
  double EvaluatePair(const Trajectory1DGenerator::LonCandidate& lon,
                      const Trajectory1DGenerator::LatCandidate& lat,
                      double target_speed) const;

  // helper: find time t such that lon.curve->value_evaluation(t,0) ~= s (binary search)
  double FindTimeForS(const Trajectory1DGenerator::LonCandidate& lon, double s, double tol=1e-3) const;

  /**
   * 生成按代价排序的候选对（最小代价优先）。
   */
  std::priority_queue<CandidatePair, std::vector<CandidatePair>, CmpPair> RankPairs(
      const std::vector<Trajectory1DGenerator::LonCandidate>& lon_candidates,
      const std::vector<Trajectory1DGenerator::LatCandidate>& lat_candidates,
      double target_speed) const;

 private:
  PlannerParams planner_params_;
  double weight_st_object_;
  double weight_st_jerk_;
  double weight_lt_offset_;
  double weight_lt_acc_;
  double weight_st_acc_;
  // 障碍距离代价权重
  double weight_obstacle_distance_;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TRAJECTORY_EVALUATOR_H_
