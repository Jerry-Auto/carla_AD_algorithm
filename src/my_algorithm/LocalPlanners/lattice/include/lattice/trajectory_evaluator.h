#ifndef LATTICE_TRAJECTORY_EVALUATOR_H_
#define LATTICE_TRAJECTORY_EVALUATOR_H_

#include <vector>
#include <memory>
#include <queue>
#include <functional>

#include "lattice/trajectory1d_generator.h"
#include "lattice/lattice_types.h"
#include "lattice/planner_weights.h"
#include "general_modules/FrenetFrame.h"
#include "lattice/lattice_collision_detection.h"

namespace AD_algorithm {
namespace planner {


class TrajectoryEvaluator {
 public:
  TrajectoryEvaluator() = delete;
  explicit TrajectoryEvaluator(const PlannerParams& params,const std::shared_ptr<latticeCollisionDetection>& CollisionDetection);
  ~TrajectoryEvaluator() = default;

  // 轨迹对队列接口
  bool has_more_pairs() const;
  size_t num_of_pairs() const;
  CandidatePair next_top_pair();
  double top_pair_cost() const;



  // 生成按代价排序的候选对队列
  void RankPairs(const std::vector<LonCandidate>& lon_candidates,
                 const std::vector<LatCandidate>& lat_candidates,
                 double target_speed,
                 const std::shared_ptr<general::FrenetFrame>& frenet_frame,
                 double s_offset,
                 double start_time);

  // 组合一维候选为笛卡尔轨迹（返回的 time_stamped 为绝对时间
  std::vector<AD_algorithm::general::TrajectoryPoint> CombineToCartesian(const LonCandidate& lon, const LatCandidate& lat,
                                                                         double dt, double s_offset,double start_time,
                                                                         const std::shared_ptr<general::FrenetFrame>& frenet_frame) const;


  // 可选：输出各分量代价
  void EvaluatePairComponents(const LonCandidate& lon, const LatCandidate& lat, double target_speed, std::vector<double>& components) const;

 private:
  // 评价与排序
  // 纵向目标/速度偏差代价
  double LonObjectiveCost(const LonCandidate& lon, double target_speed) const;
  // 纵向jerk舒适度
  double LonComfortCost(const LonCandidate& lon) const;
  // 向心加速度代价
  double CentripetalAccelerationCost(const LonCandidate& lon,const std::shared_ptr<general::FrenetFrame> frenet_frame,double s_offset) const;
  // 横向偏移代价（使用参考速度计算采样步长）
  double LatOffsetCost(const LatCandidate& lat, double reference_speed) const;
  // 横向舒适度
  double LatComfortCost(const LonCandidate& lon, const LatCandidate& lat) const;
  // 计算给定笛卡尔轨迹相对于预测障碍物的距离惩罚（基于 planner_params_.collision.safe_distance）
  double ObstacleDistanceCost(const std::vector<AD_algorithm::general::TrajectoryPoint>& traj, double start_time) const;

  double ProgressCost(const LonCandidate& lon, double target_speed) const ;

  // 综合评价，包含以上所有分量
  double EvaluatePair(const LonCandidate& lon, const LatCandidate& lat, double target_speed,
                      const std::shared_ptr<general::FrenetFrame>& frenet_frame,
                      double s_offset,
                      double start_time) const;

  PlannerParams planner_params_;
  std::shared_ptr<latticeCollisionDetection> collision_detection_;
  std::priority_queue<CandidatePair, std::vector<CandidatePair>, CmpPair> cost_queue_;
  std::shared_ptr<general::FrenetFrame> global_frenet_frame_;

};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TRAJECTORY_EVALUATOR_H_
