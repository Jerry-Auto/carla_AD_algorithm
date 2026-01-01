#ifndef LATTICE_PLANNER_WEIGHTS_H
#define LATTICE_PLANNER_WEIGHTS_H

namespace AD_algorithm {
namespace planner {

// 采样相关的可调项
struct SamplingParams {
  double sample_max_time = 3.0;
  double sample_min_time = 1.0;
  double sample_time_step = 1.0;
  double sample_lat_width = 8.0;  // 横向采样范围（单位 m），左右对称
  double sample_width_length = 0.5;  // 横向采样间隔（单位 m）
  double sample_space_resolution = 0.5; // 计算代价时横向空间(s)采样步长，单位 m（默认 0.1）
  double sample_dt = 0.02; // 用于轨迹采样与校验的时间步长（新增）
  size_t beam_size = 5; // 在 RankPairs 中对每个 lon 考虑的 top-K lat（beam 大小）
};

// 代价 / 权重 可调项
struct WeightParams {
  double weight_st_object = 1.0;
  double weight_st_jerk = 1.0;
  double weight_lt_offset = 1.0;
  double weight_lt_acc = 1.0;
  double weight_st_acc = 0.5; // 对纵向加速度的额外惩罚
  double weight_obstacle_distance = 1.0; // 障碍距离惩罚权重（越大越偏向远离障碍）
};

// 物理限制与约束
struct LimitParams {
  double max_speed = 120.0 / 3.6;
  double max_acc = 8.0;
  double max_curvature = 100.0;
  double max_jerk = 10.0;
};

// 规划器参数集合（采样 / 权重 / 限制）
struct PlannerParams {
  SamplingParams sampling;
  WeightParams weights;
  LimitParams limits;
  double cruise_speed = 8.0;
  double plan_time_step = 0.2; // 相邻规划周期之间的时间步长（秒）
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_WEIGHTS_H