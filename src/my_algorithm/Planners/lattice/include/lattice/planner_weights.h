#ifndef LATTICE_PLANNER_WEIGHTS_H
#define LATTICE_PLANNER_WEIGHTS_H

namespace AD_algorithm {
namespace planner {

// 采样相关的可调项
struct SamplingParams {
  double sample_max_time = 8.0;
  double sample_min_time = 1.0;
  double sample_time_step = 1.0; // 时间采样步长（单位 s）
  double filter_dt=0.1;  //纵向过滤时的检测时间步长（单位 s）

  double sample_lat_width = 4.0;  // 横向采样范围（单位 m），左右对称
  double sample_width_length = 0.5;  // 横向采样间隔（单位 m）

  double sample_space_resolution = 0.5; // 计算代价时横向空间(s)采样步长，单位 m（默认 0.1）
  size_t beam_size = 5; // 在 RankPairs 中对每个 lon 考虑的 top-K lat（beam 大小）
};

// 代价 / 权重 可调项
struct WeightParams {
  double weight_st_object = 5.0; // 纵向目标/速度偏差代价权重
  double weight_st_jerk = 1.0;  // 纵向jerk舒适度权重
  double weight_lt_offset = 5.0;  // 横向偏移代价权重,参考线代价
  double weight_lt_acc = 1.0;  // 横向加速度代价权重
  double weight_st_acc = 0.5; // 对纵向加速度的额外惩罚
  double weight_obstacle = 1000.0; // 障碍距离惩罚权重（越大越偏向远离障碍）
  double weight_centripetal = 0.1; // 向心加速度代价权重
  double weight_Progress = 100.0; // 进展奖励权重（在规划的时间内达到目标速度行驶距离）,有效避障
};

// 物理限制与约束
struct LimitParams {
  double max_speed = 120.0 / 3.6;
  double max_acc = 8.0;
  double max_curvature = 100.0;
  double max_jerk = 10.0;
};

// 碰撞/预测相关可调参数
struct CollisionParams {
  double safe_distance = 3.0;           // 安全缓冲（m）
  double static_speed_threshold = 0.2;  // 低于该速度视为静态障碍（m/s）
  double collide_ds=1.0;              // 静态SL碰撞检测时的 s 采样间隔（m）
  double cost_distance_threshold = 20.0; // 障碍物距离超过该值不计入代价（m）
  double leader_speed_tolerance = 2.0; // 巡航判定阈值：若障碍物速度与参考速度差值超过此阈值，则不作为跟随目标（m/s）
};

// 规划器参数集合（采样 / 权重 / 限制）
struct PlannerParams {
  SamplingParams sampling;
  WeightParams weights;
  CollisionParams collision;
  LimitParams limits;

  double cruise_speed = 8.0;      // 期望巡航速度（m/s）
  double pre_time = 0.2; // 相邻规划周期之间的时间步长（秒）
  double final_time_step = 0.02; // 最终输出轨迹的时间步长
  double rank_dt = 0.5; // 横纵向组合计算代价时的时间步长
  double visualise_ds = 1.0; // 可视化输出的时间步长
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_WEIGHTS_H