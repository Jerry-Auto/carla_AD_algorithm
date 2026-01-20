#ifndef LATTICE_PLANNER_WEIGHTS_H
#define LATTICE_PLANNER_WEIGHTS_H

namespace AD_algorithm {
namespace planner {

// 代价 / 权重 可调项
struct WeightParams {
  double weight_st_object = 20.0; // 纵向目标/速度偏差代价权重
  double weight_st_jerk = 0.0;  // 纵向jerk舒适度权重
  double weight_lt_offset = 1000.0;  // 横向偏移代价权重,参考线代价
  double weight_lt_acc = 0.0;  // 横向加速度代价权重
  double weight_st_acc = 0.0; // 对纵向加速度的额外惩罚
  double weight_obstacle = 100.0; // 障碍距离惩罚权重（越大越偏向远离障碍）
  double weight_centripetal = 0.0; // 向心加速度代价权重
  double weight_Progress = 0.0; // 进展奖励权重（在规划的时间内达到目标速度行驶距离）,有效避障
};

// 物理限制与约束
struct LimitParams {
  double max_speed = 120.0 / 3.6;
  double max_acc = 8.0;
  double max_curvature = 100.0;
  double max_jerk = 10.0;
  double max_steering_angle = 30.0 / 180.0 * 3.1415926;
  
};

// 碰撞/预测相关可调参数
struct CollisionParams {
  double safe_distance = 3.0;           // 安全缓冲（m）
  double static_speed_threshold = 0.2;  // 低于该速度视为静态障碍（m/s）
  double collide_ds=1.0;              // 静态SL碰撞检测时的 s 采样间隔（m）
  double cost_distance_threshold = 80.0; // 障碍物距离超过该值不计入代价（m）
  double leader_speed_tolerance = 2.0; // 巡航判定阈值：若障碍物速度与参考速度差值超过此阈值，则不作为跟随目标（m/s）
};


// 规划器参数集合（采样 / 权重 / 限制）
struct PlannerParams {
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