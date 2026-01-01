#ifndef LATTICE_COLLISION_DETECTION_H_
#define LATTICE_COLLISION_DETECTION_H_

#include <cfloat>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <memory>

#include "lattice/lattice_types.h"
#include "general_modules/collision_detection.h"

namespace AD_algorithm {
namespace planner {

class latticeCollisionDetection {
 public:
  // 默认构造：不会固定障碍物，实例化后应在每个规划周期通过 `Update(...)` 传入最新障碍物与参考路径
  latticeCollisionDetection();

  /**
   * @brief Construct a new Collision Detection object
   * 
   * NOTE: 该构造仅作为便利构造，仍建议在每个规划周期使用 `Update(...)` 来刷新障碍物数据，
   * 避免在构造时把数据“固定”起来导致后续周期不更新。
   *
   * @param detected_objects 检测到的其他车辆，也就是道路中的障碍物
   * @param collision_distance 碰撞距离
   * @param ref_path 参考路径（Cartesian下）
   */
  latticeCollisionDetection(const std::vector<Obstacle>& detected_objects,
                     const double& collision_distance,
                     const std::vector<PathPoint>& ref_path);

  // 设置碰撞距离（可在运行时调整）
  void setCollisionDistance(double d) { collision_dis_ = d; }
  
  /**
   * @brief 更新障碍物和参考路径
   * 
   * @param detected_objects 检测到的其他车辆
   * @param ref_path 参考路径
   */
  void Update(const std::vector<Obstacle>& detected_objects,
              const std::vector<PathPoint>& ref_path);

  /**
   * @brief 对障碍物进行分类，分为静态障碍物和动态障碍物
   * 
   * @param detected_objects 检测到的其他车辆，也就是道路中的障碍物
   */
  void ObstacleClassification(std::vector<Obstacle>& detected_objects);
  /**
   * @brief 计算障碍box
   * 
   * @param object 障碍物
   */
  void CalCollisionBox(Obstacle& object);
  /**
   * @brief 判断是否碰撞
   * 
   * @param path Frenet坐标系下的轨迹
   * @param leader_frenet_point 前面车的坐标点
   * @param is_car_followed 是否正在跟车的flag
   * @return true 会发生碰撞
   * @return false 不会发生碰撞
   */
  bool IsCollision(latticeFrenetPath& path, const latticeFrenetPoint& leader_frenet_point,
                   const bool& is_car_followed);

  // 简单的轨迹级别查询：该笛卡尔轨迹是否与任何障碍物发生碰撞？
  bool InCollision(const std::vector<AD_algorithm::general::TrajectoryPoint>& trajectory) const;

 public:
  double collision_dis_;                          // 碰撞距离
  std::vector<Obstacle> detected_objects_;        // 检测到的所有障碍物
  std::vector<Obstacle> static_obstacle_list_;    // 静态障碍物
  std::vector<Obstacle> dynamic_obstacle_list_;   // 动态障碍物
  std::vector<PathPoint> ref_path_;               // 参考轨迹（Cartesian下）

  // Time-indexed predicted obstacle polygons (per planning cycle)
  // predicted_polygons_[i] 是在 time = i * time_resolution_ 时刻的障碍多边形列表
  std::vector<std::vector<std::shared_ptr<AD_algorithm::general::Polygon2d>>> predicted_polygons_;

  // configurable timing parameters for prediction sampling
  double time_horizon_ = 5.0;        // seconds
  double time_resolution_ = 0.5;     // seconds

  // ego vehicle geometry (可在运行时设置)
  double ego_length_ = 4.5;
  double ego_width_ = 2.0;
  double ego_back_to_center_ = 1.0;

  // Build predicted polygons for each time step (call during Update)
  void BuildPredictedEnvironment();

  // set time horizon / resolution
  void setTimeHorizon(double h) { time_horizon_ = h; }
  void setTimeResolution(double r) { time_resolution_ = r; }

  // set ego size
  void setEgoSize(double length, double width, double back_to_center) {
    ego_length_ = length; ego_width_ = width; ego_back_to_center_ = back_to_center;
  }

  // Query helpers
  // 返回在指定时间点 t 时，点(x,y) 到最近障碍的距离（若超出时间范围则取最近端点）
  double MinDistanceToObstaclesAtTime(double x, double y, double t) const;

  // Check overlap between ego box at (x,y,heading) and predicted obstacles at time t
  bool HasOverlapWithPredicted(double x, double y, double heading, double t) const;
};



} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_COLLISION_DETECTION_H_
