#ifndef LATTICE_PLANNER_H_
#define LATTICE_PLANNER_H_

#include <unordered_map>
#include <queue>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "lattice/lattice_types.h"
#include "lattice/lattice_collision_detection.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/polynomial_curve.h"
#include "general_modules/planner_base.h"


namespace AD_algorithm {
namespace planner {

class Cmp {
 public:
  bool operator()(const latticeFrenetPath& a, const latticeFrenetPath& b) {
    return a.cost > b.cost;
  }
};

class latticePlanner : public PlannerBase {
 public:
  latticePlanner() = delete;
  /**
   * @brief Construct a new lattice Planner object
   * 
   * @param cruise_speed 巡航速度
   * @param lattice_params 规划器的参数
   * @param collision_detection_ptr 碰撞类
   */
  latticePlanner(const double& cruise_speed,
                 std::unordered_map<std::string, double>& lattice_params,
                 std::shared_ptr<latticeCollisionDetection> collision_detection_ptr);

  // Override PlannerBase methods
  std::vector<general::TrajectoryPoint> plan(
      const std::shared_ptr<general::VehicleState>& ego_state,
      const std::vector<general::Obstacle>& obstacles,
      double reference_speed = 8.0,
      double current_time = 0.0) override;

  bool setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) override;

  bool isTrajectoryValid(
      const std::vector<general::TrajectoryPoint>& trajectory,
      std::string* reason = nullptr,
      size_t min_points = 5) override;

  void set_log_enable(bool enable) override;

  /**
   * @brief Get the St Values object
   * 
   * @param frenet_point 当前Frenet坐标系下的点
   * @param st_polynomial 纵向多项式
   */
  void GetStValues(latticeFrenetPoint& frenet_point, 
                   const std::shared_ptr<AD_algorithm::general::PolynomialCurve> st_polynomial,
                   double t);
  
  /**
   * @brief Get the Lt Values object
   * 
   * @param frenet_point 当前Frenet坐标系下的点
   * @param lt_polynomial 横向多项式
   */
  void GetLtValues(latticeFrenetPoint& frenet_point, 
                   const std::shared_ptr<AD_algorithm::general::PolynomialCurve> lt_polynomial,
                   double t);
  /**
   * @brief Get the St Object Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @param target_speed 目标速度
   * @return double 纵向目标代价
   */
  double GetStObjectCost(const latticeFrenetPath& frenet_path, const double& target_speed);
  /**
   * @brief Get the St Jerk Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @return double 纵向舒适代价
   */
  double GetStJerkCost(const latticeFrenetPath& frenet_path);
  /**
   * @brief Get the Lt Offset Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @return double 横向偏离代价
   */
  double GetLtOffsetCost(const latticeFrenetPath& frenet_path, 
                         const latticeFrenetPoint& initial_frenet_point);
  /**
   * @brief Get the Lt Acc Cost object
   * 
   * @param frenet_path 当前Frenet坐标系下的路径
   * @return double 横向舒适代价
   */
  double GetLtAccCost(const latticeFrenetPath& frenet_path);
  /**
   * @brief 跟车时的路径生成方式
   * 
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @return std::vector<latticeFrenetPath> 待进一步选择的轨迹族，Frenet坐标系下
   */
  std::vector<latticeFrenetPath> SamplingFollowingFrenetPaths(
      const latticeFrenetPoint& initial_frenet_point, 
      const latticeFrenetPoint& leader_frenet_point);
  /**
   * @brief 无跟车时的路径生成方式
   * 
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @return std::vector<latticeFrenetPath> 待进一步选择的轨迹族，Frenet坐标系下
   */
  std::vector<latticeFrenetPath> SamplingCruisingFrenetPaths(
      const latticeFrenetPoint& initial_frenet_point);
  /**
   * @brief Get the Cartesian Paths object 将Frenet坐标系下的路径转换为Cartesian下
   * 
   * @param frenet_paths 待转换的Frenet下路经
   * @param ref_path 参考路径（Cartesian下）
   */
  void GetCartesianPaths(std::vector<latticeFrenetPath>& frenet_paths, 
                         const std::vector<PathPoint>& ref_path);
  /**
   * @brief Get the Valid Paths object 过滤不合格的路径（比如有障碍物，速度加速度曲率不合格）
   * 
   * @param frenet_paths 待选择的Frenet下路经
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @param is_car_followed 是否正在跟车的flag
   * @return std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> 
   * 最小堆，第一个即为cost最小的路径
   */
  std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> GetValidPaths(
      std::vector<latticeFrenetPath>& frenet_paths, const latticeFrenetPoint& leader_frenet_point, 
      const bool& is_car_followed);
  /**
   * @brief Get the Candidate Paths object 计算局部路径族
   * 
   * @param ref_path 参考路径（Cartesian下）
   * @param initial_frenet_point 初始路径点的Frenet坐标系下的点
   * @param leader_frenet_point 前车在Frenet坐标系下的坐标
   * @param is_car_followed 是否正在跟车的flag
   * @return std::vector<latticeFrenetPath> 未经选择的Frenet坐标系下的所有生成轨迹
   */
  std::vector<latticeFrenetPath> GetCandidatePaths(
      const std::vector<PathPoint>& ref_path, const latticeFrenetPoint& initial_frenet_point,
      const latticeFrenetPoint& leader_frenet_point, const bool& is_car_followed);
 
 public:
  const double MAX_SPEED = 50.0 / 3.6;
  const double MAX_ACCEL = 8.0;
  const double MAX_CURVATURE = 100.0;
  // sample params
  double sample_max_time_;        // 最大纵向采样时间
  double sample_min_time_;        // 最小纵向采样时间
  double sample_time_step_;       // 采样间隔
  double sample_lat_width_;       // 横向采样距离
  double sample_width_length_;    // 横向采样间隔
  // weight params
  double weight_st_object_;       // 纵向目标代价
  double weight_st_jerk_;         // 纵向舒适代价
  double weight_lt_offset_;       // 横向偏离代价
  double weight_lt_acc_;          // 横向舒适代价
  
  double desired_speed_;          // 期望速度
  double cruise_speed_;           // 巡航速度
  latticeFrenetPath best_path_;          // 当前周期的最优轨迹
  latticeFrenetPath pre_best_path_;      // 上一周期的最优轨迹
  // 碰撞检测模块
  std::shared_ptr<latticeCollisionDetection> collision_detection_ptr_;
};



std::vector<general::TrajectoryPoint> latticePlanner::plan(
    const std::shared_ptr<general::VehicleState>& ego_state,
    const std::vector<general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {

  // Step 1: Preprocessing
  latticeFrenetPoint initial_frenet_point;
  // Convert Cartesian to Frenet coordinates
  Cartesian2Frenet(ego_state, global_reference_line_, initial_frenet_point);

  // Step 2: Sampling Strategy
  std::vector<latticeFrenetPath> candidate_paths;
  if (is_following_) {
    latticeFrenetPoint leader_frenet_point;
    // Obtain leader's Frenet state
    GetLeaderFrenetState(obstacles, leader_frenet_point);
    candidate_paths = SamplingFollowingFrenetPaths(initial_frenet_point, leader_frenet_point);
  } else {
    candidate_paths = SamplingCruisingFrenetPaths(initial_frenet_point);
  }

  // Step 3: Trajectory Generation
  for (auto& path : candidate_paths) {
    GenerateFrenetTrajectory(path);
  }

  // Step 4: Frenet to Cartesian Conversion
  GetCartesianPaths(candidate_paths, global_reference_line_);

  // Step 5: Cost Calculation
  for (auto& path : candidate_paths) {
    path.cost += GetStObjectCost(path, reference_speed);
    path.cost += GetStJerkCost(path);
    path.cost += GetLtOffsetCost(path, initial_frenet_point);
    path.cost += GetLtAccCost(path);
  }

  // Step 6: Validity Check & Selection
  auto valid_paths = GetValidPaths(candidate_paths, leader_frenet_point, is_following_);
  if (valid_paths.empty()) {
    throw std::runtime_error("No valid paths found.");
  }

  best_path_ = valid_paths.top();

  return ConvertToTrajectoryPoints(best_path_);
}

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_H_
