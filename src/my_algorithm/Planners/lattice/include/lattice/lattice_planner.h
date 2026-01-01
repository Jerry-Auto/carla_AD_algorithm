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
#include "lattice/trajectory1d_generator.h"
#include "lattice/trajectory_evaluator.h"
#include "lattice/trajectory_combiner.h"
#include "lattice/lattice_collision_detection.h"
#include "lattice/planner_weights.h"
#include "general_modules/Trajectory.h"
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

/**
 * @brief latticePlanner
 *
 * 高层协调器，负责实现 lattice 规划流水线：
 * 1) 生成一维纵向与横向候选（Trajectory1DGenerator）
 * 2) 对候选对进行评估与排序（TrajectoryEvaluator）
 * 3) 将优先候选组合为笛卡尔轨迹（TrajectoryCombiner）
 * 4) 使用轨迹约束校验（TrajectoryManager）与碰撞检测（latticeCollisionDetection::InCollision）验证结果
 *
 * 该类继承自 `PlannerBase`，并提供 `plan`、`setGlobalReferenceLine` 与 `isTrajectoryValid` 等方法以便集成。
 */
class latticePlanner : public PlannerBase {
 public:
  /**
   * @brief Construct a new lattice Planner object
   * 
   * @param params grouped planner parameters (sampling/weights/limits)
   * @param collision_detection_ptr pre-constructed collision detection helper
   */
  latticePlanner();
  ~latticePlanner()=default;
  
  // 重写 PlannerBase 的方法
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

  // 更新 Planner 参数并重新初始化内部组件（例如 Trajectory1DGenerator / Evaluator）
  // 使得在运行时更换参数后，planner 能立即生效。建议在测试或运行启动时调用。
  void setPlannerParams(const PlannerParams& params);

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
   * @brief Generate ALL lateral candidate paths (Frenet -> Cartesian).
   *
   * This returns for a given lateral initial point `init` and time horizon `T` all
   * lateral candidates converted to Cartesian coordinates. Each candidate is a
   * vector of `general::TrajectoryPoint`. Time is ignored (set to 0) as requested.
   */
  /**
   * @brief Parameterless overload: return the last generated lateral candidates.
   * If not available, try to regenerate using stored last init/T/s_offset.
   */
  std::vector<std::vector<general::TrajectoryPoint>> GetAllLateralCandidatesCartesian();
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

 private:
  // 过滤辅助函数：使用 TrajectoryManager 对一维候选进行预筛选
  void FilterLonCandidates(const latticeFrenetPoint& init,
                            std::vector<Trajectory1DGenerator::LonCandidate>& lon_candidates);

  // 横向候选初筛：s_offset 为规划起点的绝对 s（用于恢复绝对 s）
  // 使用成员缓存的 FrenetFrame（`global_frenet_frame_`），若无则尝试基于 `global_reference_line_` 临时构造
  void FilterLatCandidates(std::vector<Trajectory1DGenerator::LatCandidate>& lat_candidates,double s_offset);
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
  // 规划器配置（分组）
  PlannerParams planner_params_;  // 采样 / 权重 / 限制

  double desired_speed_;          // 期望速度
  double cruise_speed_;           // 巡航速度
  latticeFrenetPath best_path_;          // 当前周期的最优轨迹
  latticeFrenetPath pre_best_path_;      // 上一周期的最优轨迹
  // 碰撞检测模块
  std::shared_ptr<latticeCollisionDetection> collision_detection_ptr_;

 private:
  // 新的模块化组件
  std::unique_ptr<Trajectory1DGenerator> traj1d_generator_;
  std::unique_ptr<TrajectoryEvaluator> traj_evaluator_;
  std::unique_ptr<TrajectoryCombiner> traj_combiner_;

  // 复用通用 TrajectoryManager 进行约束校验
  AD_algorithm::general::TrajectoryManager traj_manager_;

  std::vector<general::PathPoint> global_reference_line_;  // 为 API 兼容性保留的参考路径副本
  // 缓存从全局参考线路径构建的 FrenetFrame（避免重复构造以提升性能）
  std::shared_ptr<general::FrenetFrame> global_frenet_frame_;

  // 存储最近一次生成的横向候选（Frenet 多项式）以及相关偏移
  std::vector<Trajectory1DGenerator::LatCandidate> lat_candidates_;
  double last_lat_T_ = 0.0;
  double last_lat_s_offset_ = 0.0;

  bool log_enable_ = false;
  bool is_following_ = false;
};



// plan 的实现移至源码文件中（.cpp）

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_H_
