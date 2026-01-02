#ifndef LATTICE_PLANNER_H_
#define LATTICE_PLANNER_H_

#include <queue>
#include <memory>
#include <vector>
#include <string>

#include "lattice/lattice_types.h"
#include "lattice/lattice_collision_detection.h"
#include "lattice/trajectory1d_generator.h"
#include "lattice/trajectory_evaluator.h"
#include "lattice/planner_weights.h"
#include "general_modules/Trajectory.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/planner_base.h"


namespace AD_algorithm {
namespace planner {

class latticePlanner : public PlannerBase {
public:
    latticePlanner();
    ~latticePlanner()=default; 
    // 获取所有横向路径的笛卡尔坐标序列
    std::vector<std::vector<general::TrajectoryPoint>> GetAllLateralCandidatesCartesian();
    // 主流程接口
    std::vector<general::TrajectoryPoint> plan(
      const std::shared_ptr<general::VehicleState>& ego_state,
      const std::vector<general::Obstacle>& obstacles,
      double reference_speed = 8.0,
      double current_time = 0.0) override;

    bool setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) override;

    void setPlannerParams(const PlannerParams& params);

    bool isTrajectoryValid(
      const std::vector<general::TrajectoryPoint>& trajectory,
      std::string* reason = nullptr,
      size_t min_points = 5) override;

    void set_log_enable(bool enable) override;
    
    std::vector<std::vector<general::TrajectoryPoint>> GetExtralTraj() override;

private:
  void FilterLatCandidates(std::vector<LatCandidate>& lat_candidates, double s_offset);

  // 过滤辅助函数：使用 TrajectoryManager 对一维候选进行预筛选
  void FilterLonCandidates(std::vector<LonCandidate>& lon_candidates);

private:
  // 规划器配置（分组）
  PlannerParams planner_params_;  // 采样 / 权重 / 限制

  double desired_speed_;          // 期望速度
  double cruise_speed_;           // 巡航速度
  latticeFrenetPath best_path_;          // 当前周期的最优轨迹
  latticeFrenetPath pre_best_path_;      // 上一周期的最优轨迹

  std::shared_ptr<latticeCollisionDetection> collision_detection_ptr_;
  // 新的模块化组件
  std::unique_ptr<Trajectory1DGenerator> traj1d_generator_;
  std::unique_ptr<TrajectoryEvaluator> traj_evaluator_;

  // 复用通用 TrajectoryManager 进行约束校验
  AD_algorithm::general::TrajectoryManager traj_manager_;

  std::shared_ptr<general::FrenetFrame> global_frenet_frame_;

  std::vector<LatCandidate> lat_path_visualize_;

  double t_offset_ = 0.0;
  double s_offset_ = 0.0;

  bool log_enable_ = false;
  bool is_following_ = false;
};


// 替换Cmp为用户提供的CmpPair
using Cmp = CmpPair;


// plan 的实现移至源码文件中（.cpp）

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_H_
