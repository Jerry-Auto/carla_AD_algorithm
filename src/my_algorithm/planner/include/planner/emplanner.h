#ifndef EMPLANNER_H
#define EMPLANNER_H

#include <memory>
#include <vector>
#include <functional>
#include <string>
#include <sstream>
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "planner/planner_weights.h"  // 添加包含
#include "general_modules/Trajectory.h"
#include "planner/path_planner.h"
#include "planner/speed_planner.h"
#include "general_modules/planner_base.h"

namespace AD_algorithm {
namespace planner {

class EMPlanner : public PlannerBase {
public:
    EMPlanner();
    ~EMPlanner() = default;

    // 主规划函数
    std::vector<general::TrajectoryPoint> plan(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const std::vector<general::Obstacle>& obstacles,
        double reference_speed = 8.0,
        double current_time = 0.0) override;

    // 设置参数
    void setWeights(const WeightCoefficients& weights);
    void setPathPlannerConfig(const PathPlannerConfig& config);
    void setSpeedPlannerConfig(const SpeedPlannerConfig& config);
    
    // 设置全局的参考线，来自于全局导航，基本上只用初始化一次，除非行程有变化
    bool setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) override;

    // 规划结果合理性检验：供上层（planning_agent 等）判断是否需要重规划
    bool isTrajectoryValid(
        const std::vector<general::TrajectoryPoint>& trajectory,
        std::string* reason = nullptr,
        size_t min_points = 5) override;

    std::shared_ptr<general::FrenetFrame> getGlobalFrenetFrame_ptr() const {
        return global_frenet_frame_;
    };

    // 日志设置
    void set_log_enable(bool enable) override;
    
    // 获取历史轨迹
    const std::vector<general::TrajectoryPoint>& getLastTrajectory() const { return trajectory_manager_->getPreTrajectory(); }

private:
    // 规划器使用的全局Frenet坐标系
    std::shared_ptr<general::FrenetFrame> global_frenet_frame_=nullptr;

    // 生成轨迹 - 修正：返回向量而不是通过参数
    std::vector<general::TrajectoryPoint> generateTrajectory(
        const std::vector<general::STPoint>& speed_profile,
        const std::vector<general::TrajectoryPoint>& path_trajectory,
        double start_time);

    // 成员变量
    WeightCoefficients weights_;
    
    std::unique_ptr<general::TrajectoryManager> trajectory_manager_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<SpeedPlanner> speed_planner_;
};

} // namespace planner
} // namespace AD_algorithm

#endif // EMPLANNER_H