#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <memory>
#include <vector>
#include <functional>
#include <string>
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "planner/planner_weights.h"  // 添加包含
#include "DP_solver.h"
#include "OsqpEigen/OsqpEigen.h"

namespace AD_algorithm {
namespace planner {

class PathPlanner {
public:
    PathPlanner(const WeightCoefficients& weights,const PathPlannerConfig& config);
    ~PathPlanner() = default;

    // 设置日志回调
    void set_log_enable(bool enable) { _enable_log = enable; }
    void setLogCallback(std::function<void(const std::string&)> callback) {
        log_callback_ = callback;
    }
    
    bool set_config(const PathPlannerConfig& config) {
        if (!config.validate()) {
            log("Invalid configuration in set_config", "ERROR");
            return false;
        }
        config_ = config;
        return true;
    }

    // 路径规划主函数,frenet_frame是上层管理的全局参考线定义的Frenet坐标系
    std::vector<general::TrajectoryPoint> planPath(
        const std::shared_ptr<general::FrenetFrame>& frenet_frame,
        const general::FrenetPoint& planning_start_point,
        const std::vector<general::FrenetPoint>& static_obstacles);  // 移除默认参数 = PathPlannerConfig()

    // 获取路径规划结果
    const std::vector<general::FrenetPoint>& getDPPath() const { return dp_path_; }
    const std::vector<general::FrenetPoint>& getQPPath() const { return qp_path_; }
    const std::vector<general::TrajectoryPoint>& getTrajectory() const { return trajectory_; }

private:


    // 生成凸空间
    void generateConvexSpace(
        const std::vector<general::FrenetPoint>& static_obstacles,
        std::vector<double>& l_min,
        std::vector<double>& l_max);

    // 成员函数
    void log(const std::string& message, const std::string& level = "INFO");
    bool QP_pathOptimization(const std::vector<double>& l_min,const std::vector<double>& l_max);
    
    void IncreasePathDensity(std::vector<general::FrenetPoint>& DP_or_QP,double interval);

private:

    // 成员变量
    WeightCoefficients weights_;
    std::shared_ptr<OsqpEigen::Solver> qp_solver_;
    
    std::vector<general::FrenetPoint> dp_path_;
    std::vector<general::FrenetPoint> qp_path_;

    std::vector<general::TrajectoryPoint> trajectory_;

    PathPlannerConfig config_;
    std::shared_ptr<std::vector<std::vector<SLState>>> dp_sampling_grid_=nullptr;// 预计算的采样网格，加快规划速度

    std::function<void(const std::string&)> log_callback_;
    bool _enable_log = true;
};

} // namespace planner
} // namespace AD_algorithm

#endif // PATH_PLANNER_H