#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <memory>
#include <vector>
#include <deque>
#include <functional>
#include <string>
#include <sstream>
#include <limits>
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/logger.h"
#include "emplanner/planner_weights.h"  // 添加包含
#include "DP_solver.h"
#include "OsqpEigen/OsqpEigen.h"

namespace AD_algorithm {
namespace planner {

class PathPlanner {
public:
    PathPlanner(const WeightCoefficients& weights, const PathPlannerConfig& config, 
                std::shared_ptr<general::Logger> logger = nullptr);
    ~PathPlanner() = default;

    void set_logger(std::shared_ptr<general::Logger> logger) { logger_ = logger; }
    void set_log_enable(bool enable) { if (logger_) logger_->set_enable(enable); }

    void set_road_width(const std::shared_ptr<AD_algorithm::general::VehicleState>& vehicle_state,double offset_s) ;

    bool set_config(const PathPlannerConfig& config) {
        if (!config.validate()) {
            if (logger_) logger_->log("ERROR", "Invalid configuration in set_config");
            return false;
        }
        config_ = config;
        return true;
    }

    // 路径规划主函数,frenet_frame是上层管理的全局参考线定义的Frenet坐标系
    std::vector<general::TrajectoryPoint> planPath(
        const std::shared_ptr<general::FrenetFrame>& frenet_frame,
        const general::FrenetPoint& planning_start_point,
        const std::vector<std::vector<general::FrenetPoint>>& static_obstacles);

    // 获取路径规划结果
    const std::vector<general::FrenetPoint>& getDPPath() const { return dp_path_; }
    const std::vector<general::FrenetPoint>& getQPPath() const { return qp_path_; }
    const std::vector<general::TrajectoryPoint>& getTrajectory() const { return trajectory_; }

    // 获取最近一次生成的凸空间边界（返回与 getDPPath() 同长度的 l_min / l_max）
    void getLastConvexBounds(std::vector<double>& l_min, std::vector<double>& l_max) const {
        l_min = last_l_min_;
        l_max = last_l_max_;
    }

    // 获取最近一次 planPath 时用于 s 偏移的值（用于把局部 s 转回全局 s）
    double getLastOffsetS() const { return last_offset_s_; }

private:
    template<typename... Args>
    void log(Args&&... args) const {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

    enum class NearestObsDecision {
        Unknown = 0,
        BypassLeft = 1,
        BypassRight = 2,
    };

    void resetNearestObsDecision();
    void updateNearestObsDecision(
        double offset_s,
        const std::vector<std::vector<general::FrenetPoint>>& static_obstacles_global,
        const std::vector<std::vector<general::FrenetPoint>>& static_obstacles_local);


    // 生成凸空间
    void generateConvexSpace(
        const std::vector<std::vector<general::FrenetPoint>>& static_obstacles,
        std::vector<double>& l_min,
        std::vector<double>& l_max);

    // 成员函数
    bool QP_pathOptimization(const std::vector<double>& l_min,const std::vector<double>& l_max);
    
    void IncreasePathDensity(std::vector<general::FrenetPoint>& DP_or_QP,double interval);

private:

    // 成员变量
    WeightCoefficients weights_;
    PathPlannerConfig config_;
    std::shared_ptr<general::Logger> logger_;
    
    std::shared_ptr<OsqpEigen::Solver> qp_solver_;
    
    std::vector<general::FrenetPoint> dp_path_;
    std::vector<general::FrenetPoint> qp_path_;

    std::vector<general::TrajectoryPoint> trajectory_;

    std::shared_ptr<std::vector<std::vector<SLState>>> dp_sampling_grid_=nullptr;// 预计算的采样网格，加快规划速度

    std::vector<double> _road_width_left_vec;
    std::vector<double> _road_width_right_vec;
    double _road_width_resolution = 1.0;

    // 存储最近一次生成的凸空间边界（与 dp_path_ 对齐，s 为相对于规划起点的局部 s）
    std::vector<double> last_l_min_;
    std::vector<double> last_l_max_;
    // 记录最近一次 planPath 调用时的 s 偏移（offset_s），用于将局部 s 转回全局 s
    double last_offset_s_ = 0.0;

    // 最近障碍物的绕行决策锁定：记录近5次对最近障碍物的决策结果，取多数作为最终决策。
    // 一旦决策锁定，在越过该最近障碍物前保持不变，防止决策朝令夕改。
    static constexpr size_t kNearestObsDecisionWindow = 8;
    std::deque<NearestObsDecision> nearest_obs_decision_history_;
    NearestObsDecision locked_nearest_obs_decision_ = NearestObsDecision::Unknown;
    bool locked_nearest_obs_finalized_ = false;
    double locked_nearest_obs_global_s_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace planner
} // namespace AD_algorithm

#endif // PATH_PLANNER_H