#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>
#include <string>
#include <sstream>
#include "general_modules/Trajectory.h"
#include "DP_solver.h"
#include "planner/planner_weights.h" 
#include "general_modules/Trajectory.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/Vehicle.h"
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace planner {

class SpeedPlanner {
public:
    SpeedPlanner(const WeightCoefficients& weights, const SpeedPlannerConfig& config,
                 std::shared_ptr<general::Logger> logger = nullptr);
    ~SpeedPlanner() = default;

    void set_logger(std::shared_ptr<general::Logger> logger) { logger_ = logger; }
    void set_log_enable(bool enable) { if (logger_) logger_->set_enable(enable); }

    bool set_config(const SpeedPlannerConfig& config) {
        if (!config.validate()) {
            if (logger_) logger_->log("ERROR", "Invalid configuration in set_config");
            return false;
        }
        config_ = config;
        return true;
    }
    
    // 速度规划主函数
    std::vector<general::STPoint> planSpeed(
        const AD_algorithm::general::FrenetFrame& ref_path_frame,
        const general::TrajectoryPoint& planning_start_point,
        double reference_speed,
        const std::vector<std::vector<AD_algorithm::general::FrenetPoint>>& dynamic_frenet_obstacles); 
        
private:
    template<typename... Args>
    void log(Args&&... args) const {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

    std::shared_ptr<general::Logger> logger_;
    //成员函数
    // 生成ST图
    std::vector<std::vector<general::STPoint>> generateSTGraph(
        const std::vector<std::vector<general::FrenetPoint>>& dynamic_obstacles,
        double delta_l = 2.0);
        
    // 加密速度剖面
    void increaseSpeedProfile(std::vector<general::STPoint>& DP_or_QP,double interval);
    
    void generate_convex_space(AD_algorithm::general::FrenetFrame ref_path_frenet,
        const std::vector<STObstacle>& st_obstacles,
        std::vector<double>& s_min,std::vector<double>& s_max,
        std::vector<double>& s_dot_lb,std::vector<double>& s_dot_ub);

    bool QP_traj_optimal(const std::vector<double>& s_lb, const std::vector<double>& s_ub, 
         const std::vector<double>& s_dot_lb, const std::vector<double>& s_dot_ub,
         const double reference_speed);
        
private:
    // 成员变量
    WeightCoefficients weights_;
    SpeedPlannerConfig config_;
    std::shared_ptr<std::vector<std::vector<STState>>> _sample_grid=nullptr; // 预计算的采样网格
    std::shared_ptr<OsqpEigen::Solver> _qp_solver;
    std::vector<AD_algorithm::general::STPoint> _dp_speed_profile;
    std::vector<AD_algorithm::general::STPoint> _qp_speed_profile;
};

} // namespace planner
} // namespace AD_algorithm

#endif // SPEED_PLANNER_H