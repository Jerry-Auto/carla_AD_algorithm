#pragma once
#include <vector>
#include <memory>
#include <string>
#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/planner_base.h"
#include "general_modules/common_types.h"
#include "cilqr_planner/cost_define.h"
#include "cilqr_planner/planner_weight.h"
#include "cilqr_planner/cost_factory.h"
#include "cilqr_planner/planner_cost.h"

namespace AD_algorithm {
namespace planner {

class cilqrPlanner : public PlannerBase {
private:
    std::shared_ptr<general::ILQR> cilqr_solver_;
    std::shared_ptr<planner_cost> cilqr_cost_;
    std::shared_ptr<CILQRPlannerparams> planner_params_;
    std::shared_ptr<AD_algorithm::general::FrenetFrame> global_frenet_frame_;
public:
    cilqrPlanner();
    ~cilqrPlanner()=default;
    // 主流程接口
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
    
    std::vector<std::vector<general::TrajectoryPoint>> GetExtralTraj() override;
private:
    void solve();
    void set_reference_speed(double v_ref);
    void set_initial_state(const general::TrajectoryPoint& x0);
    void set_initial_state(const general::VehicleState& x0);
    std::vector<general::TrajectoryPoint> get_trajectory(double current_time);
    void set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles);
    void set_road_bounds(const double& lower_bound, const double& upper_bound);
    void set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf);
    std::vector<general::TrajectoryPoint> traj_densify(const std::vector<general::TrajectoryPoint>& raw_traj, double time_resolution=0.01);
private:
    std::vector<general::TrajectoryPoint> last_trajectory_;
    // 复用通用 TrajectoryManager 进行约束校验，轨迹拼接等操作
    std::shared_ptr<AD_algorithm::general::TrajectoryManager> traj_manager_;
    std::vector<std::vector<general::TrajectoryPoint>> extral_trajectories_;
    void predict_obstacles(double current_time, const std::vector<general::Obstacle>& obstacles,std::vector<std::vector<general::Obstacle>>& predicted_obstacles);
};

}}
