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

namespace AD_algorithm {
namespace planner {

class cilqrPlanner : public PlannerBase {
private:
    std::shared_ptr<general::cilqr> cilqr_solver_;
    std::shared_ptr<problem> cilqr_problem_;
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

    void setPlannerParams(const AD_algorithm::general::cilqr_params& params);

    bool isTrajectoryValid(
      const std::vector<general::TrajectoryPoint>& trajectory,
      std::string* reason = nullptr,
      size_t min_points = 5) override;

    void set_log_enable(bool enable) override;
    
    std::vector<std::vector<general::TrajectoryPoint>> GetExtralTraj() override;

};

}}
