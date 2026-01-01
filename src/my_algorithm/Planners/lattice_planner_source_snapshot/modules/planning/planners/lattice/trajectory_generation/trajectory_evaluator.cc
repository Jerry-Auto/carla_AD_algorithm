/****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/planners/lattice/trajectory_generation/trajectory_evaluator.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/planners/lattice/trajectory_generation/piecewise_braking_trajectory_generator.h"
#include "modules/planning/planning_base/common/trajectory1d/piecewise_acceleration_trajectory1d.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/constraint_checker/constraint_checker1d.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::math::PathMatcher;

using Trajectory1d = Curve1d;
using PtrTrajectory1d = std::shared_ptr<Trajectory1d>;
using Trajectory1dPair =
    std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>;

TrajectoryEvaluator::TrajectoryEvaluator(
    const std::array<double, 3>& init_s, const PlanningTarget& planning_target,
    const std::vector<PtrTrajectory1d>& lon_trajectories,
    const std::vector<PtrTrajectory1d>& lat_trajectories,
    std::shared_ptr<PathTimeGraph> path_time_graph,
    std::shared_ptr<std::vector<PathPoint>> reference_line)
    : path_time_graph_(path_time_graph),
      reference_line_(reference_line),
      init_s_(init_s) {
  const double start_time = 0.0;
  const double end_time = FLAGS_trajectory_time_length;
  path_time_intervals_ = path_time_graph_->GetPathBlockingIntervals(
      start_time, end_time, FLAGS_trajectory_time_resolution);

  reference_s_dot_ = ComputeLongitudinalGuideVelocity(planning_target);

  // if we have a stop point along the reference line,
  // filter out the lon. trajectories that pass the stop point.
  double stop_point = std::numeric_limits<double>::max();
  if (planning_target.has_stop_point()) {
    stop_point = planning_target.stop_point().s();
  }
  for (const auto& lon_trajectory : lon_trajectories) {
    double lon_end_s = lon_trajectory->Evaluate(0, end_time);
    if (init_s[0] < stop_point &&
        lon_end_s + FLAGS_lattice_stop_buffer > stop_point) {
      continue;
    }

    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*lon_trajectory)) {
      continue;
    }
    for (const auto& lat_trajectory : lat_trajectories) {
      /**
       * The validity of the code needs to be verified.
      if (!ConstraintChecker1d::IsValidLateralTrajectory(*lat_trajectory,
                                                         *lon_trajectory)) {
        continue;
      }
      */
      double cost = Evaluate(planning_target, lon_trajectory, lat_trajectory);
      cost_queue_.emplace(Trajectory1dPair(lon_trajectory, lat_trajectory),
                          cost);
    }
  }
  ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  return !cost_queue_.empty();
}

size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  return cost_queue_.size();
}

std::pair<PtrTrajectory1d, PtrTrajectory1d>
TrajectoryEvaluator::next_top_trajectory_pair() {
  ACHECK(has_more_trajectory_pairs());
  auto top = cost_queue_.top();
  cost_queue_.pop();
  return top.first;
}

double TrajectoryEvaluator::top_trajectory_pair_cost() const {
  return cost_queue_.top().second;
}

double TrajectoryEvaluator::Evaluate(
    const PlanningTarget& planning_target,
    const PtrTrajectory1d& lon_trajectory,
    const PtrTrajectory1d& lat_trajectory,
    std::vector<double>* cost_components) const {
  // Costs:
  // 1. Cost of missing the objective, e.g., cruise, stop, etc.
  // 2. Cost of longitudinal jerk
  // 3. Cost of longitudinal collision
  // 4. Cost of lateral offsets
  // 5. Cost of lateral comfort

  // Longitudinal costs
  double lon_objective_cost =
      LonObjectiveCost(lon_trajectory, planning_target, reference_s_dot_);

  double lon_jerk_cost = LonComfortCost(lon_trajectory);

  double lon_collision_cost = LonCollisionCost(lon_trajectory);

  double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);

  // decides the longitudinal evaluation horizon for lateral trajectories.
  double evaluation_horizon =
      std::min(FLAGS_speed_lon_decision_horizon,
               lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()));
  std::vector<double> s_values;
  for (double s = 0.0; s < evaluation_horizon;
       s += FLAGS_trajectory_space_resolution) {
    s_values.emplace_back(s);
  }

  // Lateral costs
  double lat_offset_cost = LatOffsetCost(lat_trajectory, s_values);

  double lat_comfort_cost = LatComfortCost(lon_trajectory, lat_trajectory);

  if (cost_components != nullptr) {
    cost_components->emplace_back(lon_objective_cost);
    cost_components->emplace_back(lon_jerk_cost);
    cost_components->emplace_back(lon_collision_cost);
    cost_components->emplace_back(lat_offset_cost);
  }

  return lon_objective_cost * FLAGS_weight_lon_objective +
         lon_jerk_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         centripetal_acc_cost * FLAGS_weight_centripetal_acceleration +
         lat_offset_cost * FLAGS_weight_lat_offset +
         lat_comfort_cost * FLAGS_weight_lat_comfort;
}