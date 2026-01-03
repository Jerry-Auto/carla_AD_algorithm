#include "lattice/lattice_planner.h"
#include <iostream>

namespace AD_algorithm {
namespace planner {

latticePlanner::latticePlanner() {
    // 使用结构化参数初始化各个模块组件
    collision_detection_ptr_=std::make_shared<latticeCollisionDetection>();
    planner_params_ = PlannerParams();
    traj1d_generator_ = std::make_unique<Trajectory1DGenerator>(planner_params_);
    traj_evaluator_ = std::make_unique<TrajectoryEvaluator>(planner_params_,collision_detection_ptr_);
    // 根据 planner 参数配置 TrajectoryManager 的限制（速度/加速度/曲率/jerk）
    traj_manager_.setLimits(planner_params_.limits.max_speed,
                            planner_params_.limits.max_acc,
                            planner_params_.limits.max_curvature,
                            planner_params_.limits.max_jerk);
    desired_speed_ = planner_params_.cruise_speed;
}

void latticePlanner::setPlannerParams(const PlannerParams& params) {
  planner_params_ = params;
  // 重新初始化子模块，确保新参数生效
  traj1d_generator_ = std::make_unique<Trajectory1DGenerator>(planner_params_);
  traj_evaluator_ = std::make_unique<TrajectoryEvaluator>(planner_params_, collision_detection_ptr_);
  // 重新设置限制
  traj_manager_.setLimits(planner_params_.limits.max_speed,
                          planner_params_.limits.max_acc,
                          planner_params_.limits.max_curvature,
                          planner_params_.limits.max_jerk);
}


bool latticePlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
  if (reference_line.size() < 2) {
    log("Failed to set reference line: too few points");
    return false;
  }

  // 缓存参考路径并构建 FrenetFrame（参考 EMPlanner 的实现）
  try {
    global_frenet_frame_ = std::make_shared<AD_algorithm::general::FrenetFrame>(reference_line);
    // cache the reference line for later use in Combine/visualization
    log("Global reference line set (", reference_line.size(), " points)");
    return true;
  } catch (const std::exception& e) {
    log("Exception while constructing FrenetFrame: ", e.what());
    global_frenet_frame_.reset();
    return false;
  }
}

bool latticePlanner::isTrajectoryValid(
    const std::vector<general::TrajectoryPoint>& trajectory,
    std::string* reason,
    size_t min_points) {
  // 委托给 TrajectoryManager 进行约束校验，数值上的检查
  return traj_manager_.isTrajectoryValid(trajectory, reason, min_points) ;
}

void latticePlanner::set_log_enable(bool enable) {
  log_enable_ = enable;
  PlannerBase::set_log_enable(enable);
}

std::vector<std::vector<general::TrajectoryPoint>> latticePlanner::GetExtralTraj(){
  return GetAllLateralCandidatesCartesian();
}

std::vector<std::vector<general::TrajectoryPoint>> latticePlanner::GetAllLateralCandidatesCartesian() {
  std::vector<std::vector<general::TrajectoryPoint>> all_trajs;
  // 只需横向候选，使用ds采样即可
  for (const auto& lat : lat_path_visualize_) {
    std::vector<general::TrajectoryPoint> traj;
      for(double s=0.0; s<=lat.param_s; s+=planner_params_.visualise_ds) {
        double d = lat.curve->value_evaluation(s, 0);
        double d_prime = lat.curve->value_evaluation(s, 1);
        double d_prime_prime = lat.curve->value_evaluation(s, 2);
        // 构建 Frenet 点
        general::FrenetPoint fpt;
        fpt.s = s + s_offset_; // 使用全局偏移
        fpt.l = d;
        fpt.l_dot = d_prime;
        fpt.l_dot_dot = d_prime_prime;

        auto traj_point = global_frenet_frame_->frenet_to_cartesian(fpt);
        traj.push_back(traj_point);
      }
    all_trajs.push_back(traj);
  }
  return all_trajs;
}


std::vector<AD_algorithm::general::TrajectoryPoint> latticePlanner::plan(
    const std::shared_ptr<AD_algorithm::general::VehicleState>& ego_state,
    const std::vector<AD_algorithm::general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {

  log("INFO", "Starting planning cycle at time: " + std::to_string(current_time));
  
  traj_manager_.setCurrentTime(current_time);
  traj_manager_.updateEgoState(ego_state);
  
  // 2. 计算规划起点
  log("INFO", "Step 2: Calculating planning start point...");
  AD_algorithm::general::TrajectoryPoint planning_start_point = traj_manager_.calculatePlanningStartPoint(ego_state, planner_params_.final_time_step);
  
  // 将 规划起点转换到 Frenet 坐标系
  auto ego_fp = global_frenet_frame_->cartesian_to_frenet(planning_start_point);
  // 记录绝对 s_offset_ / t0，并将 init 归一化为相对坐标（s=0, t=0）用于采样
  s_offset_ = ego_fp.s;
  t_offset_ = planning_start_point.time_stamped;
  latticeFrenetPoint init;
  init.t = 0.0; init.s = 0.0; init.s_dot = ego_fp.s_dot; init.s_dot_dot = ego_fp.s_dot_dot;
  init.l = ego_fp.l; init.l_dot = ego_fp.l_dot; init.l_dot_dot = ego_fp.l_dot_dot;

  // 1. 更新碰撞检测器
  log("INFO", "Step 1: Updating collision detector...");
  // 更新碰撞检测器（使用当前障碍物与参考路径）
  if (collision_detection_ptr_) collision_detection_ptr_->Update(obstacles, global_frenet_frame_,t_offset_);

  // 检测前车（简单启发式方法） - 基于 s_offset_ 计算相对纵向距离
  // 过滤掉与参考速度差异过大的障碍物（避开不相关的快速或非常慢的物体，使用巡航策略绕过）
  log("INFO", "Step 3: Detecting leader vehicle...");
  latticeFrenetPoint leader_fp;
  bool has_leader = false;
  double min_ds = 1e9;
  for (const auto& obs : obstacles) {
    double obs_speed = obs.getSpeed();
    // 若速度与参考速度相差过大，则跳过，不做为跟随目标
    if (std::fabs(obs_speed - reference_speed) > planner_params_.collision.leader_speed_tolerance) {
      continue;
    }
    AD_algorithm::general::TrajectoryPoint otp; otp.x = obs.x; otp.y = obs.y; otp.v = obs_speed; otp.heading = obs.heading;
    auto ofp = global_frenet_frame_->cartesian_to_frenet(otp);
    double ds = ofp.s - s_offset_; // relative to planning start
    if (ds > 0 && std::abs(ofp.l) < 3.5 && ds < min_ds) {
      min_ds = ds; has_leader = true;
      leader_fp.s = ofp.s - s_offset_; // store leader.s as relative
      leader_fp.s_dot = ofp.s_dot; leader_fp.l = ofp.l;
      leader_fp.x = otp.x; leader_fp.y = otp.y;
    }
  }
  // 更新是否处于跟随状态
  is_following_ = has_leader;

  // 同步 desired_speed_ 与本次调用的参考速度，确保所有基于速度的采样一致
  desired_speed_ = reference_speed;

  // 生成一维候选，使用的相对时间，以规划起点为 t=0
  log("INFO", "Step 4: Generating longitudinal candidates...");
  std::vector<LonCandidate> lon_candidates;
  if (has_leader) {
    lon_candidates = traj1d_generator_->GenerateLongitudinalFollowing(init, leader_fp, reference_speed);
  } else {
    lon_candidates = traj1d_generator_->GenerateLongitudinalCruising(init, reference_speed);
  }

  log("INFO", "Lon candidates generated: " + std::to_string(lon_candidates.size()));
  log("INFO", "s_offset=" + std::to_string(s_offset_) + " t_offset=" + std::to_string(t_offset_));

  // 用成员方法进行初筛（优化：重用缓冲区，使用 planner_params_.sampling.sample_dt）
  FilterLonCandidates(lon_candidates);
  log("INFO", "Lon candidates after FilterLonCandidates: " + std::to_string(lon_candidates.size()));

  // 对每个 T 采样横向候选（使用第一个纵向候选的 T 作为代表）
  std::vector<LatCandidate> lat_candidates;
  if (!lon_candidates.empty()) {
    double T = lon_candidates.front().T;
    lat_candidates = traj1d_generator_->GenerateLateralCandidates(init, T, reference_speed);
  }

  log("INFO", "Lat candidates generated: " + std::to_string(lat_candidates.size()));
  lat_path_visualize_=lat_candidates; // 用于可视化

  // 用成员方法进行横向初筛
  FilterLatCandidates(lat_candidates, s_offset_);
  log("INFO", "Lat candidates after FilterLatCandidates: " + std::to_string(lat_candidates.size()));

  if (lon_candidates.empty() || lat_candidates.empty()) {
    log("INFO", "No candidates generated (lon=" + std::to_string(lon_candidates.size()) + " lat=" + std::to_string(lat_candidates.size()) + ")");
    return {};
  }

  log("INFO", "Step 5: Evaluating trajectory pairs...");
  // Populate the queue with ranked pairs (pass frenet frame, s_offset and absolute start time)
  traj_evaluator_->RankPairs(lon_candidates, lat_candidates, reference_speed, global_frenet_frame_, s_offset_, current_time);
  log("DEBUG", "RankPairs completed, has_more_pairs=" + std::to_string(traj_evaluator_->has_more_pairs()));

  // 逐个迭代并选择第一个通过约束的轨迹
  log("INFO", "Step 6: Searching for valid trajectory...");
  while (traj_evaluator_->has_more_pairs()) {
    auto top = traj_evaluator_->next_top_pair();
    // Correctly use lon and lat members of CandidatePair
    auto lon = top.lon;
    auto lat = top.lat;
    log("DEBUG", "Processing pair, cost=" + std::to_string(top.cost) + " T_lon=" + std::to_string(lon ? lon->T : 0.0) + " T_lat=" + std::to_string(lat ? lat->T : 0.0));
    if (lon && lat) {
        // quick sanity checks
        log("DEBUG", "lon s(0)=" + std::to_string(lon->curve->value_evaluation(0,0)) + " v(0)=" + std::to_string(lon->curve->value_evaluation(0,1)) + " lat d(0)=" + std::to_string(lat->curve->value_evaluation(0,0)));
        std::vector<AD_algorithm::general::TrajectoryPoint> traj;
        try {
          traj = traj_evaluator_->CombineToCartesian(*lon, *lat, planner_params_.final_time_step, s_offset_,t_offset_, global_frenet_frame_);
        } catch (const std::exception& e) {
          std::cerr << "Combine failed: " << e.what() << std::endl;
          continue;
        }
        log("DEBUG", "Combine produced traj.size=" + std::to_string(traj.size()));
        std::string reason;
        // 找到满足约束的有效轨迹
        best_path_ = latticeFrenetPath(); // 注意：此处不保留 FrenetPath
        log("INFO", "Step 7: Stitching trajectory...");
        std::vector<AD_algorithm::general::TrajectoryPoint> final_trj;
        final_trj = traj_manager_.stitchTrajectory(traj);
        return final_trj;
    }
  }
  // 未找到满足约束的轨迹
  return {};
}


void latticePlanner::FilterLonCandidates(std::vector<LonCandidate>& lon_candidates) {
  const double dt = planner_params_.sampling.filter_dt;
  std::vector<AD_algorithm::general::FrenetPoint> speed_profile;
  std::vector<LonCandidate> filtered;
  filtered.reserve(lon_candidates.size());
  for (const auto& lon : lon_candidates) {
    const double T = lon.T;
    size_t expected = static_cast<size_t>(T / dt) + 1;
    speed_profile.clear();
    speed_profile.reserve(expected);
    // 曲线以相对 s (s=0 起点) 生成，因此直接使用 curve 输出作为相对 s
    for (double t = 0.0; t <= T + 1e-9; t += dt) {
      AD_algorithm::general::FrenetPoint fp;
      fp.t = t;
      fp.s = lon.curve->value_evaluation(t, 0); // relative s
      fp.s_dot = lon.curve->value_evaluation(t, 1);
      fp.s_dot_dot = lon.curve->value_evaluation(t, 2);
      speed_profile.push_back(fp);
    }
    std::string reason;
    if (traj_manager_.isSpeedProfileValid(speed_profile, &reason)) {
      filtered.push_back(lon);
    } else {
      log("INFO", "Lon candidate rejected by speed profile: " + reason + " (T=" + std::to_string(T) + ")");
    }
  }
  log("INFO", "FilterLonCandidates: kept " + std::to_string(filtered.size()) + " / " + std::to_string(lon_candidates.size()));
  lon_candidates.swap(filtered);
}

void latticePlanner::FilterLatCandidates(std::vector<LatCandidate>& lat_candidates, double s_offset) {
  std::vector<LatCandidate> filtered;
  filtered.reserve(lat_candidates.size());
  // 碰撞检测器进行碰撞检测
  for(auto & lat : lat_candidates) {
    if(collision_detection_ptr_->HasOverlapWithStaticObstacles(lat,s_offset)) {
      continue;
    }
    filtered.emplace_back(lat);
  }
  log("INFO", "FilterLatCandidates: kept " + std::to_string(filtered.size()) + " / " + std::to_string(lat_candidates.size()));
  lat_candidates.swap(filtered);
}

} // namespace planner
} // namespace AD_algorithm
