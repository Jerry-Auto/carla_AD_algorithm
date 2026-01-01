#include "lattice/lattice_planner.h"
#include <iostream>

namespace AD_algorithm {
namespace planner {

latticePlanner::latticePlanner() {
    // 使用结构化参数初始化各个模块组件
    collision_detection_ptr_=std::make_shared<latticeCollisionDetection>();
    planner_params_ = PlannerParams();
    traj1d_generator_ = std::make_unique<Trajectory1DGenerator>(planner_params_);
    traj_evaluator_ = std::make_unique<TrajectoryEvaluator>(planner_params_);
    traj_combiner_ = std::make_unique<TrajectoryCombiner>();
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
  traj_evaluator_ = std::make_unique<TrajectoryEvaluator>(planner_params_);
  traj_combiner_ = std::make_unique<TrajectoryCombiner>();
  // 重新设置限制
  traj_manager_.setLimits(planner_params_.limits.max_speed,
                          planner_params_.limits.max_acc,
                          planner_params_.limits.max_curvature,
                          planner_params_.limits.max_jerk);
  desired_speed_ = planner_params_.cruise_speed;
}
void latticePlanner::GetStValues(latticeFrenetPoint& frenet_point, 
    const std::shared_ptr<AD_algorithm::general::PolynomialCurve> st_polynomial,
    double t) {
  frenet_point.s = st_polynomial->value_evaluation(t, 0);
  frenet_point.s_dot = st_polynomial->value_evaluation(t, 1);
  frenet_point.s_dot_dot = st_polynomial->value_evaluation(t, 2);
  frenet_point.s_d_d_d = st_polynomial->value_evaluation(t, 3);
  frenet_point.v = desired_speed_;
}

bool latticePlanner::setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) {
  if (reference_line.size() < 2) {
    log("Failed to set reference line: too few points");
    return false;
  }

  // 缓存参考路径并构建 FrenetFrame（参考 EMPlanner 的实现）
  try {
    global_reference_line_ = reference_line;
    // 从 PathPoints 构建 ReferenceLine 并创建 FrenetFrame
    // 注意：测试环境下关闭参考线的二次平滑，避免平滑器在直线路径上引入不必要的曲率震荡
    AD_algorithm::general::ReferenceLine ref_line(reference_line, false);
    global_frenet_frame_ = std::make_shared<AD_algorithm::general::FrenetFrame>(ref_line);
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
  if (!collision_detection_ptr_) return false;
  // 委托给 TrajectoryManager 进行约束校验
  return traj_manager_.isTrajectoryValid(trajectory, reason, min_points) && !collision_detection_ptr_->InCollision(trajectory);
}

void latticePlanner::set_log_enable(bool enable) {
  log_enable_ = enable;
  PlannerBase::set_log_enable(enable);
}

void latticePlanner::GetLtValues(latticeFrenetPoint& frenet_point, 
    const std::shared_ptr<AD_algorithm::general::PolynomialCurve> lt_polynomial,
    double t) {
  frenet_point.l = lt_polynomial->value_evaluation(t, 0);
  frenet_point.l_dot = lt_polynomial->value_evaluation(t, 1);
  frenet_point.l_dot_dot = lt_polynomial->value_evaluation(t, 2);
  frenet_point.l_d_d_d = lt_polynomial->value_evaluation(t, 3);
}

double latticePlanner::GetStObjectCost(const latticeFrenetPath& frenet_path, 
                                       const double& target_speed) {
  double object_cost = 0.0, speed_cost = 0.0;
  double time_square_sum = 0.1, dist_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    speed_cost += pow(frenet_point.t, 2) * fabs(target_speed - frenet_point.s_dot);
    time_square_sum += pow(frenet_point.t, 2);
  }
  speed_cost = speed_cost / time_square_sum;
  if (!frenet_path.frenet_points.empty()) {
      dist_cost = 1.0 / (1.0 + frenet_path.frenet_points.back().s);
  }
  object_cost = (speed_cost + 10 * dist_cost) / 11;

  return object_cost;
}

double latticePlanner::GetStJerkCost(const latticeFrenetPath& frenet_path) {
  double st_jerk_cost = 0.0, jerk_square_sum = 0.0, jerk_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    jerk_square_sum += pow(frenet_point.s_d_d_d / 4.0, 2);
    jerk_abs_sum += fabs(frenet_point.s_d_d_d / 4.0);
  }
  if (jerk_abs_sum > 1e-6) {
      st_jerk_cost = jerk_square_sum / jerk_abs_sum;
  }
  
  return st_jerk_cost;
}

double latticePlanner::GetLtOffsetCost(const latticeFrenetPath& frenet_path, 
                                       const latticeFrenetPoint& initial_frenet_point) {
  double lt_offset_cost = 0.0, offset_square_sum = 0.0, offset_abs_sum = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    // 反向则放大代价值
    if (frenet_point.l * initial_frenet_point.l < 0.0) {
      offset_square_sum += pow(frenet_point.l / 3.5, 2) * 5;
      offset_abs_sum += fabs(frenet_point.l / 3.5) * 5;
    }
    else {
      offset_square_sum += pow(frenet_point.l / 3.5, 2);
      offset_abs_sum += fabs(frenet_point.l / 3.5);
    }
  }
  if (offset_abs_sum > 1e-6) {
      lt_offset_cost = offset_square_sum / offset_abs_sum;
  }
  
  return lt_offset_cost;
}

double latticePlanner::GetLtAccCost(const latticeFrenetPath& frenet_path) {
  double max_acc = 0.0, lt_acc_cost = 0.0;
  for (auto& frenet_point : frenet_path.frenet_points) {
    if (fabs(max_acc) < fabs(frenet_point.l_dot_dot)) {
      max_acc = frenet_point.l_dot_dot;
    }
  }
  lt_acc_cost = fabs(max_acc);

  return lt_acc_cost;
}

std::vector<latticeFrenetPath> latticePlanner::SamplingFollowingFrenetPaths(
    const latticeFrenetPoint& initial_frenet_point, 
    const latticeFrenetPoint& leader_frenet_point) {
  std::vector<latticeFrenetPath> frenet_paths;
  for (double t_i = planner_params_.sampling.sample_min_time; t_i <= planner_params_.sampling.sample_max_time;) {
    t_i += planner_params_.sampling.sample_time_step;
    
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    st_polynomial->curve_fitting(
        0.0, initial_frenet_point.s, initial_frenet_point.s_dot, 0.0,
        t_i, leader_frenet_point.s - 8.0, std::min(planner_params_.cruise_speed, leader_frenet_point.v), 0.0
    );
    
    for (double l_i = -0.5 * planner_params_.sampling.sample_lat_width; l_i <= 0.5 * planner_params_.sampling.sample_lat_width;) {
      l_i += planner_params_.sampling.sample_width_length;
      
      auto lt_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
      lt_polynomial->curve_fitting(
          0.0, initial_frenet_point.l, initial_frenet_point.l_dot, initial_frenet_point.l_dot_dot,
          t_i, l_i, 0.0, 0.0
      );
      
      latticeFrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      for (double t = 0; t < t_i; t += 0.02) {
        latticeFrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial, t);
        GetLtValues(frenet_point, lt_polynomial, t);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_dot > frenet_path.max_speed ? 
            frenet_point.s_dot : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_dot_dot > frenet_path.max_acc ? 
            frenet_point.s_dot_dot : frenet_path.max_acc;
      }
      double st_object_cost = GetStObjectCost(frenet_path, planner_params_.cruise_speed);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = planner_params_.weights.weight_st_object * st_object_cost +
                         planner_params_.weights.weight_st_jerk * st_jerk_cost + 
                         planner_params_.weights.weight_lt_offset * lt_offset_cost + 
                         planner_params_.weights.weight_lt_acc * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}

std::vector<latticeFrenetPath> latticePlanner::SamplingCruisingFrenetPaths(
    const latticeFrenetPoint& initial_frenet_point) {
  std::vector<latticeFrenetPath> frenet_paths;
  for (double t_i = planner_params_.sampling.sample_min_time; t_i <= planner_params_.sampling.sample_max_time;) {
    t_i += planner_params_.sampling.sample_time_step;

    // 纵向使用四次多项式（用于巡航）
    auto st_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
    // 使用类似五次样条的拟合（8 个参数）以兼容已安装的 PolynomialCurve。
    // 用 s + v * t 对终点位置做简单线性预测
    double est_end_s = initial_frenet_point.s + planner_params_.cruise_speed * t_i;
    st_polynomial->curve_fitting(
      0.0, initial_frenet_point.s, initial_frenet_point.s_dot, 0.0,
      t_i, est_end_s, planner_params_.cruise_speed, 0.0
    );

    for (double l_i = -1 * planner_params_.sampling.sample_lat_width; l_i <= planner_params_.sampling.sample_lat_width;) {
      l_i += planner_params_.sampling.sample_width_length;
      
      // 横向使用五次多项式拟合
      auto lt_polynomial = std::make_shared<AD_algorithm::general::PolynomialCurve>();
      lt_polynomial->curve_fitting(
          0.0, initial_frenet_point.l, initial_frenet_point.l_dot, initial_frenet_point.l_dot_dot,
          t_i, l_i, 0.0, 0.0
      );

      latticeFrenetPath frenet_path;
      frenet_path.max_speed = std::numeric_limits<double>::min();
      frenet_path.max_acc = std::numeric_limits<double>::min();
      
      for (double t = 0; t <= t_i; t += 0.02) {
        latticeFrenetPoint frenet_point;
        frenet_point.t = t;
        GetStValues(frenet_point, st_polynomial, t);
        GetLtValues(frenet_point, lt_polynomial, t);
        frenet_path.frenet_points.push_back(frenet_point);
        
        frenet_path.max_speed = frenet_point.s_dot > frenet_path.max_speed ? 
            frenet_point.s_dot : frenet_path.max_speed;
        frenet_path.max_acc = frenet_point.s_dot_dot > frenet_path.max_acc ? 
            frenet_point.s_dot_dot : frenet_path.max_acc;
      }

      double st_object_cost = GetStObjectCost(frenet_path, planner_params_.cruise_speed);
      double st_jerk_cost = GetStJerkCost(frenet_path);
      double lt_offset_cost = GetLtOffsetCost(frenet_path, initial_frenet_point);
      double lt_acc_cost = GetLtAccCost(frenet_path);
      frenet_path.cost = planner_params_.weights.weight_st_object * st_object_cost +
                         planner_params_.weights.weight_st_jerk * st_jerk_cost + 
                         planner_params_.weights.weight_lt_offset * lt_offset_cost + 
                         planner_params_.weights.weight_lt_acc * lt_acc_cost;
      frenet_paths.push_back(frenet_path);
    }
  }
  return frenet_paths;
}

void latticePlanner::GetCartesianPaths(std::vector<latticeFrenetPath>& frenet_paths, 
                                       const std::vector<PathPoint>& ref_path) {
  if (ref_path.empty()) return;

  // 优先使用缓存的 FrenetFrame（如果存在且基于同一参考路径）
  // 简化处理：若存在缓存则使用，否则临时构造一个。
  AD_algorithm::general::FrenetFrame frenet_frame = (global_frenet_frame_ ? *global_frenet_frame_ : AD_algorithm::general::FrenetFrame([&]() {
      std::vector<std::pair<double, double>> xy_points;
      xy_points.reserve(ref_path.size());
      for (const auto& p : ref_path) xy_points.push_back({p.x, p.y});
      return AD_algorithm::general::ReferenceLine(xy_points);
  }()));

  for (auto& frenet_path : frenet_paths) {
    frenet_path.size_ = 0;
    for (size_t i = 0; i < frenet_path.frenet_points.size(); i++) {
      if (frenet_path.frenet_points[i].s >= ref_path.back().accumulated_s) {
        break;
      }
      frenet_path.size_++;
    }
    for (int i = 0; i< frenet_path.size_; i++) {
      // 使用 FrenetFrame 将 Frenet 坐标转换为笛卡尔坐标
      // FrenetFrame::frenet_to_cartesian 接受 general::FrenetPoint
      // 我们的 latticeFrenetPoint 继承自 general::FrenetPoint，因此可以直接使用
      auto traj_point = frenet_frame.frenet_to_cartesian(frenet_path.frenet_points[i]);
      
      frenet_path.frenet_points[i].x = traj_point.x;
      frenet_path.frenet_points[i].y = traj_point.y;
      frenet_path.frenet_points[i].yaw = traj_point.heading;
      frenet_path.frenet_points[i].cur = traj_point.kappa;
      frenet_path.frenet_points[i].v = desired_speed_;
    }
  }
}

std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> 
    latticePlanner::GetValidPaths(std::vector<latticeFrenetPath>& frenet_paths, 
                                  const latticeFrenetPoint& leader_frenet_point, 
                                  const bool& is_car_followed) {
  std::priority_queue<latticeFrenetPath, std::vector<latticeFrenetPath>, Cmp> valid_paths;
  for (auto& frenet_path : frenet_paths) {
    if (frenet_path.max_speed < planner_params_.limits.max_speed && 
        frenet_path.max_acc < planner_params_.limits.max_acc && 
        frenet_path.max_curvature < planner_params_.limits.max_curvature && 
        !collision_detection_ptr_->IsCollision(frenet_path, 
                                               leader_frenet_point,
                                               is_car_followed)) {
      valid_paths.push(frenet_path);
    }
  }
  return valid_paths;
}

std::vector<latticeFrenetPath> latticePlanner::GetCandidatePaths(
    const std::vector<PathPoint>& ref_path, const latticeFrenetPoint& initial_frenet_point,
    const latticeFrenetPoint& leader_frenet_point, const bool& is_car_followed) {
  std::vector<latticeFrenetPath> frenet_paths;
  if (is_car_followed) {
    desired_speed_ = std::min(planner_params_.cruise_speed, leader_frenet_point.v);
    frenet_paths = std::move(SamplingFollowingFrenetPaths(initial_frenet_point, 
                                                          leader_frenet_point));
  } 
  else {
    desired_speed_ = planner_params_.cruise_speed;
    frenet_paths = std::move(SamplingCruisingFrenetPaths(initial_frenet_point));
  }
  GetCartesianPaths(frenet_paths, ref_path);
  auto&& valid_paths = 
      GetValidPaths(frenet_paths, leader_frenet_point, is_car_followed);

  std::vector<latticeFrenetPath> planning_paths;
  while (!valid_paths.empty()) {
    planning_paths.push_back(valid_paths.top());
    valid_paths.pop();
  }
  return planning_paths;
}


std::vector<AD_algorithm::general::TrajectoryPoint> latticePlanner::plan(
    const std::shared_ptr<AD_algorithm::general::VehicleState>& ego_state,
    const std::vector<AD_algorithm::general::Obstacle>& obstacles,
    double reference_speed,
    double current_time) {

  log("INFO", "Starting planning cycle at time: " + std::to_string(current_time));
  
  traj_manager_.setCurrentTime(current_time);

  traj_manager_.updateEgoState(ego_state);

  // 更新碰撞检测器（使用当前障碍物与参考路径）
  if (collision_detection_ptr_) collision_detection_ptr_->Update(obstacles, global_reference_line_);
  // 为后续有效性检查设置 TrajectoryManager 的当前时间（如需要）
  
  // 2. 计算规划起点
  log("INFO", "Step 2: Calculating planning start point...");
  AD_algorithm::general::TrajectoryPoint planning_start_point = traj_manager_.calculatePlanningStartPoint(ego_state,planner_params_.plan_time_step);
  

  // 将 规划起点转换到 Frenet 坐标系
  auto ego_fp = global_frenet_frame_->cartesian_to_frenet(planning_start_point);
  // 记录绝对 s0 / t0，并将 init 归一化为相对坐标（s=0, t=0）用于采样
  double s0 = ego_fp.s;
  double time_base = planning_start_point.time_stamped;
  latticeFrenetPoint init;
  init.t = 0.0; init.s = 0.0; init.s_dot = ego_fp.s_dot; init.s_dot_dot = ego_fp.s_dot_dot;
  init.l = ego_fp.l; init.l_dot = ego_fp.l_dot; init.l_dot_dot = ego_fp.l_dot_dot;

  // 检测前车（简单启发式方法） - 基于 s0 计算相对纵向距离
  latticeFrenetPoint leader_fp;
  bool has_leader = false;
  double min_ds = 1e9;
  for (const auto& obs : obstacles) {
    AD_algorithm::general::TrajectoryPoint otp; otp.x = obs.x; otp.y = obs.y; otp.v = obs.getSpeed(); otp.heading = obs.heading;
    auto ofp = global_frenet_frame_->cartesian_to_frenet(otp);
    double ds = ofp.s - s0; // relative to planning start
    if (ds > 0 && std::abs(ofp.l) < 3.5 && ds < min_ds) {
      min_ds = ds; has_leader = true;
      leader_fp.s = ofp.s - s0; // store leader.s as relative
      leader_fp.s_dot = ofp.s_dot; leader_fp.l = ofp.l;
      leader_fp.x = otp.x; leader_fp.y = otp.y;
    }
  }

  // 生成一维候选，使用的相对时间，以规划起点为 t=0
  std::vector<Trajectory1DGenerator::LonCandidate> lon_candidates;
  if (has_leader) {
    lon_candidates = traj1d_generator_->GenerateLongitudinalFollowing(init, leader_fp);
  } else {
    lon_candidates = traj1d_generator_->GenerateLongitudinalCruising(init);
  }

  // 用成员方法进行初筛（优化：重用缓冲区，使用 planner_params_.sampling.sample_dt）
  FilterLonCandidates(init, lon_candidates);

  // 对每个 T 采样横向候选（使用第一个纵向候选的 T 作为代表）
  std::vector<Trajectory1DGenerator::LatCandidate> lat_candidates;
  if (!lon_candidates.empty()) {
    double T = lon_candidates.front().T;
    lat_candidates = traj1d_generator_->GenerateLateralCandidates(init, T);
  }

  // 用成员方法进行横向初筛（传入已构造的 frame 由内部使用）
  // 这里传入 s0 以在需要时恢复绝对 s
  FilterLatCandidates(lat_candidates, s0);

  if (lon_candidates.empty() || lat_candidates.empty()) {
    log("No candidates generated");
    return {};
  }

  // 对纵横候选对进行排序
  auto q = traj_evaluator_->RankPairs(lon_candidates, lat_candidates, reference_speed);

  // 逐个迭代并选择第一个通过约束的轨迹
  while (!q.empty()) {
    auto top = q.top(); q.pop();
    // 将 s0 传给 Combine，以便在调用 frenet_to_cartesian 前恢复绝对 s
    auto traj = traj_combiner_->Combine(*top.lon, *top.lat, global_reference_line_, s0);
    // 把 traj 的时间戳从相对时间转换为以 planning start time 为基准的绝对时间
    for (auto &p : traj) p.time_stamped += time_base;

    if (collision_detection_ptr_->InCollision(traj)) {
      log("Trajectory rejected by collision");
      continue;
    }
    std::string reason;
    if (!traj_manager_.isTrajectoryValid(traj, &reason)) {
      log("Trajectory rejected by constraint: ", reason);
      continue;
    }
    // 找到满足约束的有效轨迹
    best_path_ = latticeFrenetPath(); // 注意：此处不保留 FrenetPath
    log("INFO", "Step 7: Stitching trajectory...");
    return traj_manager_.stitchTrajectory(traj);
  }
  // 未找到满足约束的轨迹
  return {};
}


// --- Helper filter implementations ---
void latticePlanner::FilterLonCandidates(const latticeFrenetPoint& init,
                                        std::vector<Trajectory1DGenerator::LonCandidate>& lon_candidates) {
  (void)init;
  const double dt = planner_params_.sampling.sample_dt;
  std::vector<Trajectory1DGenerator::LonCandidate> filtered;
  std::vector<AD_algorithm::general::FrenetPoint> speed_profile;

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
      log("Dropped lon candidate due to speed profile invalid: ", reason);
    }
  }
  lon_candidates.swap(filtered);
}

void latticePlanner::FilterLatCandidates(std::vector<Trajectory1DGenerator::LatCandidate>& lat_candidates,double s_offset) {
  const double dt = planner_params_.sampling.sample_dt;
  std::vector<Trajectory1DGenerator::LatCandidate> filtered;
  std::vector<AD_algorithm::general::FrenetPoint> lateral_profile;
  std::vector<AD_algorithm::general::TrajectoryPoint> path_traj;

  // 准备 FrenetFrame：优先使用缓存的 global_frenet_frame_，若不存在则尝试基于 global_reference_line_ 构造临时 frame
  if (!global_frenet_frame_ && global_reference_line_.empty()) {
    log("FilterLatCandidates skipped: no FrenetFrame or reference line available");
    lat_candidates.clear();
    return;
  }
  const AD_algorithm::general::FrenetFrame& frenet_frame = (global_frenet_frame_ ? *global_frenet_frame_ : AD_algorithm::general::FrenetFrame(AD_algorithm::general::ReferenceLine(global_reference_line_)));

  filtered.reserve(lat_candidates.size());
  for (const auto& lat : lat_candidates) {
    // 使用与横向参数 s 等距采样（避免依赖时间窗 T）
    double max_s = lat.param_s;
    if (max_s <= 1e-6) continue;
    double ds = std::max(planner_params_.sampling.sample_space_resolution, desired_speed_ * dt); // 使用配置的空间采样步长，最小以速度换算保底
    size_t expected = static_cast<size_t>(max_s / ds) + 1;
    lateral_profile.clear();
    lateral_profile.reserve(expected);
    for (double s = 0.0; s <= max_s + 1e-9; s += ds) {
      AD_algorithm::general::FrenetPoint fp;
      double eval_s = s;
      double l_val = lat.curve->value_evaluation(eval_s, 0);     // d(s)
      double l_prime_s = lat.curve->value_evaluation(eval_s, 1); // dl/ds
      double l_pprime_s = lat.curve->value_evaluation(eval_s, 2); // d2l/ds2
      fp.l_prime = l_prime_s;
      fp.l_prime_prime = l_pprime_s;
      fp.l = l_val;
      fp.s = s;
      lateral_profile.push_back(fp);
    }

    // 使用批量转换（Frenet -> Cartesian）以避免逐点开销
    try {
      // 在转换前把相对 s 恢复为绝对 s
      for (auto &fp : lateral_profile) fp.s += s_offset;
      path_traj = frenet_frame.frenet_to_cartesian(lateral_profile);
      std::string reason;
      if (traj_manager_.isPathValid(path_traj, &reason, 5)) {
        filtered.push_back(lat);
      } else {
        log("Dropped lat candidate due to path invalid: ", reason);
      }
    } catch (const std::exception& e) {
      log("Dropped lat candidate due to exception in path conversion: ", e.what());
    }
  }
  lat_candidates.swap(filtered);
}

} // namespace planner
} // namespace AD_algorithm
