#include "emplanner/speed_planner.h"
#include "emplanner/DP_solver.h"
#include "general_modules/Trajectory.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include "general_modules/tic_toc.h"

using namespace AD_algorithm::general;

namespace AD_algorithm {
namespace planner {

namespace {
inline double safeMin(double a, double b) { return std::min(a, b); }
inline double safeMax(double a, double b) { return std::max(a, b); }
} // namespace

SpeedPlanner::SpeedPlanner(const WeightCoefficients& weights, const SpeedPlannerConfig& config,
                           std::shared_ptr<general::Logger> logger)
    : weights_(weights), config_(config), logger_(logger) {
    _qp_solver = std::make_shared<OsqpEigen::Solver>();
    _qp_solver->settings()->setWarmStart(true);
}

std::vector<FrenetPoint> SpeedPlanner::planSpeed(
        const FrenetFrame& ref_path_frame,
        const TrajectoryPoint& planning_start_point,
        double reference_speed,
        const std::vector<std::vector<FrenetPoint>>& dynamic_frenet_obstacles) {

    _dp_speed_profile.clear();
    _qp_speed_profile.clear();

    if (!config_.validate()) {
        log("ERROR", "Invalid configuration");
        return {};
    }

    // Make sure accel bounds are ordered (defensive)
    const double a_min = safeMin(config_.max_deceleration, config_.max_acceleration);
    const double a_max = safeMax(config_.max_deceleration, config_.max_acceleration);

    log("INFO", "Step 5: Speed planning...");

    general::TicToc timer;

    log("INFO", "Starting speed planning with reference speed: " + std::to_string(reference_speed));
    log("INFO", "Dynamic obstacles: " + std::to_string(dynamic_frenet_obstacles.size()));

    // 1. ST graph
    log("INFO", "Generating ST graph...");
    auto st_graph = generateSTGraph(dynamic_frenet_obstacles);
    log("INFO", "Generated ST graph with " + std::to_string(st_graph.size()) + " obstacles");

    // 2. Start point in ST
    FrenetPoint start_point;
    start_point.t = 0.0;
    start_point.s = 0.0;
    start_point.s_dot = planning_start_point.v;

    const double heading = planning_start_point.heading;
    start_point.s_dot_dot = planning_start_point.ax * std::cos(heading) +
                            planning_start_point.ay * std::sin(heading);
    start_point.s_dot_dot = general::clamp(start_point.s_dot_dot, a_min, a_max);

    log("INFO", "Planning start point: t=" + std::to_string(start_point.t) +
        ", s=" + std::to_string(start_point.s) +
        ", s_dot=" + std::to_string(start_point.s_dot) +
        ", s_dot_dot=" + std::to_string(start_point.s_dot_dot));

    // 3. DP speed planning
    log("INFO", "Running DP speed planning...");
    STState start_state(start_point.t, start_point.s, start_point.s_dot, start_point.s_dot_dot);

    std::vector<STObstacle> st_obstacles =
        FrenetFrame::convertToSTObstacles(st_graph, weights_.obs_safety_margin / 10.0);

    // Update s_max from reference path
    PathPoint last_path_point = ref_path_frame[ref_path_frame.size() - 1];
    config_.s_max = last_path_point.accumulated_s;

    auto cost_function = std::make_shared<SpeedCostFunction>(
        weights_, config_, reference_speed, st_obstacles);
    auto constraint_checker = std::make_shared<SpeedConstraintChecker>(
        config_, st_obstacles);
    auto sampling_strategy = std::make_shared<SpeedSamplingStrategy>(config_);
    auto backtrack_strategy = std::make_shared<DefaultBacktrackStrategy<STState>>();

    if (!_sample_grid) {
        _sample_grid = std::make_shared<std::vector<std::vector<STState>>>(
            sampling_strategy->generateSamplingGrid(start_state, 0));
    }

    DpPlannerConfig dp_config;
    int num_layers = static_cast<int>(_sample_grid->size() + 2);
    dp_config.max_layers = num_layers;
    dp_config.enable_pruning = true;
    dp_config.pruning_keep_num = 5;
    dp_config.debug_mode = false;

    SpeedDpPlanner dp_planner(cost_function, constraint_checker,
                              sampling_strategy, backtrack_strategy, dp_config);

    auto result = dp_planner.planWithGrid(*_sample_grid, start_state, STState());
    if (!result.success) {
        log("ERROR", "DP speed planning failed: " + result.message);
        return {};
    }

    log("DP speed planning succeeded, found profile with " +
        std::to_string(result.optimal_path.size()) + " points");
    log("INFO", "Total cost: " + std::to_string(result.total_cost));
    log("INFO", "Computation time: " + std::to_string(result.computation_time_ms) + " ms");

    for (const auto& state : result.optimal_path) {
        FrenetPoint p;
        p.t = state.t;
        p.s = state.s;
        p.s_dot = std::max(0.0, state.s_dot);
        p.s_dot_dot = general::clamp(state.s_dot_dot, a_min, a_max);
        _dp_speed_profile.push_back(p);
    }

    // ST densify before convex space/QP (stable integration densify)
    increaseSpeedProfile(_dp_speed_profile, config_.qp_dense_path_interval);

    // 4. Convex space bounds
    std::vector<double> s_lb, s_ub, s_dot_lb, s_dot_ub;
    generate_convex_space(ref_path_frame, st_obstacles, s_lb, s_ub, s_dot_lb, s_dot_ub);

    if (s_lb.empty() || s_ub.empty() || s_dot_lb.empty() || s_dot_ub.empty()) {
        log("Convex space constraints are empty, skipping QP", "WARN");
    } else {
        log("INFO", "Convex space constraints generated successfully");
        log("INFO", "s_lb size: " + std::to_string(s_lb.size()) + ", s_ub size: " + std::to_string(s_ub.size()));
    }

    // 5. QP smoothing
    log("INFO", "Running speed smoothing...");
    bool qp_success = QP_traj_optimal(s_lb, s_ub, s_dot_lb, s_dot_ub, reference_speed);

    if (!qp_success) {
        log("QP optimization failed, using DP result", "WARN");
        _qp_speed_profile.assign(_dp_speed_profile.begin(), _dp_speed_profile.end());
    }

    // enforce monotonic s
    for (size_t i = 1; i < _qp_speed_profile.size(); ++i) {
        if (_qp_speed_profile[i].s < _qp_speed_profile[i - 1].s) {
            _qp_speed_profile[i].s = _qp_speed_profile[i - 1].s;
            _qp_speed_profile[i].s_dot = std::max(0.0, _qp_speed_profile[i].s_dot);
            _qp_speed_profile[i].s_dot_dot = general::clamp(_qp_speed_profile[i].s_dot_dot, a_min, a_max);
        }
    }

    // 6. Final densify (stable integration densify)
    log("INFO", "Increasing speed profile density...");
    increaseSpeedProfile(_qp_speed_profile, config_.final_path_interval);

    double elapsed_ms = timer.toc();

    log("INFO", "Speed planning completed in " + std::to_string(static_cast<int>(elapsed_ms)) + " ms");
    log("INFO", "Generated " + std::to_string(_qp_speed_profile.size()) + " speed profile points");
    return _qp_speed_profile;
}

std::vector<std::vector<general::FrenetPoint>> SpeedPlanner::generateSTGraph(
    const std::vector<std::vector<FrenetPoint>>& dynamic_obstacles,
    double delta_l) {

    std::vector<std::vector<general::FrenetPoint>> st_graph;
    for (const auto& corners : dynamic_obstacles) {
        if (corners.empty()) continue;

        double s_dot = corners[0].s_dot;
        double l_dot = corners[0].l_dot;

        double s_min = std::numeric_limits<double>::max();
        double s_max = -std::numeric_limits<double>::max();
        double l_min = std::numeric_limits<double>::max();
        double l_max = -std::numeric_limits<double>::max();

        for (const auto& p : corners) {
            s_min = std::min(s_min, p.s);
            s_max = std::max(s_max, p.s);
            l_min = std::min(l_min, p.l);
            l_max = std::max(l_max, p.l);
        }

        double s_center = (s_min + s_max) / 2.0;
        double l_center = (l_min + l_max) / 2.0;
        double half_s = (s_max - s_min) / 2.0;
        double half_l = (l_max - l_min) / 2.0;
        double effective_delta_l = delta_l + half_l;

        // Strict static obstacle filter
        if (std::abs(s_dot) < 0.5) {
            if (l_min > delta_l || l_max < -delta_l) {
                continue;
            }
        }

        // Ignore very small lateral velocity obstacles
        if (std::abs(l_dot) <= 0.5) {
            continue;
        }

        double t_in = 0.0, t_out = 0.0;
        if (std::abs(l_center) > effective_delta_l) {
            if (l_center * l_dot > 0) {
                continue;
            } else {
                t_in = std::abs(l_center / l_dot) - std::abs(effective_delta_l / l_dot);
                t_out = std::abs(l_center / l_dot) + std::abs(effective_delta_l / l_dot);
            }
        } else {
            t_in = 0.0;
            if (l_dot > 0) {
                t_out = (effective_delta_l - l_center) / l_dot;
            } else {
                t_out = (-effective_delta_l - l_center) / l_dot;
            }
        }

        if (t_out - t_in < 1.5) continue;
        if (t_out >= 8.0 || t_out <= 1.0) continue;

        double s_at_tin = s_center + s_dot * t_in;
        double s_at_tout = s_center + s_dot * t_out;

        double safety_s = 0.5;
        double total_half_s = half_s + safety_s;

        std::vector<general::FrenetPoint> st_polygon(4);
        st_polygon[0].t = t_in;  st_polygon[0].s = s_at_tin - total_half_s;
        st_polygon[1].t = t_in;  st_polygon[1].s = s_at_tin + total_half_s;
        st_polygon[2].t = t_out; st_polygon[2].s = s_at_tout + total_half_s;
        st_polygon[3].t = t_out; st_polygon[3].s = s_at_tout - total_half_s;

        st_graph.push_back(st_polygon);
    }
    return st_graph;
}

void SpeedPlanner::generate_convex_space(
    FrenetFrame ref_path_frenet,
    const std::vector<general::STObstacle>& st_obstacles,
    std::vector<double>& s_lb, std::vector<double>& s_ub,
    std::vector<double>& s_dot_lb, std::vector<double>& s_dot_ub) {

    s_lb.clear();
    s_ub.clear();
    s_dot_lb.clear();
    s_dot_ub.clear();

    const auto& speed_profile = _dp_speed_profile;
    if (speed_profile.empty()) {
        throw std::runtime_error("Speed profile is empty");
    }

    const auto& ref_path_points = ref_path_frenet.get_reference_path();
    const double max_path_s = ref_path_points.back().accumulated_s;

    // Start constraints
    s_lb.emplace_back(speed_profile.front().s);
    s_ub.emplace_back(speed_profile.front().s);
    s_dot_lb.emplace_back(speed_profile.front().s_dot);
    s_dot_ub.emplace_back(speed_profile.front().s_dot);

    // Speed bounds (curvature-based)
    for (size_t i = 1; i < speed_profile.size(); i++) {
        double cur_s = speed_profile[i].s;
        PathPoint path_point;
        if (cur_s >= max_path_s) {
            path_point = ref_path_points.back();
        } else {
            // NOTE: still index-based; ideally query by s
            path_point = ref_path_frenet[i];
        }

        double cur_kappa = path_point.kappa;
        double kappa_abs = std::abs(cur_kappa);

        // 这个参数极大限制过弯速度
        double speed_limit = config_.max_speed;
        if (kappa_abs > 1e-6) {
            speed_limit = std::sqrt(config_.max_lateral_acc / kappa_abs);
            speed_limit *= 0.95; // optional safety factor
            // key: minimum speed in curves to avoid crawling
            speed_limit = std::max(speed_limit, config_.min_curve_speed);
        }
        
        s_dot_ub.emplace_back(std::min(speed_limit, config_.max_speed));
        s_dot_lb.emplace_back(0.0);

        if (s_dot_lb[i] > s_dot_ub[i]) {
            double mid = (s_dot_lb[i] + s_dot_ub[i]) / 2.0;
            s_dot_lb[i] = 0.0;
            s_dot_ub[i] = std::max(mid, 0.5);
        }
    }

    // Base position bounds
    for (size_t i = 1; i < speed_profile.size(); i++) {
        s_ub.emplace_back(max_path_s);
        s_lb.emplace_back(0.0);
    }

    // Obstacle-induced position bounds
    for (size_t i = 1; i < speed_profile.size(); i++) {
        double t_i = speed_profile[i].t;
        double s_i = speed_profile[i].s;

        double nearest_upper_bound = std::numeric_limits<double>::max();
        double nearest_lower_bound = std::numeric_limits<double>::lowest();

        for (const auto& obs : st_obstacles) {
            if (!obs.polygon) continue;
            const auto& pts = obs.polygon->points();
            if (pts.size() < 4) continue;

            double t_start = pts[0].x;
            double t_end = pts[2].x;
            if (t_i < t_start || t_i > t_end) continue;

            double ratio = (t_i - t_start) / (t_end - t_start + 1e-9);
            double obs_lower_bound = pts[0].y + ratio * (pts[3].y - pts[0].y);
            double obs_upper_bound = pts[1].y + ratio * (pts[2].y - pts[1].y);

            if (s_i > obs_upper_bound) {
                nearest_lower_bound = std::max(nearest_lower_bound, obs_upper_bound);
            } else if (s_i < obs_lower_bound) {
                nearest_upper_bound = std::min(nearest_upper_bound, obs_lower_bound);
            } else {
                double overlap = std::min(s_i - obs_lower_bound, obs_upper_bound - s_i);
                if (overlap < 0.5) continue;

                double dist_to_lower = std::abs(s_i - obs_lower_bound);
                double dist_to_upper = std::abs(s_i - obs_upper_bound);
                if (dist_to_lower < dist_to_upper) {
                    nearest_upper_bound = std::min(nearest_upper_bound, std::max(s_i, obs_lower_bound));
                } else {
                    nearest_lower_bound = std::max(nearest_lower_bound, std::min(s_i, obs_upper_bound));
                }
            }
        }

        if (nearest_upper_bound == std::numeric_limits<double>::max()) nearest_upper_bound = max_path_s;
        if (nearest_lower_bound == std::numeric_limits<double>::lowest()) nearest_lower_bound = 0.0;

        s_lb[i] = std::max(s_lb[i], nearest_lower_bound);
        s_ub[i] = std::min(s_ub[i], nearest_upper_bound);

        if (s_lb[i] > s_ub[i]) {
            double middle = 0.5 * (s_lb[i] + s_ub[i]);
            s_lb[i] = middle;
            s_ub[i] = middle;
            log("WARN", "约束冲突 index=", i, " adjusted to ", middle);
        }
    }
}

// ===== Stable densify (integration-based) =====
// This avoids polynomial overshoot of acceleration inside each segment.
void SpeedPlanner::increaseSpeedProfile(std::vector<general::FrenetPoint>& prof, double /*interval*/) {
    if (prof.size() < 2) return;

    const double a_min = safeMin(config_.max_deceleration, config_.max_acceleration);
    const double a_max = safeMax(config_.max_deceleration, config_.max_acceleration);

    std::vector<general::FrenetPoint> out;
    out.reserve(prof.size() * 10);

    constexpr double kMinDt = 1e-3;
    constexpr double kDtBase = 0.05; // 20Hz time resolution

    for (size_t i = 0; i + 1 < prof.size(); ++i) {
        const auto& p0 = prof[i];
        const auto& p1 = prof[i + 1];

        const double T_raw = p1.t - p0.t;
        if (!std::isfinite(T_raw) || T_raw <= kMinDt) continue;

        int steps = static_cast<int>(std::ceil(T_raw / kDtBase));
        if (steps < 1) steps = 1;
        const double dt = T_raw / steps;

        double t = p0.t;
        double s = p0.s;
        double v = std::max(0.0, p0.s_dot);
        double a = general::clamp(p0.s_dot_dot, a_min, a_max);

        for (int k = 0; k < steps; ++k) {
            general::FrenetPoint fp;
            fp.t = t;
            fp.s = s;
            fp.s_dot = v;
            fp.s_dot_dot = a;
            out.push_back(fp);

            // Guide acceleration toward next segment's endpoint speed, but keep within limits
            const double v_target = std::max(0.0, p1.s_dot);
            const double a_cmd = (v_target - v) / std::max(dt, 1e-2);
            a = general::clamp(a_cmd, a_min, a_max);

            // Semi-implicit integration (more stable than explicit)
            v = std::max(0.0, v + a * dt);
            s = s + v * dt;
            t = t + dt;
        }
    }

    out.push_back(prof.back());
    prof.swap(out);
}

bool SpeedPlanner::QP_traj_optimal(const std::vector<double>& s_lb, const std::vector<double>& s_ub,
                                  const std::vector<double>& s_dot_lb, const std::vector<double>& s_dot_ub,
                                  const double reference_speed) {
    const size_t point_num = _dp_speed_profile.size();
    if (point_num < 2) {
        log("ERROR", "Not enough points for QP optimization");
        return false;
    }

    if (s_lb.size() != point_num || s_ub.size() != point_num ||
        s_dot_lb.size() != point_num || s_dot_ub.size() != point_num) {
        log("ERROR", "Constraint vector size mismatch");
        return false;
    }

    const double a_min = safeMin(config_.max_deceleration, config_.max_acceleration);
    const double a_max = safeMax(config_.max_deceleration, config_.max_acceleration);

    log("INFO", "Starting QP optimization for speed profile with " + std::to_string(point_num) + " points");

    // 1) Cost
    Eigen::SparseMatrix<double> H_total(3 * point_num, 3 * point_num);

    const double total_speed_weight = weights_.speed_qp_w_ref_speed + weights_.speed_qp_w_target_speed;
    for (size_t i = 0; i < point_num; i++) {
        H_total.coeffRef(3 * i + 1, 3 * i + 1) += 2.0 * total_speed_weight;
        H_total.coeffRef(3 * i + 2, 3 * i + 2) += 2.0 * weights_.speed_qp_w_a;
    }

    constexpr double kMinDt = 1e-3;
    for (size_t i = 0; i < point_num - 1; i++) {
        const double dt_raw = _dp_speed_profile[i + 1].t - _dp_speed_profile[i].t;
        const double dt = (std::isfinite(dt_raw) && dt_raw > kMinDt) ? dt_raw : kMinDt;
        const double w = 2.0 * weights_.speed_qp_w_jerk / (dt * dt);

        H_total.coeffRef(3 * i + 2, 3 * i + 2) += w;
        H_total.coeffRef(3 * i + 2, 3 * (i + 1) + 2) -= w;
        H_total.coeffRef(3 * (i + 1) + 2, 3 * i + 2) -= w;
        H_total.coeffRef(3 * (i + 1) + 2, 3 * (i + 1) + 2) += w;
    }

    Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * point_num);
    for (size_t i = 0; i < point_num; i++) {
        f[3 * i + 1] = -2.0 * total_speed_weight * reference_speed;
    }

    // 2) Constraints
    Eigen::SparseMatrix<double> A_continuity(2 * (point_num - 1), 3 * point_num);
    for (size_t i = 0; i < point_num - 1; i++) {
        const double dt_raw = _dp_speed_profile[i + 1].t - _dp_speed_profile[i].t;
        const double dt = (std::isfinite(dt_raw) && dt_raw > kMinDt) ? dt_raw : kMinDt;

        const int r = static_cast<int>(2 * i);
        const int c = static_cast<int>(3 * i);

        A_continuity.insert(r, c + 0) = 1.0;
        A_continuity.insert(r, c + 1) = dt;
        A_continuity.insert(r, c + 2) = 0.5 * dt * dt;
        A_continuity.insert(r, c + 3) = -1.0;

        A_continuity.insert(r + 1, c + 1) = 1.0;
        A_continuity.insert(r + 1, c + 2) = dt;
        A_continuity.insert(r + 1, c + 4) = -1.0;
    }
    Eigen::VectorXd continuity_bound = Eigen::VectorXd::Zero(2 * (point_num - 1));

    Eigen::SparseMatrix<double> A_start(3, 3 * point_num);
    A_start.insert(0, 0) = 1.0;
    A_start.insert(1, 1) = 1.0;
    A_start.insert(2, 2) = 1.0;
    Eigen::VectorXd start_val(3);
    start_val << _dp_speed_profile[0].s, _dp_speed_profile[0].s_dot, general::clamp(_dp_speed_profile[0].s_dot_dot, a_min, a_max);

    Eigen::SparseMatrix<double> A_speed_limit(point_num, 3 * point_num);
    Eigen::VectorXd speed_lower(point_num), speed_upper(point_num);
    for (size_t i = 0; i < point_num; i++) {
        A_speed_limit.insert(static_cast<int>(i), static_cast<int>(3 * i + 1)) = 1.0;
        speed_lower[i] = 0.0;
        speed_upper[i] = std::numeric_limits<double>::infinity();
    }

    Eigen::SparseMatrix<double> A_convex(2 * point_num, 3 * point_num);
    Eigen::VectorXd convex_lower(2 * point_num), convex_upper(2 * point_num);
    for (size_t i = 0; i < point_num; i++) {
        A_convex.insert(static_cast<int>(2 * i), static_cast<int>(3 * i + 0)) = 1.0;
        convex_lower[2 * i] = s_lb[i];
        convex_upper[2 * i] = s_ub[i];

        A_convex.insert(static_cast<int>(2 * i + 1), static_cast<int>(3 * i + 1)) = 1.0;
        convex_lower[2 * i + 1] = s_dot_lb[i];
        convex_upper[2 * i + 1] = s_dot_ub[i];
    }

    // Hard accel bounds
    Eigen::SparseMatrix<double> A_acc_limit(point_num, 3 * point_num);
    Eigen::VectorXd acc_lower(point_num), acc_upper(point_num);
    for (size_t i = 0; i < point_num; i++) {
        A_acc_limit.insert(static_cast<int>(i), static_cast<int>(3 * i + 2)) = 1.0;
        acc_lower[i] = a_min;
        acc_upper[i] = a_max;
    }

    const int total_rows =
        A_continuity.rows() + A_start.rows() + A_speed_limit.rows() + A_convex.rows() + A_acc_limit.rows();

    Eigen::SparseMatrix<double> A_total_T(3 * point_num, total_rows);
    int current_col = 0;
    A_total_T.middleCols(current_col, A_continuity.rows()) = A_continuity.transpose(); current_col += A_continuity.rows();
    A_total_T.middleCols(current_col, A_start.rows()) = A_start.transpose(); current_col += A_start.rows();
    A_total_T.middleCols(current_col, A_speed_limit.rows()) = A_speed_limit.transpose(); current_col += A_speed_limit.rows();
    A_total_T.middleCols(current_col, A_convex.rows()) = A_convex.transpose(); current_col += A_convex.rows();
    A_total_T.middleCols(current_col, A_acc_limit.rows()) = A_acc_limit.transpose();
    Eigen::SparseMatrix<double> A_total = A_total_T.transpose();

    Eigen::VectorXd lower_total(total_rows), upper_total(total_rows);
    int row = 0;

    lower_total.segment(row, continuity_bound.size()) = continuity_bound;
    upper_total.segment(row, continuity_bound.size()) = continuity_bound;
    row += static_cast<int>(continuity_bound.size());

    lower_total.segment(row, start_val.size()) = start_val;
    upper_total.segment(row, start_val.size()) = start_val;
    row += static_cast<int>(start_val.size());

    lower_total.segment(row, speed_lower.size()) = speed_lower;
    upper_total.segment(row, speed_upper.size()) = speed_upper;
    row += static_cast<int>(speed_lower.size());

    lower_total.segment(row, convex_lower.size()) = convex_lower;
    upper_total.segment(row, convex_upper.size()) = convex_upper;
    row += static_cast<int>(convex_lower.size());

    lower_total.segment(row, acc_lower.size()) = acc_lower;
    upper_total.segment(row, acc_upper.size()) = acc_upper;

    // 3) Solve
    try {
        _qp_solver->clearSolver();
        _qp_solver->data()->clearHessianMatrix();
        _qp_solver->data()->clearLinearConstraintsMatrix();
        _qp_solver->settings()->setVerbosity(false);
        _qp_solver->settings()->setMaxIteration(4000);

        _qp_solver->data()->setNumberOfVariables(static_cast<int>(3 * point_num));
        _qp_solver->data()->setNumberOfConstraints(total_rows);

        if (!_qp_solver->data()->setHessianMatrix(H_total)) return false;
        if (!_qp_solver->data()->setGradient(f)) return false;
        if (!_qp_solver->data()->setLinearConstraintsMatrix(A_total)) return false;
        if (!_qp_solver->data()->setLowerBound(lower_total)) return false;
        if (!_qp_solver->data()->setUpperBound(upper_total)) return false;

        if (!_qp_solver->initSolver()) return false;
        if (_qp_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            log("ERROR", "QP solver failed");
            return false;
        }

        auto solution = _qp_solver->getSolution();
        _qp_speed_profile.clear();
        _qp_speed_profile.reserve(point_num);

        for (size_t i = 0; i < point_num; i++) {
            FrenetPoint p;
            p.t = _dp_speed_profile[i].t;
            p.s = solution[3 * i + 0];
            p.s_dot = std::max(0.0, solution[3 * i + 1]);
            p.s_dot_dot = general::clamp(solution[3 * i + 2], a_min, a_max);
            _qp_speed_profile.emplace_back(p);
        }

        return true;
    } catch (const std::exception& e) {
        log("ERROR", "QP optimization exception: " + std::string(e.what()));
        return false;
    }
}

} // namespace planner
} // namespace AD_algorithm