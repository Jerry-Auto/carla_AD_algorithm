/* Copyright 2025 <Your Name> */

#include "general_modules/FrenetFrame.h"
#include <limits>
#include <cmath>
#include <algorithm>

namespace AD_algorithm {
namespace general {


PathPoint FrenetFrame::interpolate_by_s(double s) const {
    const auto& pts = ref_line_.get_path_points();
    size_t n = pts.size();
    if (n == 1) return pts[0];
    if (s <= pts.front().accumulated_s) return pts.front();
    if (s >= pts.back().accumulated_s) return pts.back();

    auto it = std::lower_bound(pts.begin(), pts.end(), s,
        [](const PathPoint& p, double val) { return p.accumulated_s < val; });
    size_t i = std::distance(pts.begin(), it) - 1;
    if (i >= n - 1) i = n - 2;

    const PathPoint& p0 = pts[i];
    const PathPoint& p1 = pts[i + 1];
    double ds = p1.accumulated_s - p0.accumulated_s;
    double t = (ds < 1e-12) ? 0.0 : (s - p0.accumulated_s) / ds;

    PathPoint interp;
    interp.x = p0.x + t * (p1.x - p0.x);
    interp.y = p0.y + t * (p1.y - p0.y);
    interp.heading = p0.heading + t * (p1.heading - p0.heading);
    interp.kappa = p0.kappa + t * (p1.kappa - p0.kappa);
    interp.kappa_rate = p0.kappa_rate + t * (p1.kappa_rate - p0.kappa_rate);
    interp.accumulated_s = s;
    return interp;
}

size_t FrenetFrame::find_nearest_index(double x, double y) const {
    const auto& path = ref_line_.get_path_points();
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        double dx = path[i].x - x;
        double dy = path[i].y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = i;
        }
    }
    return best_idx;
}

FrenetFrame::ProjectionResult FrenetFrame::project_to_path(double x, double y) const {
    size_t idx = find_nearest_index(x, y);
    const auto& path = ref_line_.get_path_points();
    const PathPoint& match = path[idx];

    Eigen::Vector2d point(x, y);
    Eigen::Vector2d r_match(match.x, match.y);
    Eigen::Vector2d tau(std::cos(match.heading), std::sin(match.heading));
    Eigen::Vector2d diff = point - r_match;
    double proj_len = diff.dot(tau);

    double s_proj = match.accumulated_s + proj_len;
    Eigen::Vector2d r_proj = r_match + proj_len * tau;
    Eigen::Vector2d nor(-std::sin(match.heading), std::cos(match.heading));
    double l = (point - r_proj).dot(nor);



    PathPoint ref_interp = interpolate_by_s(s_proj);
    double heading = ref_interp.heading;
    double kappa = ref_interp.kappa;
    Eigen::Vector2d tau_final(std::cos(heading), std::sin(heading));
    Eigen::Vector2d nor_final(-std::sin(heading), std::cos(heading));

    return {s_proj, l, r_proj, heading, kappa, tau_final, nor_final};
}

// ===== 新增：带状态记忆的匹配 =====
size_t FrenetFrame::find_nearest_index_with_memory(double x, double y, double /*heading*/) const {
    const auto& path = ref_line_.get_path_points();
    size_t n = path.size();
    if (n == 0) return 0;

    constexpr int kMaxForward = 50;
    constexpr int kMaxBackward = 30;
    constexpr double kThresh = 1e-2;

    size_t match_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    if (_is_first_run) {
        _is_first_run = false;
        int count = 0;
        for (size_t i = 0; i < n; ++i) {
            double d = std::hypot(x - path[i].x, y - path[i].y);
            if (d < min_dist) {
                min_dist = d;
                match_idx = i;
                count = 0;
            }
            if (++count > kMaxForward) break;
        }
    } else {
        size_t start_idx = std::min(_previous_match_index, n - 1);
        const PathPoint& start_pt = path[start_idx];

        Eigen::Vector2d tau(std::cos(start_pt.heading), std::sin(start_pt.heading));
        Eigen::Vector2d ego_vec(x - start_pt.x, y - start_pt.y);
        double dot = 0.0;
        if (ego_vec.norm() > 1e-6) dot = tau.dot(ego_vec.normalized());

        if (dot > kThresh) {
            int count = 0;
            for (size_t i = start_idx; i < n; ++i) {
                double d = std::hypot(x - path[i].x, y - path[i].y);
                if (d < min_dist) {
                    min_dist = d;
                    match_idx = i;
                    count = 0;
                }
                if (++count > kMaxForward) break;
            }
        } else if (dot < -kThresh) {
            int count = 0;
            for (int i = static_cast<int>(start_idx); i >= 0; --i) {
                double d = std::hypot(x - path[i].x, y - path[i].y);
                if (d < min_dist) {
                    min_dist = d;
                    match_idx = i;
                    count = 0;
                }
                if (++count > kMaxBackward) break;
            }
        } else {
            match_idx = start_idx;
        }
    }

    _previous_match_index = match_idx;
    return match_idx;
}

PathPoint FrenetFrame::get_matched_point(double x, double y, double heading)const{
    PathPoint matched_points=(*this)[find_nearest_index_with_memory(x,y,heading)];
    Eigen::Vector2d vec_d(x - matched_points.x, y - matched_points.y);
    Eigen::Vector2d tao(std::cos(matched_points.heading),std::sin(matched_points.heading));
    Eigen::Vector2d nor(-std::sin(matched_points.heading),std::cos(matched_points.heading));
    double es = tao.transpose() * vec_d;//纵向距离误差
    //double theta_r = matched_points.heading;//Apollo版本
    double theta_r = matched_points.heading + matched_points.kappa * es;//老王版本
    PathPoint target_point;
    target_point.x=matched_points.x+es*std::cos((theta_r+matched_points.heading)/2);
    target_point.y=matched_points.y+es*std::sin((theta_r+matched_points.heading)/2);
    target_point.heading=theta_r;
    target_point.kappa=matched_points.kappa;
    target_point.accumulated_s=matched_points.accumulated_s+es;
    target_point.kappa_rate=matched_points.kappa_rate;
    return target_point;
}

std::pair<TrajectoryPoint, double> FrenetFrame::get_matched_trj_point(double x, double y, double heading)const{
    size_t index=find_nearest_index_with_memory(x,y,heading);
    PathPoint matched_points=(*this)[index];
    TrajectoryPoint target_point;
    Eigen::Vector2d vec_d(x - matched_points.x, y - matched_points.y);
    Eigen::Vector2d tao(std::cos(matched_points.heading),std::sin(matched_points.heading));
    Eigen::Vector2d nor(-std::sin(matched_points.heading),std::cos(matched_points.heading));
    double es = tao.transpose() * vec_d;//纵向距离误差
    if(_trajectory_points.size()==ref_line_.size())
    {
        target_point=_trajectory_points[index];
    }
    else{
        //double theta_r = matched_points.heading;//Apollo版本
        double theta_r = matched_points.heading + matched_points.kappa * es;//老王版本
        target_point.x=matched_points.x+es*std::cos((theta_r+matched_points.heading)/2);
        target_point.y=matched_points.y+es*std::sin((theta_r+matched_points.heading)/2);
        target_point.heading=theta_r;
        target_point.kappa=matched_points.kappa;
    }
    return {target_point,matched_points.accumulated_s+es};
}


std::pair<TrajectoryPoint, double> FrenetFrame::get_matched_trj_point(double time_stamp) const {
    // 边界情况处理
    if (_trajectory_points.empty()) {
        // 返回一个默认值或抛出异常
        static std::pair<TrajectoryPoint, double> default_result;
        return default_result;
    }

    // 如果时间戳小于第一个点的时间，返回第一个点
    if (time_stamp <= _trajectory_points.front().time_stamped) {
        return std::make_pair(_trajectory_points.front(), (*this)[0].accumulated_s);
    }
    // 如果时间戳大于最后一个点的时间，返回最后一个点
    if (time_stamp >= _trajectory_points.back().time_stamped) {
        size_t last_idx = _trajectory_points.size() - 1;
        return std::make_pair(_trajectory_points.back(), (*this)[last_idx].accumulated_s);
    }
    // 二分查找找到第一个大于等于time_stamp的点
    size_t left = 0;
    size_t right = _trajectory_points.size() - 1;
    size_t index = 0;
    while (left <= right) {
        size_t mid = left + (right - left) / 2;
        
        if (_trajectory_points[mid].time_stamped < time_stamp) {
            left = mid + 1;
        } else {
            index = mid;
            right = mid - 1;
        }
    }
    return std::make_pair(_trajectory_points[index], (*this)[index].accumulated_s);
}
    
FrenetFrame::ProjectionResult FrenetFrame::project_to_path_with_memory(
    double x, double y, double heading) const {
    
    size_t idx = find_nearest_index_with_memory(x, y, heading);
    const auto& path = ref_line_.get_path_points();
    const PathPoint& match = path[idx];

    Eigen::Vector2d point(x, y);
    Eigen::Vector2d r_match(match.x, match.y);
    Eigen::Vector2d tau(std::cos(match.heading), std::sin(match.heading));
    double proj_len = (point - r_match).dot(tau);

    double s_proj = match.accumulated_s + proj_len;
    Eigen::Vector2d r_proj = r_match + proj_len * tau;
    Eigen::Vector2d nor(-std::sin(match.heading), std::cos(match.heading));
    double l = (point - r_proj).dot(nor);

    PathPoint ref_interp = interpolate_by_s(s_proj);
    double h = ref_interp.heading;
    double k = ref_interp.kappa;
    Eigen::Vector2d tau_f(std::cos(h), std::sin(h));
    Eigen::Vector2d nor_f(-std::sin(h), std::cos(h));

    return {s_proj, l, r_proj, h, k, tau_f, nor_f};
}

// ===== 转换接口（使用新投影）=====

FrenetPoint FrenetFrame::cartesian_to_frenet(double x, double y) const {
    auto proj = project_to_path(x, y);
    FrenetPoint fp;
    fp.s = proj.s;
    fp.l = proj.l;
    return fp;
}

FrenetPoint FrenetFrame::cartesian_to_frenet(const TrajectoryPoint& tp) const {
    auto proj = project_to_path_with_memory(tp.x, tp.y, tp.heading);

    double vx = std::cos(tp.heading) * tp.v;
    double vy = std::sin(tp.heading) * tp.v;
    Eigen::Vector2d v_cart(vx, vy);

    double c = 1.0 - proj.kappa * proj.l;
    double s_dot = v_cart.dot(proj.tau) / (c + 1e-9);
    double l_dot = v_cart.dot(proj.nor);
    double l_prime = (std::abs(s_dot) > 1e-5) ? l_dot / s_dot : 0.0;

    // 使用全量加速度投影来计算 s̈ 和 l̈（包含 κ' 与耦合项），更精确
    Eigen::Vector2d a_cart(tp.ax, tp.ay);
    double a_t = a_cart.dot(proj.tau);
    double a_n = a_cart.dot(proj.nor);

    PathPoint ref = interpolate_by_s(proj.s);
    double kappa = proj.kappa;
    double kappa_prime = ref.kappa_rate; // dκ/ds

    // a_n = l̈ + κ (1 - κ l) ṡ^2  => l̈ = a_n - κ (1 - κ l) ṡ^2
    double l_dot_dot = a_n - kappa * c * s_dot * s_dot;

    // a_t = (1 - κ l) s̈ - κ' l ṡ^2 - 2 κ ṡ l̇
    // => s̈ = (a_t + κ' l ṡ^2 + 2 κ ṡ l̇) / (1 - κ l)
    double s_dot_dot = (a_t + kappa_prime * proj.l * s_dot * s_dot + 2.0 * kappa * s_dot * l_dot) / (c + 1e-9);

    FrenetPoint fp;
    fp.t = tp.time_stamped;
    fp.s = proj.s;
    fp.l = proj.l;
    fp.s_dot = s_dot;
    fp.s_dot_dot = s_dot_dot;
    fp.l_dot = l_dot;
    fp.l_dot_dot = l_dot_dot;
    fp.l_prime = l_prime;

    // l''(s) = d^2 l / ds^2 = (l̈ * ṡ - l̇ * s̈) / ṡ^3
    if (std::abs(s_dot) > 1e-6) {
        fp.l_prime_prime = (l_dot_dot * s_dot - l_dot * s_dot_dot) / (s_dot * s_dot * s_dot);
    } else {
        fp.l_prime_prime = 0.0;
    }

    return fp;
}

FrenetPoint FrenetFrame::cartesian_to_frenet(const VehicleState& tp) const {
    auto proj = project_to_path_with_memory(tp.x, tp.y, tp.heading);

    double vx = std::cos(tp.heading) * tp.v;
    double vy = std::sin(tp.heading) * tp.v;
    Eigen::Vector2d v_cart(vx, vy);

    double c = 1.0 - proj.kappa * proj.l;
    double s_dot = v_cart.dot(proj.tau) / (c + 1e-9);
    double l_dot = v_cart.dot(proj.nor);
    double l_prime = (std::abs(s_dot) > 1e-5) ? l_dot / s_dot : 0.0;

    Eigen::Vector2d a_cart(tp.ax, tp.ay);
    double a_t = a_cart.dot(proj.tau);
    double a_n = a_cart.dot(proj.nor);

    PathPoint ref = interpolate_by_s(proj.s);
    double kappa = proj.kappa;
    double kappa_prime = ref.kappa_rate;

    double l_dot_dot = a_n - kappa * c * s_dot * s_dot;
    double s_dot_dot = (a_t + kappa_prime * proj.l * s_dot * s_dot + 2.0 * kappa * s_dot * l_dot) / (c + 1e-9);

    FrenetPoint fp;
    fp.t = tp.time_stamp;
    fp.s = proj.s;
    fp.l = proj.l;
    fp.s_dot = s_dot;
    fp.l_dot = l_dot;
    fp.l_prime = l_prime;
    fp.s_dot_dot = s_dot_dot;
    fp.l_dot_dot = l_dot_dot;

    // 计算 l''(s) 并保护 ṡ≈0 的情况
    if (std::abs(s_dot) > 1e-6) {
        fp.l_prime_prime = (l_dot_dot * s_dot - l_dot * s_dot_dot) / (s_dot * s_dot * s_dot);
    } else {
        fp.l_prime_prime = 0.0;
    }

    return fp;
}

std::vector<FrenetPoint> FrenetFrame::cartesian_to_frenet(const std::vector<TrajectoryPoint>& points) const {
    std::vector<FrenetPoint> res;
    res.reserve(points.size());
    for (const auto& p : points) res.push_back(cartesian_to_frenet(p));
    return res;
}

TrajectoryPoint FrenetFrame::frenet_to_cartesian(const FrenetPoint& fp) const {
    double s = fp.s;
    double l = fp.l;
    const auto& path = get_reference_path();
    double total_s = path.back().accumulated_s;
    TrajectoryPoint result;
    if (s < 0.0) {
        // 外推起点之前：使用起点姿态
        const PathPoint& p0 = path.front();
        double dx = -s * std::cos(p0.heading); // 注意：s<0，所以 -s>0，向后退
        double dy = -s * std::sin(p0.heading);
        // 法向偏移
        double nx = -std::sin(p0.heading);
        double ny =  std::cos(p0.heading);
        result.x = p0.x - dx + l * nx; // 注意方向：s<0 表示在起点前，所以减去 dx
        result.y = p0.y - dy + l * ny;
        result.heading = p0.heading; // 可选：是否保持 heading？
        result.kappa = p0.kappa;
        result.v = std::abs(fp.s_dot);
        result.a_tau = fp.s_dot_dot;
        const double a_n = result.v * result.v * result.kappa;
        result.ax = result.a_tau * std::cos(result.heading) - a_n * std::sin(result.heading);
        result.ay = result.a_tau * std::sin(result.heading) + a_n * std::cos(result.heading);
    } else if (s > total_s) {
        // 外推终点之后：使用终点姿态
        const PathPoint& pe = path.back();
        double ds = s - total_s; // 超出的距离
        double dx = ds * std::cos(pe.heading);
        double dy = ds * std::sin(pe.heading);
        double nx = -std::sin(pe.heading);
        double ny =  std::cos(pe.heading);
        result.x = pe.x + dx + l * nx;
        result.y = pe.y + dy + l * ny;
        result.heading = pe.heading;
        result.kappa = pe.kappa;
        result.v = std::abs(fp.s_dot);
        result.a_tau = fp.s_dot_dot;
        const double a_n = result.v * result.v * result.kappa;
        result.ax = result.a_tau * std::cos(result.heading) - a_n * std::sin(result.heading);
        result.ay = result.a_tau * std::sin(result.heading) + a_n * std::cos(result.heading);
    } else {
        PathPoint ref = interpolate_by_s(fp.s);
        Eigen::Vector2d pos(ref.x, ref.y);
        Eigen::Vector2d tau_ref(std::cos(ref.heading), std::sin(ref.heading));
        Eigen::Vector2d nor_ref(-std::sin(ref.heading), std::cos(ref.heading));
        pos += fp.l * nor_ref;
        double c = 1.0 - ref.kappa * fp.l;
        double u = fp.l_prime; // dl/ds
        double denom = c * c + u * u;
        double delta_theta = std::atan2(u, c);
        double heading = ref.heading + delta_theta;
        double v = std::hypot(fp.s_dot * c, fp.l_dot);
        // prepare derivatives
        double s_dot = fp.s_dot;
        double s_dot_dot = fp.s_dot_dot;
        double l_dot = fp.l_dot;
        double u_dot = fp.l_prime_prime * s_dot; // dl'/dt = (d^2 l / ds^2) * ds/dt
        // d/dt c = - (kappa' * s_dot * l + kappa * l_dot)
        double d_c = - (ref.kappa_rate * s_dot * fp.l + ref.kappa * l_dot);
        // 计算 r'' 在参考系的切向和法向分量
        // τ_comp = s̈ * c + ṡ * d_c - κ_ref * ṡ^2 * u
        // n_comp = s̈ * u + κ_ref * c * ṡ^2 + ṡ * u_dot
        double tau_comp = s_dot_dot * c + s_dot * d_c - ref.kappa * s_dot * s_dot * u;
        double nor_comp = s_dot_dot * u + ref.kappa * c * s_dot * s_dot + s_dot * u_dot;
        // 全局加速度
        result.x = pos.x();
        result.y = pos.y();
        result.heading = heading;
        result.v = v;
        result.ax = tau_comp * tau_ref.x() + nor_comp * nor_ref.x();
        result.ay = tau_comp * tau_ref.y() + nor_comp * nor_ref.y();
        // 切向加速度（投影到车辆朝向）
        result.a_tau = result.ax * std::cos(heading) + result.ay * std::sin(heading);
        // 计算航向变化率（θ̇ = κ_ref * ṡ + δθ̇）
        double delta_theta_dot = 0.0;
        if (denom > 1e-12) {
            delta_theta_dot = (u_dot * c - u * d_c) / denom;
        }
        double heading_dot = ref.kappa * s_dot + delta_theta_dot;
        // 曲率 κ = θ̇ / v
        // 注意：在低速情况下直接除以 v 会造成数值爆炸，使用更宽松的速度阈值作为退化条件
        const double kLowSpeedThreshold = 0.1; // m/s
        if (v > kLowSpeedThreshold) {
            result.kappa = heading_dot / v;
            // 额外检测：如果计算得到极大曲率，记录用于诊断
            if (std::abs(result.kappa) > 1000.0) {
                std::cerr << "[DEBUG] large computed kappa=" << result.kappa << " at t=" << fp.t
                          << " heading_dot=" << heading_dot << " v=" << v << " ref.kappa=" << ref.kappa << std::endl;
            }
        } else {
            // 低速或驻停：退化到参考线曲率，避免除零或噪声放大
            result.kappa = ref.kappa; // 低速退化到参考线曲率
            if (std::abs(heading_dot) > 1.0) {
                std::cerr << "[DEBUG] low-speed fallback: heading_dot=" << heading_dot << " v=" << v << " ref.kappa=" << ref.kappa << std::endl;
            }
        }
    }
    result.time_stamped = fp.t;
    return result;
}

std::vector<TrajectoryPoint> FrenetFrame::frenet_to_cartesian(const std::vector<FrenetPoint>& fps) const {
    std::vector<TrajectoryPoint> res;
    res.reserve(fps.size());
    for (const auto& fp : fps) res.push_back(frenet_to_cartesian(fp));
    return res;
}

std::vector<FrenetPoint> FrenetFrame::project_obstacle_to_frenet(const Obstacle& obstacle) const {
    double half_l = obstacle.length / 2.0;
    double half_w = obstacle.width / 2.0;
    double cos_h = std::cos(obstacle.heading);
    double sin_h = std::sin(obstacle.heading);
    
    // 计算矩形障碍物的四个角点
    std::vector<Vec2d> corners = {
        {obstacle.x + half_l * cos_h - half_w * sin_h, obstacle.y + half_l * sin_h + half_w * cos_h},
        {obstacle.x - half_l * cos_h - half_w * sin_h, obstacle.y - half_l * sin_h + half_w * cos_h},
        {obstacle.x - half_l * cos_h + half_w * sin_h, obstacle.y - half_l * sin_h - half_w * cos_h},
        {obstacle.x + half_l * cos_h + half_w * sin_h, obstacle.y + half_l * sin_h - half_w * cos_h}
    };
    
    std::vector<FrenetPoint> frenet_corners;
    frenet_corners.reserve(corners.size());
    for (const auto& corner : corners) {
        frenet_corners.push_back(cartesian_to_frenet(corner.x, corner.y));
    }
    return frenet_corners;
}

// 将一组 Frenet 多边形转换为 SLObstacle 列表（静态工具）
std::vector<SLObstacle> FrenetFrame::convertToSLObstacles(
    const std::vector<std::vector<FrenetPoint>>& frenet_obstacles,
    double safety_margin) {
    std::vector<SLObstacle> obstacles;
    for (const auto& corners : frenet_obstacles) {
        obstacles.emplace_back(corners, safety_margin);
    }
    return obstacles;
}

// 将 ST 图节点（使用 FrenetPoint 的 t/s 字段）转换为 STObstacle 列表（静态工具）
std::vector<STObstacle> FrenetFrame::convertToSTObstacles(
    const std::vector<std::vector<FrenetPoint>>& st_graph,
    double safety_margin) {
    std::vector<STObstacle> obstacles;
    for (const auto& points : st_graph) {
        obstacles.emplace_back(points, safety_margin);
    }
    return obstacles;
}

std::vector<FrenetPoint> FrenetFrame::project_dynamic_obstacle_to_frenet(const Obstacle& obstacle) const {
    double half_l = obstacle.length / 2.0;
    double half_w = obstacle.width / 2.0;
    double cos_h = std::cos(obstacle.heading);
    double sin_h = std::sin(obstacle.heading);
    
    // 1. 计算 4 个角点
    std::vector<Vec2d> corners = {
        {obstacle.x + half_l * cos_h - half_w * sin_h, obstacle.y + half_l * sin_h + half_w * cos_h},
        {obstacle.x - half_l * cos_h - half_w * sin_h, obstacle.y - half_l * sin_h + half_w * cos_h},
        {obstacle.x - half_l * cos_h + half_w * sin_h, obstacle.y - half_l * sin_h - half_w * cos_h},
        {obstacle.x + half_l * cos_h + half_w * sin_h, obstacle.y + half_l * sin_h - half_w * cos_h}
    };
    
    // 2. 计算中心点的 Frenet 速度
    TrajectoryPoint center_tp;
    center_tp.x = obstacle.x;
    center_tp.y = obstacle.y;
    center_tp.v = std::hypot(obstacle.vx, obstacle.vy);
    center_tp.heading = std::atan2(obstacle.vy, obstacle.vx);
    
    FrenetPoint center_fp = cartesian_to_frenet(center_tp);
    
    // 3. 投影 4 个角点，并赋予中心点的速度
    std::vector<FrenetPoint> frenet_corners;
    frenet_corners.reserve(corners.size());
    for (const auto& corner : corners) {
        FrenetPoint fp = cartesian_to_frenet(corner.x, corner.y);
        fp.s_dot = center_fp.s_dot;
        fp.l_dot = center_fp.l_dot;
        frenet_corners.push_back(fp);
    }
    return frenet_corners;
}

} // namespace general
} // namespace AD_algorithm