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

    FrenetPoint fp;
    fp.s = proj.s;
    fp.l = proj.l;
    fp.s_dot = s_dot;
    fp.l_dot = l_dot;
    fp.l_prime = l_prime;
    fp.s_dot_dot = tp.a_tau;
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

    // 计算加速度在 Frenet 坐标系中的切向分量
    Eigen::Vector2d a_cart(tp.ax, tp.ay);
    double s_dot_dot = a_cart.dot(proj.tau) / (c + 1e-9);

    FrenetPoint fp;
    fp.s = proj.s;
    fp.l = proj.l;
    fp.s_dot = s_dot;
    fp.l_dot = l_dot;
    fp.l_prime = l_prime;
    fp.s_dot_dot = s_dot_dot;
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
        return result;
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
        return result;
    } else {
    PathPoint ref = interpolate_by_s(fp.s);
    Eigen::Vector2d pos(ref.x, ref.y);
    pos += fp.l * Eigen::Vector2d(-std::sin(ref.heading), std::cos(ref.heading));

    double c = 1.0 - ref.kappa * fp.l;
    double delta_theta = std::atan2(fp.l_prime, c);
    double heading = ref.heading + delta_theta;
    double v = std::hypot(fp.s_dot * c, fp.l_dot);

    TrajectoryPoint tp;
    tp.x = pos.x();
    tp.y = pos.y();
    tp.heading = heading;
    tp.v = v;
    tp.kappa = ref.kappa;
    tp.a_tau = fp.s_dot_dot;
    // 将加速度分解为：切向 a_t + 向心 a_n，并投影到全局坐标 (x,y)
    const double a_n = tp.v * tp.v * tp.kappa;
    tp.ax = tp.a_tau * std::cos(tp.heading) - a_n * std::sin(tp.heading);
    tp.ay = tp.a_tau * std::sin(tp.heading) + a_n * std::cos(tp.heading);
    return tp;
    }
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