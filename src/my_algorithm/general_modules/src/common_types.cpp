/* Copyright 2025 <Your Name> */

#include "general_modules/common_types.h"
#include "general_modules/collision_detection.h" 

namespace AD_algorithm {
namespace general {

// SLObstacle 构造函数实现
SLObstacle::SLObstacle(const std::vector<FrenetPoint>& corners, double margin)
    : safety_margin(margin) {
    std::vector<Vec2d> points;
    for (const auto& cp : corners) {
        points.emplace_back(cp.s, cp.l);
    }
    // 使用 Polygon2d 表示 SL 平面上的障碍物
    polygon = std::make_shared<Polygon2d>(points);
}

SLObstacle::SLObstacle(double s, double l, double length, double width, double margin)
    : safety_margin(margin) {
    double s_min = s - length / 2.0;
    double s_max = s + length / 2.0;
    double l_min = l - width / 2.0;
    double l_max = l + width / 2.0;

    std::vector<Vec2d> points = {
        {s_min, l_min}, {s_max, l_min}, {s_max, l_max}, {s_min, l_max}
    };
    polygon = std::make_shared<Polygon2d>(points);
}

bool SLObstacle::contains(double s, double l, double safety_margin) const {
    if (!polygon) return false;
    Vec2d point(s, l);
    double dist = CollisionDetection::distance_to(polygon, point);
    return dist <= safety_margin;
}

double SLObstacle::minDistanceTo(double s, double l) const {
    if (!polygon) return std::numeric_limits<double>::max();
    Vec2d point(s, l);
    return CollisionDetection::distance_to(polygon, point);
}

// STObstacle 构造函数实现
STObstacle::STObstacle(const std::vector<FrenetPoint>& points, double margin)
    : safety_margin(margin) {
    std::vector<Vec2d> vertices;
    for (const auto& p : points) {
        vertices.emplace_back(p.t, p.s);
    }
    polygon = std::make_shared<Polygon2d>(vertices);
}

STObstacle::STObstacle(double t_start, double t_end, double s_start, double s_end, double margin)
    : safety_margin(margin) {
    std::vector<Vec2d> vertices;
    vertices.emplace_back(t_start, s_start);
    vertices.emplace_back(t_start, s_end);
    vertices.emplace_back(t_end, s_end);
    vertices.emplace_back(t_end, s_start);
    polygon = std::make_shared<Polygon2d>(vertices);
}

bool STObstacle::contains(double t, double s, double safety_margin) const {
    if (!polygon) return false;
    Vec2d point(t, s);
    double dist = CollisionDetection::distance_to(polygon, point);
    return dist <= safety_margin;
}

double STObstacle::minDistanceTo(double t, double s) const {
    if (!polygon) return std::numeric_limits<double>::max();
    Vec2d point(t, s);
    return CollisionDetection::distance_to(polygon, point);
}

// 辅助函数实现




} // namespace general
} // namespace AD_algorithm