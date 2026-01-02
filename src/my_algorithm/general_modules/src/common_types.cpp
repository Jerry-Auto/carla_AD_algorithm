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

double SLObstacle::minDistanceTo(const std::shared_ptr<Polygon2d>& poly2d) const{
    return CollisionDetection::distance_to(polygon, poly2d);
}
    
bool SLObstacle::hasOverlap(const std::shared_ptr<Polygon2d>& poly2d) const{
    return CollisionDetection::has_overlap(polygon, poly2d);
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

// Obstacle 方法实现
Obstacle::Obstacle()
    : id(0), x(0.0), y(0.0), z(0.0), heading(0.0), vx(0.0), vy(0.0), vz(0.0),
      length(5.0), width(2.0), height(1.5) {
    // 延迟构造 polygon，避免在头文件中对不完整类型的依赖
    polygon = CollisionDetection::get_bounding_box(*this);
}

bool Obstacle::contains(double x, double y, double safety_margin) const {
    Vec2d point(x, y);
    std::shared_ptr<Polygon2d> poly_ptr;
    if (polygon) {
        poly_ptr = polygon;
    } else {
        poly_ptr = CollisionDetection::get_bounding_box(*this);
    }
    return CollisionDetection::distance_to(poly_ptr, point) <= safety_margin;
}

double Obstacle::minDistanceTo(double x, double y) const {
    Vec2d point(x, y);
    std::shared_ptr<Polygon2d> poly_ptr;
    if (polygon) {
        poly_ptr = polygon;
    } else {
        poly_ptr = CollisionDetection::get_bounding_box(*this);
    }
    return CollisionDetection::distance_to(poly_ptr, point);
}

double Obstacle::minDistanceTo(const std::shared_ptr<Polygon2d>& obs) const {
    std::shared_ptr<Polygon2d> poly_ptr = polygon ? polygon : CollisionDetection::get_bounding_box(*this);
    return CollisionDetection::distance_to(poly_ptr, obs);
}

double Obstacle::minDistanceTo(const Obstacle& box2d) const {
    std::shared_ptr<Polygon2d> poly_a = polygon ? polygon : CollisionDetection::get_bounding_box(*this);
    std::shared_ptr<Polygon2d> poly_b = box2d.polygon ? box2d.polygon : CollisionDetection::get_bounding_box(box2d);
    return CollisionDetection::distance_to(poly_a, poly_b);
}
double Obstacle::minDistanceTo(const TrajectoryPoint& trj_point) const{
    std::shared_ptr<Polygon2d> poly_ptr = polygon ? polygon : CollisionDetection::get_bounding_box(*this);
    std::shared_ptr<Polygon2d> box = CollisionDetection::get_bounding_box(trj_point);
    return CollisionDetection::distance_to(poly_ptr, box);
}

bool Obstacle::hasOverlap(const Obstacle& obs) const {
    std::shared_ptr<Polygon2d> poly_a = polygon ? polygon : CollisionDetection::get_bounding_box(*this);
    std::shared_ptr<Polygon2d> poly_b = obs.polygon ? obs.polygon : CollisionDetection::get_bounding_box(obs);
    return CollisionDetection::has_overlap(poly_a, poly_b);
}

bool Obstacle::hasOverlap(const std::shared_ptr<Polygon2d>& box2d) const {
    std::shared_ptr<Polygon2d> poly_a = polygon ? polygon : CollisionDetection::get_bounding_box(*this);
    return CollisionDetection::has_overlap(poly_a, box2d);
}

// 辅助函数实现


} // namespace general
} // namespace AD_algorithm