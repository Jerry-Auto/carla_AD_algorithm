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
STObstacle::STObstacle(const std::vector<STPoint>& points, double margin)
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
std::vector<SLObstacle> convertToSLObstacles(
    const std::vector<std::vector<FrenetPoint>>& frenet_obstacles,
    double safety_margin) {

    std::vector<SLObstacle> obstacles;
    for (const auto& corners : frenet_obstacles) {
        obstacles.emplace_back(corners, safety_margin);
    }
    return obstacles;
}

std::vector<STObstacle> convertToSTObstacles(
    const std::vector<std::vector<STPoint>>& st_graph,
    double safety_margin) {

    std::vector<STObstacle> obstacles;
    for (const auto& points : st_graph) {
        obstacles.emplace_back(points, safety_margin);
    }
    return obstacles;
}

// SLTObstacle 构造函数实现
SLTObstacle::SLTObstacle(const std::vector<SLTPoint>& points, double margin)
    : safety_margin(margin) {
    // 对于 SLT 空间，我们使用 2D 多边形表示 (s,l) 投影，时间作为属性
    // 这里简化处理，将所有点的 (s,l) 作为多边形的顶点
    std::vector<Vec2d> vertices;
    for (const auto& p : points) {
        vertices.emplace_back(p.s, p.l);
    }
    polygon = std::make_shared<Polygon2d>(vertices);
}

SLTObstacle::SLTObstacle(double s_min, double s_max, double l_min, double l_max, 
                        double t_min, double t_max, double margin)
    : safety_margin(margin) {
    std::vector<Vec2d> points = {
        {s_min, l_min}, {s_max, l_min}, {s_max, l_max}, {s_min, l_max}
    };
    polygon = std::make_shared<Polygon2d>(points);
    // 注意：这里没有存储时间范围，实际应用中可能需要扩展
}

bool SLTObstacle::contains(double s, double l, double t, double safety_margin) const {
    if (!polygon) return false;
    Vec2d point(s, l);
    double dist = CollisionDetection::distance_to(polygon, point);
    return dist <= safety_margin;
}

double SLTObstacle::minDistanceTo(double s, double l, double t) const {
    if (!polygon) return std::numeric_limits<double>::max();
    Vec2d point(s, l);
    return CollisionDetection::distance_to(polygon, point);
}

} // namespace general
} // namespace AD_algorithm