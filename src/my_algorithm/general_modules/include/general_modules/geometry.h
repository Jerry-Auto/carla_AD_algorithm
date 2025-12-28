#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

namespace AD_algorithm {
namespace general {

struct Vec2d {
    double x = 0.0;
    double y = 0.0;
    Vec2d() = default;
    Vec2d(double x_, double y_) : x(x_), y(y_) {}

    Vec2d operator+(const Vec2d& other) const { return {x + other.x, y + other.y}; }
    Vec2d operator-(const Vec2d& other) const { return {x - other.x, y - other.y}; }
    Vec2d operator*(double scalar) const { return {x * scalar, y * scalar}; }
    double DistanceTo(const Vec2d& other) const { return std::hypot(x - other.x, y - other.y); }
    void Normalize() {
        double len = std::hypot(x, y);
        if (len > 1e-9) {
            x /= len;
            y /= len;
        }
    }
};

struct AABB {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

struct LineSegment2d {
    Vec2d start;
    Vec2d end;
    Vec2d unit_direction;
    double heading = 0.0;
    double length = 0.0;

    LineSegment2d() = default;
    LineSegment2d(const Vec2d& start_, const Vec2d& end_) : start(start_), end(end_) {
        Vec2d diff = end - start;
        length = std::hypot(diff.x, diff.y);
        if (length > 1e-9) {
            unit_direction = {diff.x / length, diff.y / length};
            heading = std::atan2(diff.y, diff.x);
        }
    }
};

struct Polygon2d {
    std::vector<LineSegment2d> edges; 
    
    Polygon2d() = default;
    Polygon2d(const std::vector<Vec2d>& points) {
        if (points.size() < 3) return;
        for (size_t i = 0; i < points.size(); ++i) {
            edges.emplace_back(points[i], points[(i + 1) % points.size()]);
        }
    }

    virtual ~Polygon2d() = default;
    virtual std::vector<Vec2d> points() const {
        std::vector<Vec2d> pts;
        for (const auto& edge : edges) {
            pts.push_back(edge.start);
        }
        return pts;
    }

    virtual AABB aabb() const {
        AABB res;
        if (edges.empty()) return res;
        res.min_x = res.max_x = edges[0].start.x;
        res.min_y = res.max_y = edges[0].start.y;
        for (const auto& edge : edges) {
            res.min_x = std::min({res.min_x, edge.start.x, edge.end.x});
            res.max_x = std::max({res.max_x, edge.start.x, edge.end.x});
            res.min_y = std::min({res.min_y, edge.start.y, edge.end.y});
            res.max_y = std::max({res.max_y, edge.start.y, edge.end.y});
        }
        return res;
    }

    virtual bool is_convex() const { return true; } // 默认 AD 中多边形多为凸
};

struct Box2d : public Polygon2d {
    Vec2d center;
    double length = 0.0;
    double width = 0.0;
    double half_length = 0.0;
    double half_width = 0.0;
    double heading = 0.0;
    double cos_heading = 1.0;
    double sin_heading = 0.0;

    Box2d() = default;
    Box2d(const Vec2d& center_, double length_, double width_, double heading_)
        : center(center_), length(length_), width(width_), heading(heading_) {
        half_length = length / 2.0;
        half_width = width / 2.0;
        cos_heading = std::cos(heading);
        sin_heading = std::sin(heading);
        
        std::vector<Vec2d> corners;
        double x_corners[] = {half_length, -half_length, -half_length, half_length};
        double y_corners[] = {half_width, half_width, -half_width, -half_width};
        
        for(int i=0; i<4; ++i) {
            double x = x_corners[i];
            double y = y_corners[i];
            corners.push_back({
                x * cos_heading - y * sin_heading + center.x,
                x * sin_heading + y * cos_heading + center.y
            });
        }

        for (size_t i = 0; i < corners.size(); ++i) {
            edges.emplace_back(corners[i], corners[(i + 1) % corners.size()]);
        }
    }
};

struct Triangle2d : public Polygon2d {
    Triangle2d() = default;
    Triangle2d(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3) {
        std::vector<Vec2d> vertices = {p1, p2, p3};
        for (size_t i = 0; i < vertices.size(); ++i) {
            edges.emplace_back(vertices[i], vertices[(i + 1) % vertices.size()]);
        }
    }
};

struct Ellipse {
    Vec2d center;
    double a = 0.0; // 长半轴
    double b = 0.0; // 短半轴
    double heading = 0.0; // 旋转角度
};

} // namespace general
} // namespace AD_algorithm
