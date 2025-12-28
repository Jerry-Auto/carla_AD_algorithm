#include "general_modules/math_tool.h"

namespace AD_algorithm {
namespace general {

// 角度标准化
double normalize_angle(const double angle)
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

double sign(double x) { return x >= 0 ? 1 : -1; }

double p_t_l_distance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y) {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double length = std::hypot(dx, dy);
  const double proj = x0 * dx + y0 * dy;

  if (proj <= 0.0) {
    return std::hypot(x0, y0);
  }
  if (proj >= length * length) {
    return std::hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

double cross_prod(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  Vec2d v1{end_point_1.x - start_point.x, end_point_1.y - start_point.y};
  Vec2d v2{end_point_2.x - start_point.x, end_point_2.y - start_point.y};

  return v1.x * v2.y - v1.y * v2.x;
}

double inner_prod(const Vec2d &v1, const Vec2d &v2){
    return v1.x * v2.x + v1.y * v2.y;
}

double distance_to(const std::shared_ptr<LineSegment2d> &line_segment2d, const Vec2d &point) {
  if (line_segment2d->length <= kMathEpsilon) {
    return std::hypot(
        point.x - line_segment2d->start.x,
        point.y - line_segment2d->start.y);
  }
  const double x0 = point.x - line_segment2d->start.x;
  const double y0 = point.y - line_segment2d->start.y;
  const double proj = x0 * line_segment2d->unit_direction.x +
                      y0 * line_segment2d->unit_direction.y;
  if (proj <= 0.0) {
    return std::hypot(x0, y0);
  }
  if (proj >= line_segment2d->length) {
    return std::hypot(point.x - line_segment2d->end.x,
                      point.y - line_segment2d->end.y);
  }
  return std::abs(x0 * line_segment2d->unit_direction.y -
                  y0 * line_segment2d->unit_direction.x);
}

Vec2d rotate_point(const Vec2d& point, double rotation) {
    double s = std::sin(rotation);
    double c = std::cos(rotation);
    Vec2d V;
    V.x = point.x * c - point.y * s;
    V.y = point.x * s + point.y * c;
    return V;
}

std::vector<LineSegment2d> compute_rectangle_edges(
    double center_x, double center_y,
    double length, double width,
    double rotation
) {
    std::vector<LineSegment2d> edges;
    double half_length = length / 2.0;
    double half_width = width / 2.0;

    std::vector<Vec2d> local_corners = {
        Vec2d(-half_length, half_width),
        Vec2d(half_length, half_width),
        Vec2d(half_length, -half_width),
        Vec2d(-half_length, -half_width)
    };

    std::vector<Vec2d> world_corners;
    for (const auto& corner : local_corners) {
        Vec2d rotated = rotate_point(corner, rotation);
        world_corners.push_back(Vec2d(center_x + rotated.x, center_y + rotated.y));
    }

    for (size_t i = 0; i < world_corners.size(); ++i) {
        size_t next_i = (i + 1) % world_corners.size();
        LineSegment2d edge;
        edge.start = world_corners[i];
        edge.end = world_corners[next_i];

        Vec2d direction(edge.end.x - edge.start.x, edge.end.y - edge.start.y);
        edge.length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (edge.length > 0.0) {
            edge.unit_direction = Vec2d(direction.x / edge.length, direction.y / edge.length);
            edge.heading = std::atan2(direction.y, direction.x);
        } else {
            edge.unit_direction = Vec2d(0.0, 0.0);
            edge.heading = 0.0;
        }
        edges.push_back(edge);
    }
    return edges;
}

}}