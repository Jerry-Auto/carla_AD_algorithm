#include "general_modules/collision_detection.h"
#include <memory>

namespace AD_algorithm {
namespace general {

double CollisionDetection::distance_to(const std::shared_ptr<Polygon2d> &box1, const std::shared_ptr<Polygon2d> &box2) {
    if (auto b1 = std::dynamic_pointer_cast<Box2d>(box1)) {
        if (auto b2 = std::dynamic_pointer_cast<Box2d>(box2)) {
            return distance_to(b1, b2);
        }
        if (auto t2 = std::dynamic_pointer_cast<Triangle2d>(box2)) {
            return distance_to(b1, t2);
        }
    }
    if (auto t1 = std::dynamic_pointer_cast<Triangle2d>(box1)) {
        if (auto t2 = std::dynamic_pointer_cast<Triangle2d>(box2)) {
            return distance_to(t1, t2);
        }
        if (auto b2 = std::dynamic_pointer_cast<Box2d>(box2)) {
            return distance_to(b2, t1);
        }
    }
    
    if (has_overlap(box1, box2)) {
        return 0.0;
    }
    
    double min_dist = std::numeric_limits<double>::max();
    const auto& points1 = box1->points();
    for (size_t i = 0; i < points1.size(); ++i) {
        auto edge1 = std::make_shared<LineSegment2d>(points1[i], points1[(i + 1) % points1.size()]);
        min_dist = std::min(min_dist, distance_to(box2, edge1));
    }
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Polygon2d> &bounding_box, const Vec2d &point) {
    if (auto box_ptr = std::dynamic_pointer_cast<Box2d>(bounding_box)) {
        return distance_to(box_ptr, point);
    }
    if (auto triangle_ptr = std::dynamic_pointer_cast<Triangle2d>(bounding_box)) {
        return distance_to(triangle_ptr, point);
    }
    
    if (is_point_in(bounding_box, point)) {
        return 0.0;
    }
    
    double min_dist = std::numeric_limits<double>::max();
    const auto& points = bounding_box->points();
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[(i + 1) % points.size()];
        min_dist = std::min(min_dist, p_t_l_distance(point.x, point.y, p1.x, p1.y, p2.x, p2.y));
    }
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Polygon2d> &bounding_box, const std::shared_ptr<LineSegment2d> &line_segment) {
    if (auto box_ptr = std::dynamic_pointer_cast<Box2d>(bounding_box)) {
        return distance_to(box_ptr, line_segment);
    }
    if (auto triangle_ptr = std::dynamic_pointer_cast<Triangle2d>(bounding_box)) {
        return distance_to(triangle_ptr, line_segment);
    }
    
    if (has_overlap(bounding_box, line_segment)) {
        return 0.0;
    }
    
    double min_dist = std::numeric_limits<double>::max();
    
    // Distance from line segment endpoints to polygon
    min_dist = std::min(min_dist, distance_to(bounding_box, line_segment->start));
    min_dist = std::min(min_dist, distance_to(bounding_box, line_segment->end));
    
    // Distance from polygon vertices to line segment
    const auto& points = bounding_box->points();
    for (const auto& p : points) {
        min_dist = std::min(min_dist, AD_algorithm::general::distance_to(line_segment, p));
    }
    
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Box2d> &bounding_box, const Vec2d &point) {
  const double x0 = point.x - bounding_box->center.x;
  const double y0 = point.y - bounding_box->center.y;
  const double dx =
      std::abs(x0 * bounding_box->cos_heading + y0 * bounding_box->sin_heading) -
      bounding_box->half_length;
  const double dy =
      std::abs(x0 * bounding_box->sin_heading - y0 * bounding_box->cos_heading) -
      bounding_box->half_width;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return std::hypot(dx, dy);
}

double CollisionDetection::distance_to(const std::shared_ptr<Box2d> &bounding_box,
                                      const std::shared_ptr<LineSegment2d> &line_segment) {

  if (line_segment->length <= kMathEpsilon) {
    return distance_to(bounding_box, line_segment->start);
  }

  const double ref_x1 = line_segment->start.x - bounding_box->center.x;
  const double ref_y1 = line_segment->start.y - bounding_box->center.y;
  double x1 =
      ref_x1 * bounding_box->cos_heading + ref_y1 * bounding_box->sin_heading;
  double y1 =
      ref_x1 * bounding_box->sin_heading - ref_y1 * bounding_box->cos_heading;
  double box_x = bounding_box->half_length;
  double box_y = bounding_box->half_width;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }

  const double ref_x2 = line_segment->end.x - bounding_box->center.x;
  const double ref_y2 = line_segment->end.y - bounding_box->center.y;
  double x2 =
      ref_x2 * bounding_box->cos_heading + ref_y2 * bounding_box->sin_heading;
//   double x2_tmp = x2; // Avoid confusion
  double y2 =
      ref_x2 * bounding_box->sin_heading - ref_y2 * bounding_box->cos_heading;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return p_t_l_distance(box_x, box_y, x1, y1, x2, y2);
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : p_t_l_distance(box_x, box_y, x1, y1, x2, y2);
      case 2:
        return (x1 > x2) ? p_t_l_distance(box_x, -box_y, x1, y1, x2, y2)
                         : p_t_l_distance(box_x, box_y, x1, y1, x2, y2);
      case -1:
        return cross_prod({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : p_t_l_distance(box_x, -box_y, x1, y1, x2, y2);
      case -4:
        return cross_prod({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? p_t_l_distance(box_x, -box_y, x1, y1, x2, y2)
                   : (cross_prod({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : p_t_l_distance(-box_x, box_y, x1, y1, x2, y2));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : p_t_l_distance(box_x, box_y, x1, y1, x2, y2);
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return cross_prod({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : p_t_l_distance(box_x, box_y, x1, y1, x2, y2);
      case -3:
        return 0.0;
    }
  }
  return 0.0;
}

bool CollisionDetection::is_point_in(const std::shared_ptr<Box2d> &bounding_box, const Vec2d &point) {
  const double x0 = point.x - bounding_box->center.x;
  const double y0 = point.y - bounding_box->center.y;
  const double dx =
      std::abs(x0 * bounding_box->cos_heading + y0 * bounding_box->sin_heading);
  const double dy =
      std::abs(-x0 * bounding_box->sin_heading + y0 * bounding_box->cos_heading);
  return dx <= bounding_box->half_length + kMathEpsilon &&
         dy <= bounding_box->half_width + kMathEpsilon;
}

bool CollisionDetection::has_overlap(const std::shared_ptr<Box2d> &bounding_box, const std::shared_ptr<LineSegment2d> &line_segment) {
  if (line_segment->length <= kMathEpsilon) {
    return is_point_in(bounding_box, line_segment->start);
  }

  // 1. 将线段转换到矩形的局部坐标系 (中心在原点，轴对齐)
  const double dx1 = line_segment->start.x - bounding_box->center.x;
  const double dy1 = line_segment->start.y - bounding_box->center.y;
  const double lx1 = dx1 * bounding_box->cos_heading + dy1 * bounding_box->sin_heading;
  const double ly1 = -dx1 * bounding_box->sin_heading + dy1 * bounding_box->cos_heading;

  const double dx2 = line_segment->end.x - bounding_box->center.x;
  const double dy2 = line_segment->end.y - bounding_box->center.y;
  const double lx2 = dx2 * bounding_box->cos_heading + dy2 * bounding_box->sin_heading;
  const double ly2 = -dx2 * bounding_box->sin_heading + dy2 * bounding_box->cos_heading;

  // 2. 使用分离轴定理 (SAT)
  // 轴 1 & 2: 矩形的局部坐标轴 (X 和 Y)
  if (std::min(lx1, lx2) > bounding_box->half_length + kMathEpsilon ||
      std::max(lx1, lx2) < -bounding_box->half_length - kMathEpsilon ||
      std::min(ly1, ly2) > bounding_box->half_width + kMathEpsilon ||
      std::max(ly1, ly2) < -bounding_box->half_width - kMathEpsilon) {
    return false;
  }

  // 轴 3: 线段的法线方向 (在局部坐标系下)
  const double v_lx = lx2 - lx1;
  const double v_ly = ly2 - ly1;
  // 局部坐标系下的法线 n = (-v_ly, v_lx)
  // 矩形在法线上的投影半径为: |n_x| * half_length + |n_y| * half_width
  const double max_box_proj = std::abs(v_ly) * bounding_box->half_length + 
                               std::abs(v_lx) * bounding_box->half_width;
  // 线段在法线上的投影 (两端点投影相同): n_x * lx1 + n_y * ly1
  const double line_proj = -v_ly * lx1 + v_lx * ly1;

  return std::abs(line_proj) <= max_box_proj + kMathEpsilon;
}

bool CollisionDetection::has_overlap(const std::shared_ptr<Polygon2d>& polygon, const std::shared_ptr<LineSegment2d> &line_segment) {
    if (!polygon || polygon->points().empty()) return false;

    // 1. 优先进行类型分发 (Type Dispatch) - 高效路径
    if (auto box_ptr = std::dynamic_pointer_cast<Box2d>(polygon)) {
        return has_overlap(box_ptr, line_segment);
    }

    if (auto triangle_ptr = std::dynamic_pointer_cast<Triangle2d>(polygon)) {
        return has_overlap(triangle_ptr, line_segment);
    }

    // 2. 通用多边形处理
    // 先进行 AABB 粗检
    if (!has_overlap_aabb(polygon, line_segment)) {
        return false;
    }

    // 3. 精细检查
    return has_overlap_polygon_line_segment(polygon, line_segment);
}

bool CollisionDetection::has_overlap(const std::shared_ptr<Triangle2d> &triangle, const std::shared_ptr<LineSegment2d> &line_segment) {
    // 1. 检查线段端点是否在三角形内
    if (is_point_in(triangle, line_segment->start) || is_point_in(triangle, line_segment->end)) {
        return true;
    }

    // 2. 检查线段是否与三角形的三条边相交
    const auto& points = triangle->points();
    for (size_t i = 0; i < points.size(); ++i) {
        auto edge = std::make_shared<LineSegment2d>(points[i], points[(i + 1) % points.size()]);
        if (segments_intersect(edge, line_segment)) {
            return true;
        }
    }

    return false;
}

bool CollisionDetection::has_overlap_aabb(const std::shared_ptr<Polygon2d> &polygon, const std::shared_ptr<LineSegment2d> &line_segment) {
    const auto& aabb = polygon->aabb();
    double seg_min_x = std::min(line_segment->start.x, line_segment->end.x);
    double seg_max_x = std::max(line_segment->start.x, line_segment->end.x);
    double seg_min_y = std::min(line_segment->start.y, line_segment->end.y);
    double seg_max_y = std::max(line_segment->start.y, line_segment->end.y);

    return !(seg_max_x < aabb.min_x - kMathEpsilon || seg_min_x > aabb.max_x + kMathEpsilon ||
             seg_max_y < aabb.min_y - kMathEpsilon || seg_min_y > aabb.max_y + kMathEpsilon);
}

bool CollisionDetection::has_overlap_polygon_line_segment(const std::shared_ptr<Polygon2d> &polygon, const std::shared_ptr<LineSegment2d> &line_segment) {
    // 1. 检查线段端点是否在多边形内
    if (is_point_in(polygon, line_segment->start) || 
        is_point_in(polygon, line_segment->end)) {
        return true;
    }

    // 2. 检查线段是否与多边形的任何一条边相交
    const auto& points = polygon->points();
    for (size_t i = 0; i < points.size(); ++i) {
        auto edge = std::make_shared<LineSegment2d>(points[i], points[(i + 1) % points.size()]);
        if (segments_intersect(edge, line_segment)) {
            return true;
        }
    }

    return false;
}

bool CollisionDetection::segments_intersect(const std::shared_ptr<LineSegment2d> &s1, const std::shared_ptr<LineSegment2d> &s2) {
    // 快速排斥实验
    if (std::max(s1->start.x, s1->end.x) < std::min(s2->start.x, s2->end.x) - kMathEpsilon ||
        std::max(s2->start.x, s2->end.x) < std::min(s1->start.x, s1->end.x) - kMathEpsilon ||
        std::max(s1->start.y, s1->end.y) < std::min(s2->start.y, s2->end.y) - kMathEpsilon ||
        std::max(s2->start.y, s2->end.y) < std::min(s1->start.y, s1->end.y) - kMathEpsilon) {
        return false;
    }

    // 跨立实验
    // 使用 cross_prod(A, B, C) 计算向量 AB 和 AC 的叉积
    double cp1 = cross_prod(s1->start, s1->end, s2->start);
    double cp2 = cross_prod(s1->start, s1->end, s2->end);
    double cp3 = cross_prod(s2->start, s2->end, s1->start);
    double cp4 = cross_prod(s2->start, s2->end, s1->end);

    return (((cp1 > kMathEpsilon && cp2 < -kMathEpsilon) || (cp1 < -kMathEpsilon && cp2 > kMathEpsilon)) &&
            ((cp3 > kMathEpsilon && cp4 < -kMathEpsilon) || (cp3 < -kMathEpsilon && cp4 > kMathEpsilon)));
}

std::shared_ptr<Polygon2d> CollisionDetection::get_bounding_box(const std::shared_ptr<VehicleState> &vehicle_state, double length, double width, double back_to_center) {
  double shift_distance = length / 2.0 - back_to_center;
  double cos_heading = std::cos(vehicle_state->heading);
  double sin_heading = std::sin(vehicle_state->heading);
  Vec2d center{vehicle_state->x + shift_distance * cos_heading, vehicle_state->y + shift_distance * sin_heading};
  return std::make_shared<Box2d>(center, length, width, vehicle_state->heading);
}

std::shared_ptr<Polygon2d> CollisionDetection::get_bounding_box(const std::shared_ptr<Obstacle> &obstacle) {
    return get_bounding_box(Vec2d{obstacle->x, obstacle->y}, obstacle->length, obstacle->width, obstacle->heading);
}

std::shared_ptr<Polygon2d> CollisionDetection::get_bounding_box(const Vec2d &center, double length, double width, double heading) {
    return std::make_shared<Box2d>(center, length, width, heading);
}

std::shared_ptr<Polygon2d> CollisionDetection::get_bounding_box(const std::shared_ptr<VehicleParams> &vehicle_params, double x, double y, double heading) {
    double length = vehicle_params->lf + vehicle_params->lr;
    double width = vehicle_params->width;
    double shift_distance = length / 2.0 - vehicle_params->lr;
    double cos_heading = std::cos(heading);
    double sin_heading = std::sin(heading);
    Vec2d center{x + shift_distance * cos_heading, y + shift_distance * sin_heading};
    return get_bounding_box(center, length, width, heading);
}

double CollisionDetection::distance_to(const std::shared_ptr<Box2d> &box1, const std::shared_ptr<Box2d> &box2) {
    if (has_overlap(box1, box2)) {
        return 0.0;
    }

    const auto& points2 = box2->points();
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points2.size(); ++i) {
        auto edge = std::make_shared<LineSegment2d>(points2[i], points2[(i + 1) % points2.size()]);
        min_dist = std::min(min_dist, distance_to(box1, edge));
    }
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Triangle2d> &t1, const std::shared_ptr<Triangle2d> &t2) {
    if (has_overlap(t1, t2)) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::max();
    const auto& p1 = t1->points();
    for (size_t i = 0; i < 3; ++i) {
        auto edge = std::make_shared<LineSegment2d>(p1[i], p1[(i + 1) % 3]);
        min_dist = std::min(min_dist, distance_to(t2, edge));
    }
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Box2d> &box, const std::shared_ptr<Triangle2d> &triangle) {
    if (has_overlap(box, triangle)) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::max();
    const auto& p = triangle->points();
    for (size_t i = 0; i < 3; ++i) {
        auto edge = std::make_shared<LineSegment2d>(p[i], p[(i + 1) % 3]);
        min_dist = std::min(min_dist, distance_to(box, edge));
    }
    return min_dist;
}

double CollisionDetection::distance_to(const std::shared_ptr<Triangle2d> &triangle, const Vec2d &point) {
    if (is_point_in(triangle, point)) {
        return 0.0;
    }
    const auto& p = triangle->points();
    double d0 = p_t_l_distance(point.x, point.y, p[0].x, p[0].y, p[1].x, p[1].y);
    double d1 = p_t_l_distance(point.x, point.y, p[1].x, p[1].y, p[2].x, p[2].y);
    double d2 = p_t_l_distance(point.x, point.y, p[2].x, p[2].y, p[0].x, p[0].y);
    return std::min({d0, d1, d2});
}

double CollisionDetection::distance_to(const std::shared_ptr<Triangle2d> &triangle, const std::shared_ptr<LineSegment2d> &line_segment) {
    if (has_overlap(triangle, line_segment)) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::max();
    min_dist = std::min(min_dist, distance_to(triangle, line_segment->start));
    min_dist = std::min(min_dist, distance_to(triangle, line_segment->end));
    
    const auto& p = triangle->points();
    for (const auto& vertex : p) {
        min_dist = std::min(min_dist, AD_algorithm::general::distance_to(line_segment, vertex));
    }
    return min_dist;
}

bool CollisionDetection::is_point_in(const std::shared_ptr<Triangle2d> &triangle, const Vec2d &point) {
    // Use Barycentric Coordinates (Fastest & Stable)
    const auto& p = triangle->points();
    const auto& A = p[0];
    const auto& B = p[1];
    const auto& C = p[2];
    
    double det = (B.y - C.y) * (A.x - C.x) + (C.x - B.x) * (A.y - C.y);
    double lambda1 = ((B.y - C.y) * (point.x - C.x) + (C.x - B.x) * (point.y - C.y)) / det;
    double lambda2 = ((C.y - A.y) * (point.x - C.x) + (A.x - C.x) * (point.y - C.y)) / det;
    double lambda3 = 1.0 - lambda1 - lambda2;
    
    return lambda1 >= -kMathEpsilon && lambda2 >= -kMathEpsilon && lambda3 >= -kMathEpsilon;
}

bool CollisionDetection::is_point_in(const std::shared_ptr<Polygon2d>& bounding_box, const Vec2d &point) {
    if (!bounding_box || bounding_box->points().empty()) return false;

    // 1. 优先进行类型分发 (Type Dispatch) - O(1) 路径
    if (auto box_ptr = std::dynamic_pointer_cast<Box2d>(bounding_box)) {
        return is_point_in(box_ptr, point);
    }

    if (auto triangle_ptr = std::dynamic_pointer_cast<Triangle2d>(bounding_box)) {
        return is_point_in(triangle_ptr, point);
    }

    // 2. 通用多边形处理 - O(N) 路径
    // 先进行 AABB 粗检 (Coarse Check) 以快速剔除远距离点
    if (!is_point_in_aabb(bounding_box, point)) {
        return false;
    }
    
    // 3. 叉乘法 (Cross Product Check) - 适用于凸多边形
    if (bounding_box->is_convex() && is_point_in_convex_polygon(bounding_box, point)) {
        return true;
    }
    // 4. 射线法 (Ray Casting) - 最后的保底方案，支持凹多边形
    return is_point_in_polygon_ray_casting(bounding_box, point);
}

bool CollisionDetection::is_point_in_aabb(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point) {
    const auto& aabb = polygon->aabb();
    return !(point.x < aabb.min_x - kMathEpsilon || point.x > aabb.max_x + kMathEpsilon || 
             point.y < aabb.min_y - kMathEpsilon || point.y > aabb.max_y + kMathEpsilon);
}

bool CollisionDetection::is_point_in_convex_polygon(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point) {
    bool all_positive = true;
    bool all_negative = true;
    const auto& points = polygon->points();
    for (size_t i = 0; i < points.size(); ++i) {
        double cp = cross_prod(points[i], points[(i + 1) % points.size()], point);
        if (cp < -kMathEpsilon) all_positive = false;
        if (cp > kMathEpsilon) all_negative = false;
        if (!all_positive && !all_negative) return false;
    }
    return all_positive || all_negative;
}

bool CollisionDetection::is_point_in_polygon_ray_casting(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point) {
    bool inside = false;
    const auto& points = polygon->points();
    for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
        const auto& p1 = points[i];
        const auto& p2 = points[j];
        
        if (((p1.y > point.y) != (p2.y > point.y)) &&
            (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool CollisionDetection::AABB_detect(const std::shared_ptr<Polygon2d>& boxA, const std::shared_ptr<Polygon2d>& boxB) {
    if (!boxA || !boxB || boxA->points().empty() || boxB->points().empty()) {
        return false;
    }
    const auto& aabbA = boxA->aabb();
    const auto& aabbB = boxB->aabb();

    return (aabbA.min_x <= aabbB.max_x && aabbA.max_x >= aabbB.min_x &&
            aabbA.min_y <= aabbB.max_y && aabbA.max_y >= aabbB.min_y);
}

bool CollisionDetection::SAT_detect(const std::shared_ptr<Polygon2d>& polyA, const std::shared_ptr<Polygon2d>& polyB) {
    if (!polyA || !polyB || polyA->points().empty() || polyB->points().empty()) {
        return false;
    }

    auto check_separation = [](const std::shared_ptr<Polygon2d>& p1, const std::shared_ptr<Polygon2d>& p2) {
        const auto& points1 = p1->points();
        const auto& points2 = p2->points();
        for (size_t i = 0; i < points1.size(); ++i) {
            Vec2d edge = points1[(i + 1) % points1.size()] - points1[i];
            Vec2d axis{-edge.y, edge.x}; // Normal
            axis.Normalize();

            double min1 = std::numeric_limits<double>::max();
            double max1 = std::numeric_limits<double>::lowest();
            for (const auto& p : points1) {
                 double proj = inner_prod(p, axis);
                 min1 = std::min(min1, proj);
                 max1 = std::max(max1, proj);
            }

            double min2 = std::numeric_limits<double>::max();
            double max2 = std::numeric_limits<double>::lowest();
            for (const auto& p : points2) {
                 double proj = inner_prod(p, axis);
                 min2 = std::min(min2, proj);
                 max2 = std::max(max2, proj);
            }

            if (max1 < min2 - kMathEpsilon || max2 < min1 - kMathEpsilon) {
                return true; // Separated
            }
        }
        return false;
    };

    if (check_separation(polyA, polyB)) return false;
    if (check_separation(polyB, polyA)) return false;

    return true;
}

bool CollisionDetection::GJK_detect(const std::shared_ptr<Polygon2d>& boxA, const std::shared_ptr<Polygon2d>& boxB) {
    if (!boxA || !boxB || boxA->points().empty() || boxB->points().empty()) return false;

    auto dot = [](const Vec2d& a, const Vec2d& b) -> double { return a.x * b.x + a.y * b.y; };
    
    auto get_support = [&](const std::shared_ptr<Polygon2d>& poly, const Vec2d& dir) -> Vec2d {
        double max_d = -std::numeric_limits<double>::infinity();
        const auto& points = poly->points();
        Vec2d best = points[0];
        for (const auto& p : points) {
            double d = dot(p, dir);
            if (d > max_d) {
                max_d = d;
                best = p;
            }
        }
        return best;
    };

    auto minkowski_support = [&](const Vec2d& dir) -> Vec2d {
        return get_support(boxA, dir) - get_support(boxB, Vec2d{-dir.x, -dir.y});
    };

    Vec2d simplex[3];
    int simplex_size = 0;

    Vec2d direction = {1.0, 0.0}; 
    simplex[0] = minkowski_support(direction);
    direction = Vec2d{-simplex[0].x, -simplex[0].y};
    simplex_size = 1;

    int iter = 0;
    while (iter++ < 15) {
        if (dot(direction, direction) < kMathEpsilon) return true; 

        Vec2d A = minkowski_support(direction);
        if (dot(A, direction) < 0) return false; 

        simplex[simplex_size++] = A; 

        if (simplex_size == 2) {
            Vec2d B = simplex[0];
            Vec2d AB = B - A;
            Vec2d AO = Vec2d{-A.x, -A.y};
            Vec2d perp = {-AB.y, AB.x};
            if (dot(perp, AO) < 0) perp = Vec2d{-perp.x, -perp.y};
            direction = perp;
        } else if (simplex_size == 3) {
            Vec2d C = simplex[0];
            Vec2d B = simplex[1];
            Vec2d AB = B - A; 
            Vec2d AC = C - A; 
            Vec2d AO = Vec2d{-A.x, -A.y};    
            
            Vec2d AB_perp = {-AB.y, AB.x};
            if (dot(AB_perp, AC) > 0) AB_perp = Vec2d{-AB_perp.x, -AB_perp.y};
            
            Vec2d AC_perp = {-AC.y, AC.x};
            if (dot(AC_perp, AB) > 0) AC_perp = Vec2d{-AC_perp.x, -AC_perp.y};
            
            if (dot(AB_perp, AO) > 0) {
                simplex[0] = B;
                simplex[1] = A;
                simplex_size = 2;
                direction = AB_perp;
            } else if (dot(AC_perp, AO) > 0) {
                simplex[0] = C;
                simplex[1] = A;
                simplex_size = 2;
                direction = AC_perp;
            } else {
                return true; 
            }
        }
    }
    return false;
}

bool CollisionDetection::has_overlap(const std::shared_ptr<Polygon2d>& boxA, const std::shared_ptr<Polygon2d>& boxB) {
    if (!AABB_detect(boxA, boxB)) {
        return false;
    }

    if(boxA->points().size() + boxB->points().size() > 20){
        return GJK_detect(boxA, boxB); 
    }
    else{
        return SAT_detect(boxA, boxB);
    }
}

}} // namespace AD_algorithm::general