#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include "general_modules/common_types.h"
#include "general_modules/math_tool.h"
#include "general_modules/geometry.h"

namespace AD_algorithm {
namespace general {

struct Vec2d;
struct LineSegment2d;
struct Polygon2d;
struct Box2d;
struct Triangle2d;
struct VehicleState;
struct Obstacle;
struct VehicleParams;

class CollisionDetection {
public:

    CollisionDetection() = default;
    ~CollisionDetection() = default;

    
    static double distance_to(const std::shared_ptr<Polygon2d> &box1, const std::shared_ptr<Polygon2d> &box2);

    static double distance_to(const std::shared_ptr<Polygon2d> &bounding_box, const Vec2d &point);

    static double distance_to(const std::shared_ptr<Polygon2d> &bounding_box, const std::shared_ptr<LineSegment2d> &line_segment);

    
    static bool is_point_in(const std::shared_ptr<Polygon2d>& bounding_box, const Vec2d &point);

    static bool has_overlap(const std::shared_ptr<Polygon2d>& polygon, const std::shared_ptr<LineSegment2d> &line_segment);

    static bool has_overlap(const std::shared_ptr<Polygon2d>& boxA,const std::shared_ptr<Polygon2d>& boxB);
    
    
    static std::shared_ptr<Polygon2d> get_bounding_box(const std::shared_ptr<VehicleState> &vehicle_state, double length, double width, double back_to_center);
    
    static std::shared_ptr<Polygon2d> get_bounding_box(const std::shared_ptr<VehicleParams> &vehicle_params, double x, double y, double heading);
    
    static std::shared_ptr<Polygon2d> get_bounding_box(const Vec2d &center, double length, double width, double heading);
    
    static std::shared_ptr<Polygon2d> get_bounding_box(const std::shared_ptr<Obstacle> &obstacle);

private:

    static double distance_to(const std::shared_ptr<Box2d> &box1, const std::shared_ptr<Box2d> &box2);

    static double distance_to(const std::shared_ptr<Triangle2d> &t1, const std::shared_ptr<Triangle2d> &t2);

    static double distance_to(const std::shared_ptr<Box2d> &box, const std::shared_ptr<Triangle2d> &triangle);

    static double distance_to(const std::shared_ptr<Box2d> &bounding_box, const Vec2d &point);

    static double distance_to(const std::shared_ptr<Box2d> &bounding_box, const std::shared_ptr<LineSegment2d> &line_segment);

    static double distance_to(const std::shared_ptr<Triangle2d> &triangle, const Vec2d &point);

    static double distance_to(const std::shared_ptr<Triangle2d> &triangle, const std::shared_ptr<LineSegment2d> &line_segment);
    
    static bool is_point_in(const std::shared_ptr<Box2d> &bounding_box, const Vec2d &point);

    static bool is_point_in(const std::shared_ptr<Triangle2d> &bounding_box, const Vec2d &point);

    static bool is_point_in_aabb(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point);

    static bool is_point_in_convex_polygon(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point);

    static bool is_point_in_polygon_ray_casting(const std::shared_ptr<Polygon2d> &polygon, const Vec2d &point);
    
    static bool has_overlap(const std::shared_ptr<Box2d> &box, const std::shared_ptr<LineSegment2d> &line_segment);

    static bool has_overlap(const std::shared_ptr<Triangle2d> &triangle, const std::shared_ptr<LineSegment2d> &line_segment);

    static bool has_overlap_aabb(const std::shared_ptr<Polygon2d> &polygon, const std::shared_ptr<LineSegment2d> &line_segment);

    static bool has_overlap_polygon_line_segment(const std::shared_ptr<Polygon2d> &polygon, const std::shared_ptr<LineSegment2d> &line_segment);

    static bool segments_intersect(const std::shared_ptr<LineSegment2d> &s1, const std::shared_ptr<LineSegment2d> &s2);

    static bool AABB_detect(const std::shared_ptr<Polygon2d>& boxA,const std::shared_ptr<Polygon2d>& boxB);

    static bool SAT_detect(const std::shared_ptr<Polygon2d>& boxA,const std::shared_ptr<Polygon2d>& boxB);

    static bool GJK_detect(const std::shared_ptr<Polygon2d>& boxA,const std::shared_ptr<Polygon2d>& boxB);

};

}} // namespace AD_algorithm::general