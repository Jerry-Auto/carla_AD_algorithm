#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include <memory>
#include "general_modules/common_types.h"
#include "general_modules/geometry.h"

namespace AD_algorithm {
namespace general {

constexpr double kMathEpsilon = 1e-10;

// 限制函数
template<typename T>
T clamp(const T value, T bound1, T bound2)
{
    if (bound1 > bound2)
    {
        std::swap(bound1, bound2);
    }

    if ( value < bound1)
    {
        return bound1;
    }
    else if(value > bound2)
    {
        return bound2;
    }

    return value;
}

// 角度标准化
double normalize_angle(const double angle);

double sign(double x);

double p_t_l_distance(double query_x, double query_y, double start_x, double start_y, double end_x, double end_y);

double cross_prod(const Vec2d &start_point, const Vec2d &end_point_1, const Vec2d &end_point_2);

double inner_prod(const Vec2d &v1, const Vec2d &v2);

double distance_to(const std::shared_ptr<LineSegment2d> &line_segment2d, const Vec2d &point);

Vec2d rotate_point(const Vec2d& point, double rotation);

std::vector<LineSegment2d> compute_rectangle_edges(
    double center_x, double center_y,  // 矩形中心坐标
    double length, double width,       // 矩形的长和宽
    double rotation                    // 旋转角度（弧度）
);

}}