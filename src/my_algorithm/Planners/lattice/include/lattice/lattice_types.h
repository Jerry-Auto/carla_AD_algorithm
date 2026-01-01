#ifndef LATTICE_TYPES_H_
#define LATTICE_TYPES_H_

#include <vector>
#include <cmath>
#include <limits>
#include "general_modules/common_types.h"

namespace AD_algorithm {
namespace planner {

using PathPoint = AD_algorithm::general::PathPoint;
using Obstacle = AD_algorithm::general::Obstacle;

struct latticeFrenetPoint : public AD_algorithm::general::FrenetPoint {
 public:
  // 继承自 general::FrenetPoint：s, s_dot, s_dot_dot, l, l_dot, l_dot_dot, l_prime, l_prime_prime
  // 说明：s_dot 为纵向速度（s 的一阶时间导数），l_prime 表示 dl/ds（横向对弧长的导数）

  
  double s_d_d_d = 0.0;
  double l_d_d_d = 0.0;
  // 空间导数命名（明确说明）：
  // l_dot_prime = d(l_dot)/ds  （即横向速度关于弧长的导数）
  // l_dot_dot_prime = d(l_dd)/ds （即横向加速度关于弧长的导数）
  double l_dot_prime = 0.0;      // d(dl/dt)/ds
  double l_dot_dot_prime = 0.0;  // d(d^2l/dt^2)/ds

  double v = 0.0; // 笛卡尔速度（近似）
  // 笛卡尔坐标在转换过程中被填充
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double cur = 0.0;

  // 注意：已移除兼容性辅助方法 — 请直接使用底层 FrenetPoint 成员：
  //   - s_dot, s_dot_dot, l_dot, l_dot_dot, l_prime
  // 这样可以保持类型简洁且无歧义。
};

struct latticeFrenetPath {
 public:
  int size_ = 0;
  double cost = 0.0;
  std::vector<latticeFrenetPoint> frenet_points;
  double max_speed = 0.0;
  double max_acc = 0.0;
  double max_curvature = 0.0;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_TYPES_H_
