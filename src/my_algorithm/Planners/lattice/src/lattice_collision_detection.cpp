#include "lattice/lattice_collision_detection.h"
#include "general_modules/FrenetFrame.h"

namespace AD_algorithm {
namespace planner {

latticeCollisionDetection::latticeCollisionDetection() : collision_dis_(1.0) {
    // 默认构造不填充障碍物，使用 Update(...) 在每个规划周期填入最新数据
    detected_objects_.clear();
    static_obstacle_list_.clear();
    dynamic_obstacle_list_.clear();
    ref_path_.clear();
}

latticeCollisionDetection::latticeCollisionDetection(const std::vector<Obstacle>& detected_objects,
                     const double& collision_distance,
                     const std::vector<PathPoint>& ref_path) : latticeCollisionDetection() {
    // 委托给默认构造，然后使用 Update 保持逻辑一致（建议周期性调用 Update）
    collision_dis_ = collision_distance;
    Update(detected_objects, ref_path);
}

void latticeCollisionDetection::Update(const std::vector<Obstacle>& detected_objects,
              const std::vector<PathPoint>& ref_path) {
    detected_objects_ = detected_objects;
    ref_path_ = ref_path;
    static_obstacle_list_.clear();
    dynamic_obstacle_list_.clear();
    ObstacleClassification(detected_objects_);

    // rebuild predicted polygons for this planning cycle
    predicted_polygons_.clear();
    BuildPredictedEnvironment();
}

void latticeCollisionDetection::CalCollisionBox(Obstacle& object) {
  // 使用 general::CollisionDetection 工具构建障碍包围多边形
  // 我们不在此处缓存具体点阵，而是在 BuildPredictedEnvironment 中为每个时刻构建预测包围盒
  (void)object;
}

bool latticeCollisionDetection::IsCollision(latticeFrenetPath& path, 
                                     const latticeFrenetPoint& leader_point,
                                     const bool& is_car_followed) {
  // 判定：对每个时刻（由 frenet_point.t 给出），构建 ego 的包围多边形并与预测障碍在同一时刻的多边形集合做 overlap 检测
  for (int i = 0; i < path.size_; ++i) {
    const auto& fp = path.frenet_points[i];
    double t = fp.t;
    // ego 在 Cartesian 下的位置已在 frenet_point 中填充（调用者需要保证转换完成）
    double x = fp.x;
    double y = fp.y;
    double heading = fp.yaw;

    // 如果与 leader 的相对关系且正在跟车，则对该 leader 采取特殊放宽策略（与原实现一致）
    bool near_leader = false;
    if (is_car_followed) {
      double d_leader = std::hypot(leader_point.x - x, leader_point.y - y);
      near_leader = d_leader < 2.0;
    }

    // 先计算最小距离并将小范围内的点加到 cost
    double min_dist = MinDistanceToObstaclesAtTime(x, y, t);
    if (min_dist < 3.5 && !(is_car_followed && near_leader)) {
      path.cost += min_dist / 3.0;
    }

    // 重叠判定（严格碰撞）：构造 ego 多边形并用 has_overlap 判定
    if (HasOverlapWithPredicted(x, y, heading, t)) {
      return true;
    }
  }
  return false; 
}

bool latticeCollisionDetection::InCollision(const std::vector<AD_algorithm::general::TrajectoryPoint>& trajectory) const {
  if (trajectory.empty()) return false;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto& tp = trajectory[i];
    double t = tp.time_stamped;
    if (t <= 0.0) t = i * time_resolution_; // fallback
    auto ego_box = AD_algorithm::general::CollisionDetection::get_bounding_box(
        AD_algorithm::general::Vec2d{tp.x, tp.y}, ego_length_, ego_width_, tp.heading);
    if (!ego_box) continue;
    size_t idx = static_cast<size_t>(std::round(t / time_resolution_));
    if (idx >= predicted_polygons_.size()) idx = predicted_polygons_.size() - 1;
    for (const auto& poly : predicted_polygons_[idx]) {
      if (AD_algorithm::general::CollisionDetection::has_overlap(ego_box, poly)) return true;
      // 如果没有重叠，也可以基于最小距离判定
      double dist = AD_algorithm::general::CollisionDetection::distance_to(poly, AD_algorithm::general::Vec2d{tp.x,tp.y});
      if (dist <= collision_dis_) return true;
    }
  }
  return false;
}

void latticeCollisionDetection::ObstacleClassification(
    std::vector<Obstacle>& detected_objects) {
  for (auto& obstacle : detected_objects) {
    if (obstacle.getSpeed() > 0.2) {
      dynamic_obstacle_list_.push_back(obstacle);
    }
    else {
      static_obstacle_list_.push_back(obstacle);
    }
  }
}

void latticeCollisionDetection::BuildPredictedEnvironment() {
  // 构建每个采样时刻的障碍多边形集合（使用简单的匀速模型预测）
  if (time_resolution_ <= 0.0 || time_horizon_ <= 0.0) return;
  size_t steps = static_cast<size_t>(std::ceil(time_horizon_ / time_resolution_));
  predicted_polygons_.resize(steps + 1);

  for (size_t i = 0; i <= steps; ++i) {
    double t = i * time_resolution_;
    std::vector<std::shared_ptr<AD_algorithm::general::Polygon2d>> polys;
    polys.reserve(detected_objects_.size());
    for (const auto& obs : detected_objects_) {
      // 预测位置：简单匀速线性预测
      Obstacle pred = obs;
      pred.x = obs.x + obs.vx * t;
      pred.y = obs.y + obs.vy * t;
      // 使用通用接口构造多边形包围盒
      auto poly = AD_algorithm::general::CollisionDetection::get_bounding_box(
          std::make_shared<Obstacle>(pred));
      // 扩展包围盒以包含安全缓冲（以 collision_dis_ 为基础）
      if (poly) {
        // 目前没有直接的缩放接口，采用扩展车辆尺寸方法：重新生成更大的 box
        double ext_length = pred.length + 2.0 * collision_dis_;
        double ext_width = pred.width + 2.0 * collision_dis_;
        auto ext_poly = AD_algorithm::general::CollisionDetection::get_bounding_box(
            AD_algorithm::general::Vec2d{pred.x, pred.y}, ext_length, ext_width, pred.heading);
        if (ext_poly) polys.push_back(ext_poly);
      }
    }
    predicted_polygons_[i] = std::move(polys);
  }
}

// 返回点到时刻 t 的最小距离
double latticeCollisionDetection::MinDistanceToObstaclesAtTime(double x, double y, double t) const {
  if (predicted_polygons_.empty()) return std::numeric_limits<double>::infinity();
  // use floor-like indexing and clamp to available range to avoid boundary jitter
  size_t idx = 0;
  if (t <= 0.0) {
    idx = 0;
  } else {
    idx = static_cast<size_t>(t / time_resolution_);
  }
  if (idx >= predicted_polygons_.size()) idx = predicted_polygons_.size() - 1;
  AD_algorithm::general::Vec2d pt(x, y);
  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& poly : predicted_polygons_[idx]) {
    double dist = AD_algorithm::general::CollisionDetection::distance_to(poly, pt);
    if (dist < min_dist) min_dist = dist;
  }
  return min_dist;
}

bool latticeCollisionDetection::HasOverlapWithPredicted(double x, double y, double heading, double t) const {
  if (predicted_polygons_.empty()) return false;
  // use floor-like indexing and clamp to available range to avoid boundary jitter
  size_t idx = 0;
  if (t <= 0.0) {
    idx = 0;
  } else {
    idx = static_cast<size_t>(t / time_resolution_);
  }
  if (idx >= predicted_polygons_.size()) idx = predicted_polygons_.size() - 1;
  // 构造ego多边形（使用ego_length/width/back_to_center），并修正reference-point与中心点的不一致
  double shift_distance = ego_length_ / 2.0 - ego_back_to_center_;
  AD_algorithm::general::Vec2d shifted_center(x + shift_distance * std::cos(heading),
                                             y + shift_distance * std::sin(heading));
  auto ego_box = AD_algorithm::general::CollisionDetection::get_bounding_box(
      shifted_center, ego_length_, ego_width_, heading);
  if (!ego_box) return false;
  for (const auto& poly : predicted_polygons_[idx]) {
    if (AD_algorithm::general::CollisionDetection::has_overlap(ego_box, poly)) return true;
  }
  return false;
}

} // namespace planner
} // namespace AD_algorithm
