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
}

void latticeCollisionDetection::CalCollisionBox(Obstacle& object) {
  // general_modules 中的 Obstacle 已包含 length, width, heading, x, y。
  // 若使用 general::CollisionDetection::get_bounding_box，则无需手动计算包围盒点
  (void)object;
}

bool latticeCollisionDetection::IsCollision(latticeFrenetPath& path, 
                                     const latticeFrenetPoint& leader_point,
                                     const bool& is_car_followed) {
  for (auto& obstacle : detected_objects_) {
    auto obs_box = AD_algorithm::general::CollisionDetection::get_bounding_box(
        std::make_shared<Obstacle>(obstacle));
    
    for (int i = 0; i < path.size_; i++) {
      AD_algorithm::general::Vec2d path_point(path.frenet_points[i].x, path.frenet_points[i].y);
      
      double dist = AD_algorithm::general::CollisionDetection::distance_to(obs_box, path_point);
      
      if (dist < 3.5 && !(is_car_followed && std::hypot(obstacle.x - leader_point.x, obstacle.y - leader_point.y) < 2.0)) {
          path.cost += dist / 3.0;
      }
      if (dist <= collision_dis_) {
          return true;
      }
    }
  }
  return false; 
}

bool latticeCollisionDetection::InCollision(const std::vector<AD_algorithm::general::TrajectoryPoint>& trajectory) const {
  for (const auto& tp : trajectory) {
    AD_algorithm::general::Vec2d pt(tp.x, tp.y);
    for (const auto& obs : detected_objects_) {
      auto box = AD_algorithm::general::CollisionDetection::get_bounding_box(std::make_shared<Obstacle>(obs));
      double dist = AD_algorithm::general::CollisionDetection::distance_to(box, pt);
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

} // namespace planner
} // namespace AD_algorithm
