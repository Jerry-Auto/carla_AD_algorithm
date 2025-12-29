#include "lattice/lattice_collision_detection.h"
#include "general_modules/FrenetFrame.h"

namespace AD_algorithm {
namespace planner {

latticeCollisionDetection::latticeCollisionDetection(const std::vector<Obstacle>& detected_objects,
                     const double& collision_distance,
                     const std::vector<PathPoint>& ref_path) : 
                     collision_dis_(collision_distance),
                     detected_objects_(detected_objects), 
                     ref_path_(ref_path) {
    static_obstacle_list_.clear();
    dynamic_obstacle_list_.clear();
    ObstacleClassification(this->detected_objects_);
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
  // Obstacle in general_modules already has length, width, heading, x, y.
  // We don't need to manually calculate box points if we use general::CollisionDetection::get_bounding_box
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
