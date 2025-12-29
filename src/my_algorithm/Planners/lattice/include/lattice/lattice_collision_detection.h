#ifndef LATTICE_COLLISION_DETECTION_H_
#define LATTICE_COLLISION_DETECTION_H_

#include <cfloat>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <memory>

#include "lattice/lattice_types.h"
#include "general_modules/collision_detection.h"

namespace AD_algorithm {
namespace planner {

class latticeCollisionDetection {
 public:
  latticeCollisionDetection() = delete;
  /**
   * @brief Construct a new Collision Detection object
   * 
   * @param detected_objects 检测到的其他车辆，也就是道路中的障碍物
   * @param collision_distance 碰撞距离
   * @param ref_path 参考路径（Cartesian下）
   */
  latticeCollisionDetection(const std::vector<Obstacle>& detected_objects,
                     const double& collision_distance,
                     const std::vector<PathPoint>& ref_path);
  
  /**
   * @brief 更新障碍物和参考路径
   * 
   * @param detected_objects 检测到的其他车辆
   * @param ref_path 参考路径
   */
  void Update(const std::vector<Obstacle>& detected_objects,
              const std::vector<PathPoint>& ref_path);

  /**
   * @brief 对障碍物进行分类，分为静态障碍物和动态障碍物
   * 
   * @param detected_objects 检测到的其他车辆，也就是道路中的障碍物
   */
  void ObstacleClassification(std::vector<Obstacle>& detected_objects);
  /**
   * @brief 计算障碍box
   * 
   * @param object 障碍物
   */
  void CalCollisionBox(Obstacle& object);
  /**
   * @brief 判断是否碰撞
   * 
   * @param path Frenet坐标系下的轨迹
   * @param leader_frenet_point 前面车的坐标点
   * @param is_car_followed 是否正在跟车的flag
   * @return true 会发生碰撞
   * @return false 不会发生碰撞
   */
  bool IsCollision(latticeFrenetPath& path, const latticeFrenetPoint& leader_frenet_point,
                   const bool& is_car_followed);

 public:
  double collision_dis_;                          // 碰撞距离
  std::vector<Obstacle> detected_objects_;        // 检测到的所有障碍物
  std::vector<Obstacle> static_obstacle_list_;    // 静态障碍物
  std::vector<Obstacle> dynamic_obstacle_list_;   // 动态障碍物
  std::vector<PathPoint> ref_path_;               // 参考轨迹（Cartesian下）
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_COLLISION_DETECTION_H_
