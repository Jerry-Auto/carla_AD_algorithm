#ifndef LATTICE_COLLISION_DETECTION_H_
#define LATTICE_COLLISION_DETECTION_H_

#include <cfloat>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <memory>

#include "lattice/lattice_types.h"
#include "general_modules/collision_detection.h"
#include "lattice/planner_weights.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/common_types.h"


namespace AD_algorithm {
namespace planner {
        
class latticeCollisionDetection {
private:
    double plan_start_time_ = 0.0; // 本次规划周期的起始时间戳
    std::vector<Obstacle> detected_objects_;        // 检测到的所有障碍物,里面有记录初始时间戳
    std::vector<AD_algorithm::general::SLObstacle> static_obstacle_list_;    // 静态障碍物
    std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame_ = nullptr;
    AD_algorithm::general::VehicleParams vehicle_params_;
    PlannerParams params_;
    std::vector<std::vector<std::shared_ptr<AD_algorithm::general::Obstacle>>> predicted_obstacles_; // 每个时间点上的预测障碍物列表

private:
    // 笛卡尔坐标系下预测在时间点 t 时，所有障碍物的SLObstacle位置，绝对时间
    void PredictObstaclesAtTime(double t);

    // 笛卡尔坐标系下预测所有障碍物的SLObstacle位置
    void PredictObstacles();

    // 在SL空间，frenet坐标系下，只提出静态障碍物，放到成员变量 static_obstacle_list_ 中
    void StaticObstacleClassification();

public:    
    // 默认构造：不会固定障碍物，实例化后应在每个规划周期通过 `Update(...)` 传入最新障碍物与参考路径
    // 参数直接使用结构体默认值
    latticeCollisionDetection();
    
    void setParameters(const PlannerParams& params);

    // 更新本规划周期的障碍物和参考frenet框架
    void Update(const std::vector<Obstacle>& detected_objects,
        const std::shared_ptr<AD_algorithm::general::FrenetFrame>& ref_path,double start_time);

    // 自车在当前轨迹下，在某一时间点 t 与当前时间点所有障碍物的最小距离
    // 其实就是把所有障碍物统一当做在某一时间点上的静态障碍物来处理
    // 笛卡尔坐标系下
    double DistanceToObstaclesAtTime(const general::TrajectoryPoint& traj, double t) const;

    // 直接在SL空间下计算横向路径是否与静态障碍物碰撞，不考虑动态障碍物，便于前期直接过滤一部分横向路径，纵向过滤只考虑数值约束
    bool HasOverlapWithStaticObstacles(const LatCandidate& latcandidate,double offset_s) const;
};

} // namespace planner
} // namespace AD_algorithm

#endif 
