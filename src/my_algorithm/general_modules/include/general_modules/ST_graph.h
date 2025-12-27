#pragma once
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"

#include <vector>
#include <unordered_map>

namespace AD_algorithm {
namespace general {

class STCoordinate {
public:
    STCoordinate() = default;
    ~STCoordinate() = default;

    // 复用通用类的TrajectoryPoint，替换原自定义结构体
    bool cartesianToST(const std::vector<general::TrajectoryPoint>& cartesianTraj, 
                       std::vector<general::STPoint>& stTraj);

    // 动态障碍物转ST坐标（输入为FrenetPoint，复用通用类）
    bool dynamicObsToST(const std::vector<general::FrenetPoint>& obsFrenetSet, 
                        double deltaL, 
                        std::vector<std::unordered_map<std::string, double>>& stGraph);

    // ST坐标转笛卡尔轨迹（输出为通用类的TrajectoryPoint）
    bool stToCartesian(const std::vector<general::STPoint>& stTraj,
                       const std::vector<general::TrajectoryPoint>& pathTraj,
                       const std::vector<double>& pathIndex2s,
                       std::vector<general::TrajectoryPoint>& cartesianTraj);

    // 在ST轨迹中查找时间对应的索引
    int findTIndex(const std::vector<general::STPoint>& stTraj, double t);

private:
    // 复用ReferenceLine的路径长度计算逻辑，删除手动实现
    std::vector<double> calculatePathIndex2s(const std::vector<general::TrajectoryPoint>& pathTraj);
};

} // namespace planning
} // namespace AD_algorithm
