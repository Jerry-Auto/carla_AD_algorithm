#pragma once

#include <vector>
#include <stdexcept>
#include "ReferenceLine.h"
#include "general_modules/common_types.h"
#include <eigen3/Eigen/Dense>
#include <algorithm>

namespace AD_algorithm {
namespace general {

class FrenetFrame {
public:

    explicit FrenetFrame(const std::vector<std::pair<double, double>>& xy_points)
        : ref_line_(xy_points) {
        if (!ref_line_.flag_ref()) {
            throw std::runtime_error("Invalid ReferenceLine from xy points.");
        }
    }

    explicit FrenetFrame(const std::vector<TrajectoryPoint>& traj_points,bool is_smooth=true)
        : ref_line_([&traj_points, is_smooth]() {
            // 如果轨迹来自于控制，那么就不用再平滑处理了
            std::vector<std::pair<double, double>> xy;
            for (const auto& tp : traj_points)
                xy.push_back({tp.x, tp.y});
            return ReferenceLine(xy, is_smooth);
        }()),
        _trajectory_points(traj_points)
    {
        if (!ref_line_.flag_ref()) {
            throw std::runtime_error("Failed to build valid ReferenceLine from trajectory points.");
        }
    }
    
    explicit FrenetFrame(const ReferenceLine& reference_line)
        : ref_line_(reference_line) {
        if (!ref_line_.flag_ref()) {
            throw std::invalid_argument("Input ReferenceLine is invalid.");
        }
    }

    // 坐标转换
    FrenetPoint cartesian_to_frenet(double x, double y) const;// 只能获得s,l
    FrenetPoint cartesian_to_frenet(const TrajectoryPoint& cart_point) const;// 能够获得完整的frenet信息
    FrenetPoint cartesian_to_frenet(const VehicleState& cart_point) const;// 能够获得完整的frenet信息
    std::vector<FrenetPoint> cartesian_to_frenet(const std::vector<TrajectoryPoint>& cart_points) const;// 批量转换
    
    TrajectoryPoint frenet_to_cartesian(const FrenetPoint& frenet_point) const; // 单点转换,完备信息
    std::vector<TrajectoryPoint> frenet_to_cartesian(const std::vector<FrenetPoint>& frenet_points) const; // 批量转换

    // 障碍物投影
    std::vector<FrenetPoint> project_obstacle_to_frenet(const Obstacle& obstacle) const;// 仅投影位置s,l
    std::vector<FrenetPoint> project_dynamic_obstacle_to_frenet(const Obstacle& obstacle) const;// 投影位置并赋予速度信息s,l,ṡ,l̇

    // 将一组 Frenet 多边形（每个 obstacle 的角点）转换为 SLObstacle 列表
    // static 便于在不持有 FrenetFrame 实例时调用
    static std::vector<SLObstacle> convertToSLObstacles(
        const std::vector<std::vector<FrenetPoint>>& frenet_obstacles,
        double safety_margin = 0.5);

    // 将 ST 图节点（FrenetPoint 按 t,s）转换为 STObstacle 列表
    static std::vector<STObstacle> convertToSTObstacles(
        const std::vector<std::vector<FrenetPoint>>& st_graph,
        double safety_margin = 0.5);

    // 访问
    const std::vector<PathPoint>& get_reference_path() const { return ref_line_.get_path_points(); }
    const ReferenceLine& get_reference_line() const { return ref_line_; }
    // 获取在frenet坐标下某一点对应的匹配点，精准定位
    PathPoint get_matched_point(double x, double y, double heading)const;
    
    // 用于纵向控制，获取匹配的轨迹点，由于轨迹点是没有s的，因此需要返回s，不是精准定位
    // 除此之外，纵向控制涉及s,v,a，应该是用时间来查找匹配点，我希望在特定时间点上车辆应该有什么样的状态
    std::pair<TrajectoryPoint,double> get_matched_trj_point(double time_stamp)const;

    std::pair<TrajectoryPoint,double> get_matched_trj_point(double x, double y, double heading)const;
    
    // 允许通过 [] 访问参考线上的 PathPoint（只读）
    const PathPoint& operator[](size_t index) const {
        return ref_line_[index];
    }
    const TrajectoryPoint& traj_point(size_t index) const {
        return _trajectory_points[index];
    }
    size_t size() const {
        return ref_line_.size();
    }
private:
    struct ProjectionResult {
        double s;
        double l;
        Eigen::Vector2d r;
        double heading;
        double kappa;
        Eigen::Vector2d tau;
        Eigen::Vector2d nor;
    };

    ProjectionResult project_to_path_with_memory(double x, double y, double heading) const;
    
    ReferenceLine ref_line_;
    std::vector<TrajectoryPoint> _trajectory_points;

    mutable size_t _previous_match_index = 0;
    mutable bool _is_first_run = true;
    
    // 新增：高效匹配点查找（带方向预测）
    size_t find_nearest_index_with_memory(double x, double y, double heading) const;
    PathPoint interpolate_by_s(double s) const;


    size_t find_nearest_index(double x, double y) const ;
    ProjectionResult project_to_path(double x, double y) const ;
};
} // namespace general
} // namespace AD_algorithm