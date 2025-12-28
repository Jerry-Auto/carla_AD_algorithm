#pragma once

#ifndef PLANNER_BASE_H
#define PLANNER_BASE_H

#include <vector>
#include <memory>
#include <string>
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace planner {

class PlannerBase {
public:
    PlannerBase(const std::string& name = "Planner") {
        logger_ = std::make_shared<general::Logger>(name);
    }
    virtual ~PlannerBase() = default;

    /**
     * @brief 设置日志开关
     */
    virtual void set_log_enable(bool enable) {
        if (logger_) logger_->set_enable(enable);
    }

protected:
    /**
     * @brief 辅助函数，方便子类调用
     */
    template<typename... Args>
    void log(Args&&... args) const {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

    std::shared_ptr<general::Logger> logger_;

public:
    /**
     * @brief 主规划接口
     * @param ego_state 车辆状态
     * @param obstacles 障碍物列表
     * @param reference_speed 参考速度
     * @param current_time 当前时间
     * @return 规划出的轨迹点列表
     */
    virtual std::vector<general::TrajectoryPoint> plan(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const std::vector<general::Obstacle>& obstacles,
        double reference_speed,
        double current_time) = 0;

    /**
     * @brief 设置全局参考线
     * @param reference_line 全局路径点
     * @return 是否设置成功
     */
    virtual bool setGlobalReferenceLine(const std::vector<general::PathPoint>& reference_line) = 0;

    /**
     * @brief 检查轨迹有效性
     * @param trajectory 待检查轨迹
     * @param ego_state 车辆状态
     * @param reason 失败原因输出
     * @param min_points 最小点数要求
     * @return 是否有效
     */
    virtual bool isTrajectoryValid(
        const std::vector<general::TrajectoryPoint>& trajectory,
        std::string* reason = nullptr,
        size_t min_points = 5) = 0;
};

} // namespace planner
} // namespace AD_algorithm

#endif // PLANNER_BASE_H
