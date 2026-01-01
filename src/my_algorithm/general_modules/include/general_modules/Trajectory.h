#pragma once
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/logger.h"


namespace AD_algorithm {
namespace general {
    class TrajectoryManager {
    public:
        TrajectoryManager(std::shared_ptr<Logger> logger = nullptr);
        ~TrajectoryManager() = default;

        void set_logger(std::shared_ptr<Logger> logger) { logger_ = logger; }
        void set_log_enable(bool enable) { if (logger_) logger_->set_enable(enable); }

        // 设置当前时间
        void setCurrentTime(double current_time) { current_time_ = current_time; }

        // 计算规划起点
        TrajectoryPoint calculatePlanningStartPoint(
            const std::shared_ptr<VehicleState>& ego_state,
            double delta_T = 0.1);

        // 障碍物过滤与分类
        void classifyObstacles(
            const std::shared_ptr<VehicleState>& ego_state,
            const std::vector<Obstacle>& obstacles,
            std::vector<Obstacle>& static_obstacles,
            std::vector<Obstacle>& dynamic_obstacles);

        // 轨迹拼接
        std::vector<TrajectoryPoint> stitchTrajectory(
            const std::vector<TrajectoryPoint>& current_trajectory,
            int stitch_points = 20);

        // 更新最近一次使用的自车状态
        void updateEgoState(const std::shared_ptr<VehicleState>& ego_state);

        // 横向路径结果合理性检验
        bool isPathValid(
            const FrenetFrame& path_trajectory,
            std::string* reason = nullptr,
            size_t min_points = 5) const;

        bool isPathValid(
            const std::vector<TrajectoryPoint>& path_trajectory,
            std::string* reason,
            size_t min_points) const;

        // 纵向速度结果合理性检验
        bool isSpeedProfileValid(
            const std::vector<FrenetPoint>& speed_profile,
            std::string* reason = nullptr,
            size_t min_points = 5) const;

        // 轨迹合理性检验：用于上层判断规划结果是否可用
        // 返回 true 表示合理；false 表示不合理，并可返回原因
        bool isTrajectoryValid(
            const std::vector<TrajectoryPoint>& trajectory,
            std::string* reason = nullptr,
            size_t min_points = 5);

        // Tunable limits used by validity checks (defaults are conservative)
        void setLimits(double max_speed, double max_acc, double max_curvature, double max_jerk);
        void getLimits(double& max_speed, double& max_acc, double& max_curvature, double& max_jerk) const;

        // 获取上一周期的轨迹（供上层读取历史轨迹）
        const std::vector<TrajectoryPoint>& getPreviousTrajectory() const { return previous_trajectory_; }

    private:
        // limits
        double max_speed_ = 100.0;
        double max_acc_ = 50.0;
        double max_curvature_ = 10.0;
        double max_jerk_ = 50.0;


        // 获取拼接轨迹
        const std::vector<TrajectoryPoint>& getStitchedTrajectory() const { return stitch_trajectory_; }
        
        const std::vector<TrajectoryPoint>& getPreTrajectory() const {
            return previous_trajectory_;
        }

        // 清空历史轨迹（例如复位时）
        void reset() {
            stitch_trajectory_.clear();
            previous_trajectory_.clear();
            is_first_run_ = true;
            has_latest_ego_state_ = false;
        }

    private:
        template<typename... Args>
        void log(Args&&... args) const {
            if (logger_) logger_->log(std::forward<Args>(args)...);
        }

        std::shared_ptr<Logger> logger_;
        
        std::vector<TrajectoryPoint> previous_trajectory_;
        std::vector<TrajectoryPoint> stitch_trajectory_;
        bool is_first_run_ = true;
        double current_time_ = 0.0;
        int _pre_pnt_num = 100; // 拼接上一周期的点的数量

        VehicleState latest_ego_state_;
        bool has_latest_ego_state_ = false;
    };

}
}
