#pragma once
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"


namespace AD_algorithm {
namespace general {
    class TrajectoryManager {
    public:
        TrajectoryManager();
        ~TrajectoryManager() = default;

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

        // 路径QP结果合理性检验
        bool isPathValid(
            const FrenetFrame& path_trajectory,
            std::string* reason = nullptr,
            size_t min_points = 5) const;

        // 速度QP结果合理性检验
        bool isSpeedProfileValid(
            const std::vector<STPoint>& speed_profile,
            std::string* reason = nullptr,
            size_t min_points = 5) const;

        // 轨迹合理性检验：用于上层判断规划结果是否可用
        // 返回 true 表示合理；false 表示不合理，并可返回原因
        bool isTrajectoryValid(
            const std::vector<TrajectoryPoint>& trajectory,
            std::string* reason = nullptr,
            size_t min_points = 5);


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

        // 日志输出
        void set_log_enable(bool enable) { enable_logging_ = enable; }
        void setLogCallback(std::function<void(const std::string&)> callback) {
            log_callback_ = callback;
        }

    private:
        void log(const std::string& message, const std::string& level = "INFO");

        std::vector<TrajectoryPoint> previous_trajectory_;
        std::vector<TrajectoryPoint> stitch_trajectory_;
        bool is_first_run_ = true;
        double current_time_ = 0.0;
        bool enable_logging_ = true;
        int _pre_pnt_num = 500; // 拼接上一周期的点的数量
        std::function<void(const std::string&)> log_callback_;

        VehicleState latest_ego_state_;
        bool has_latest_ego_state_ = false;
    };

}
}
