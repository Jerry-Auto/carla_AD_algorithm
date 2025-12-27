#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <mutex>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <ctime>
#include "general_modules/Trajectory.h"

namespace AD_algorithm {
namespace general {

class CSVLogger {
public:
    static CSVLogger& getInstance() {
        static CSVLogger instance;
        return instance;
    }

    // 初始化日志目录
    void init(const std::string& log_dir, const std::string& role_name);

    // 规划模块日志接口
    void logPlanningTrajectory(
        const std::vector<TrajectoryPoint>& trajectory,
        double ref_speed,
        const std::string& stage,
        bool is_valid,
        const std::string& reason);

    // 控制模块指令日志接口
    void logControlCmd(
        double ros_time,
        double throttle,
        double brake,
        double steer,
        double ego_x,
        double ego_y,
        double ego_heading,
        double ego_v);

    // 控制模块轨迹跟踪日志接口
    void logControlTrajectory(
        double ros_time,
        const std::vector<TrajectoryPoint>& trajectory);

private:
    CSVLogger() = default;
    ~CSVLogger();
    CSVLogger(const CSVLogger&) = delete;
    CSVLogger& operator=(const CSVLogger&) = delete;

    std::string nowTimestampForFilename();
    bool openCsvAppendWithHeader(std::ofstream& ofs, const std::string& path, const std::string& header);
    std::string csvEscape(const std::string& s);

    std::string _log_dir;
    std::string _role_name;
    bool _initialized = false;

    // 规划日志
    std::ofstream _planning_ofs;
    std::mutex _planning_mutex;
    uint64_t _plan_seq = 0;

    // 控制指令日志
    std::ofstream _control_cmd_ofs;
    std::mutex _control_cmd_mutex;
    uint64_t _control_cmd_seq = 0;

    // 控制轨迹日志
    std::ofstream _control_traj_ofs;
    std::mutex _control_traj_mutex;
    uint64_t _control_traj_seq = 0;
};

} // namespace general
} // namespace AD_algorithm
