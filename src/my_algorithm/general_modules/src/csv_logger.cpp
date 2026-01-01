/* Copyright 2025 <Your Name> */

#include "general_modules/csv_logger.h"
#include <iostream> 

namespace AD_algorithm {
namespace general {

void CSVLogger::init(const std::string& log_dir, const std::string& role_name) {
    if (_initialized) return;
    
    _log_dir = log_dir;
    _role_name = role_name;

    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(_log_dir, ec);
    if (ec) {
        std::cerr << "[CSVLogger] Failed to create directory: " << _log_dir << std::endl;
        return;
    }

    std::string ts = nowTimestampForFilename();

    // 1. 规划日志
    std::string plan_path = _log_dir + "/planning_" + _role_name + "_" + ts + ".csv";
    std::string plan_header = "seq,stage,valid,reason,idx,x,y,heading,kappa,v,ax,ay,a_tau,time_stamped,ref_speed";
    openCsvAppendWithHeader(_planning_ofs, plan_path, plan_header);

    // 2. 控制指令日志
    std::string cmd_path = _log_dir + "/control_cmd_" + _role_name + "_" + ts + ".csv";
    std::string cmd_header = "seq,ros_time,throttle,brake,steer,ego_x,ego_y,ego_heading,ego_v";
    openCsvAppendWithHeader(_control_cmd_ofs, cmd_path, cmd_header);

    // 3. 控制轨迹日志
    std::string traj_path = _log_dir + "/control_traj_" + _role_name + "_" + ts + ".csv";
    std::string traj_header = "seq,ros_time,idx,x,y,heading,kappa,v,ax,ay,a_tau,time_stamped";
    openCsvAppendWithHeader(_control_traj_ofs, traj_path, traj_header);

    _initialized = true;
}

void CSVLogger::logPlanningTrajectory(
    const std::vector<TrajectoryPoint>& trajectory,
    double ref_speed,
    const std::string& stage,
    bool is_valid,
    const std::string& reason) 
{
    if (!_initialized || !_planning_ofs.is_open()) return;

    std::lock_guard<std::mutex> lock(_planning_mutex);
    ++_plan_seq;
    std::string reason_esc = csvEscape(reason);
    int valid_flag = is_valid ? 1 : 0;

    if (trajectory.empty()) {
        _planning_ofs << _plan_seq << "," << stage << "," << valid_flag << "," << reason_esc 
                      << ",,,,,,,,,,," << ref_speed << "\n";
    } else {
        for (size_t i = 0; i < trajectory.size(); ++i) {
            const auto& p = trajectory[i];
            _planning_ofs << _plan_seq << "," << stage << "," << valid_flag << "," << reason_esc << ","
                          << i << ","
                          << p.x << "," << p.y << "," << p.heading << "," << p.kappa << ","
                          << p.v << "," << p.ax << "," << p.ay << "," << p.a_tau << ","
                          << p.time_stamped << "," << ref_speed << "\n";
        }
    }
    _planning_ofs.flush();
}

void CSVLogger::logControlCmd(
    double ros_time,
    double throttle,
    double brake,
    double steer,
    double ego_x,
    double ego_y,
    double ego_heading,
    double ego_v)
{
    if (!_initialized || !_control_cmd_ofs.is_open()) return;

    std::lock_guard<std::mutex> lock(_control_cmd_mutex);
    ++_control_cmd_seq;
    _control_cmd_ofs << _control_cmd_seq << "," << std::fixed << std::setprecision(6) << ros_time << ","
                     << throttle << "," << brake << "," << steer << ","
                     << ego_x << "," << ego_y << "," << ego_heading << "," << ego_v << "\n";
    _control_cmd_ofs.flush();
}

void CSVLogger::logControlTrajectory(
    double ros_time,
    const std::vector<TrajectoryPoint>& trajectory)
{
    if (!_initialized || !_control_traj_ofs.is_open()) return;

    std::lock_guard<std::mutex> lock(_control_traj_mutex);
    ++_control_traj_seq;
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& p = trajectory[i];
        _control_traj_ofs << _control_traj_seq << "," << std::fixed << std::setprecision(6) << ros_time << ","
                          << i << ","
                          << p.x << "," << p.y << "," << p.heading << "," << p.kappa << ","
                          << p.v << "," << p.ax << "," << p.ay << "," << p.a_tau << ","
                          << p.time_stamped << "\n";
    }
    _control_traj_ofs.flush();
}

std::string CSVLogger::nowTimestampForFilename() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

bool CSVLogger::openCsvAppendWithHeader(std::ofstream& ofs, const std::string& path, const std::string& header) {
    namespace fs = std::filesystem;
    bool need_header = true;
    std::error_code ec;
    if (fs::exists(path, ec) && !ec) {
        need_header = (fs::file_size(path, ec) == 0);
    }

    ofs.open(path, std::ios::out | std::ios::app);
    if (!ofs.is_open()) return false;
    
    if (need_header) {
        ofs << header << "\n";
        ofs.flush();
    }
    return true;
}

std::string CSVLogger::csvEscape(const std::string& s) {
    bool need_quote = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') {
            need_quote = true;
            break;
        }
    }
    if (!need_quote) return s;
    
    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

CSVLogger::~CSVLogger() {
    if (_planning_ofs.is_open()) _planning_ofs.close();
    if (_control_cmd_ofs.is_open()) _control_cmd_ofs.close();
    if (_control_traj_ofs.is_open()) _control_traj_ofs.close();
}

} // namespace general
} // namespace AD_algorithm
