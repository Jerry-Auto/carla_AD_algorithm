#pragma once

#include <string>
#include <functional>
#include <memory>
#include <sstream>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace AD_algorithm {
namespace general {

/**
 * @brief 独立的日志记录器类 (基于 spdlog)
 */
class Logger {
public:
    Logger(const std::string& name = "AD_Algorithm") : name_(name) {
        init_spdlog();
    }

    /**
     * @brief 设置日志开关
     */
    void set_enable(bool enable) { 
        enable_logging_ = enable; 
        if (spd_logger_) {
            spd_logger_->set_level(enable ? spdlog::level::trace : spdlog::level::off);
        }
    }

    /**
     * @brief 核心日志函数 (spdlog 风格)
     */
    template<typename... Args>
    void info(spdlog::string_view_t fmt, Args&&... args) {
        if (enable_logging_ && spd_logger_) spd_logger_->info(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void warn(spdlog::string_view_t fmt, Args&&... args) {
        if (enable_logging_ && spd_logger_) spd_logger_->warn(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void error(spdlog::string_view_t fmt, Args&&... args) {
        if (enable_logging_ && spd_logger_) spd_logger_->error(fmt, std::forward<Args>(args)...);
    }

    template<typename... Args>
    void debug(spdlog::string_view_t fmt, Args&&... args) {
        if (enable_logging_ && spd_logger_) spd_logger_->debug(fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 兼容旧代码的变长参数打印 (空格分隔)
     */
    template<typename... Args>
    void log(const std::string& level, Args&&... args) {
        if (!enable_logging_ || !spd_logger_) return;

        std::ostringstream oss;
        int i = 0;
        ((oss << (i++ > 0 ? " " : "") << args), ...);
        
        std::string msg = oss.str();
        if (level == "ERROR") spd_logger_->error("{}", msg);
        else if (level == "WARN") spd_logger_->warn("{}", msg);
        else if (level == "DEBUG") spd_logger_->debug("{}", msg);
        else spd_logger_->info("{}", msg);
    }

private:
    void init_spdlog() {
        spd_logger_ = spdlog::get(name_);
        if (!spd_logger_) {
            spd_logger_ = spdlog::stdout_color_mt(name_);
            spd_logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        }
    }

    std::string name_;
    std::shared_ptr<spdlog::logger> spd_logger_;
    bool enable_logging_ = true;
};

} // namespace general
} // namespace AD_algorithm
