#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include "general_modules/json.hpp"  
#include "general_modules/common_types.h"
#include <iostream>
#include <fstream>

namespace AD_algorithm {
namespace general {
class ControlLookupTable {
public:
    bool load(const std::string& bin_path, const std::string& json_path);

    // 双线性插值查询原始控制信号（正=油门，负=刹车）
    float query_control(double speed, double accel) const;

    // 辅助函数：直接返回 throttle 和 brake
    void query_throttle_brake(double speed, double accel, float& throttle, float& brake) const;
    
    bool can_use();
private:
    std::vector<float> data_;
    double speed_min_ = 0.0;
    double speed_max_ = 0.0;
    double accel_min_ = 0.0;
    double accel_max_ = 0.0;
    size_t speed_steps_ = 0;
    size_t accel_steps_ = 0;
    bool _flag=false;
};

class ControlCMD
{
public:
    ControlCMD()=default;
    ControlCMD(const std::string& bin_path, const std::string& json_path);
    bool set_steer(const double& steer);
    bool set_acceleration(const double& acceleration,const double &speed);
    double get_throttle() const;
    double get_brake() const;
    double get_steer() const;
    const double& operator[](size_t index) const ;

private:
    double _steer = 0.0;
    double _acceleration = 0.0;
    double _carla_steer = 0.0;
    double _vehicle_speed = 0.0;
    double _carla_throttle = 0.0;
    double _carla_brake = 0.0;
    ControlLookupTable _lut;
    void change_throttle_brake();
    void change_steer();

};

}}
