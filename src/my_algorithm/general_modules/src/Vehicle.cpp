/* Copyright 2025 <Your Name> */

#include "general_modules/Vehicle.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <algorithm> 

namespace AD_algorithm {
namespace general {

// ==========================================
// VehicleState Implementation
// ==========================================
VehicleState& VehicleState::operator=(const VehicleState& other) {
    if (this != &other) {  // 防止自赋值
        x = other.x;
        y = other.y;
        z = other.z;
        heading = other.heading;
        v = other.v;
        ax = other.ax;
        ay = other.ay;
        omega = other.omega;
        alpha = other.alpha;
        time_stamp = other.time_stamp;
        id = other.id;
        flag_imu = other.flag_imu;
        flag_ode = other.flag_ode;
        flag_info = other.flag_info;
        road_width_left_vec = other.road_width_left_vec;
        road_width_right_vec = other.road_width_right_vec;
        road_width_resolution = other.road_width_resolution;
    }
    return *this;
}

// ==========================================
// ControlLookupTable Implementation
// ==========================================
bool ControlLookupTable::load(const std::string& bin_path, const std::string& json_path) {
    // Load JSON metadata
    std::ifstream meta_file(json_path);
    if (!meta_file.is_open()) {
        std::cerr << "❌ Cannot open JSON: " << json_path << "\n";
        return false;
    }
    nlohmann::json meta;
    meta_file >> meta;

    speed_min_ = meta["speed_min"];
    speed_max_ = meta["speed_max"];
    accel_min_ = meta["accel_min"];
    accel_max_ = meta["accel_max"];
    speed_steps_ = meta["speed_steps"];
    accel_steps_ = meta["accel_steps"];

    // Load binary data
    std::ifstream bin_file(bin_path, std::ios::binary);
    if (!bin_file.is_open()) {
        std::cerr << "❌ Cannot open BIN: " << bin_path << "\n";
        return false;
    }

    size_t total = speed_steps_ * accel_steps_;
    data_.resize(total);
    bin_file.read(reinterpret_cast<char*>(data_.data()), total * sizeof(float));
    if (!bin_file.good()) {
        std::cerr << "❌ Failed to read binary data\n";
        return false;
    }
    _flag=true;
    std::cout << "✅ LUT loaded: " << speed_steps_ << "×" << accel_steps_ 
                << " (" << total * sizeof(float) / 1024 << " KB)\n";
    return true;
}

float ControlLookupTable::query_control(double speed, double accel) const {
    // Clamp to valid range
    speed = std::max(speed_min_, std::min(speed_max_, speed));
    accel = std::max(accel_min_, std::min(accel_max_, accel));

    // Normalize to [0, 1]
    double s_norm = (speed - speed_min_) / (speed_max_ - speed_min_);
    double a_norm = (accel - accel_min_) / (accel_max_ - accel_min_);

    // Map to index space [0, steps-1]
    double s_idx = s_norm * (speed_steps_ - 1);
    double a_idx = a_norm * (accel_steps_ - 1);

    int s0 = static_cast<int>(std::floor(s_idx));
    int a0 = static_cast<int>(std::floor(a_idx));
    int s1 = std::min(s0 + 1, static_cast<int>(speed_steps_ - 1));
    int a1 = std::min(a0 + 1, static_cast<int>(accel_steps_ - 1));

    // Helper lambda to get value at (si, ai)
    auto at = [&](int si, int ai) -> float {
        return data_[si * accel_steps_ + ai];
    };

    float v00 = at(s0, a0);
    float v01 = at(s0, a1);
    float v10 = at(s1, a0);
    float v11 = at(s1, a1);

    double ds = s_idx - s0;
    double da = a_idx - a0;

    return static_cast<float>(
        v00 * (1 - ds) * (1 - da) +
        v01 * (1 - ds) * da +
        v10 * ds * (1 - da) +
        v11 * ds * da
    );
}

void ControlLookupTable::query_throttle_brake(double speed, double accel, float& throttle, float& brake) const {
    float control = query_control(speed, accel);
    throttle = std::max(0.0f, control);
    brake    = std::max(0.0f, -control);
}

bool ControlLookupTable::can_use(){
    return _flag;
}

// ==========================================
// ControlCMD Implementation
// ==========================================
ControlCMD::ControlCMD(const std::string& bin_path, const std::string& json_path): _steer(0.0), _acceleration(0.0), _carla_steer(0.0),
      _vehicle_speed(0.0), _carla_throttle(0.0), _carla_brake(0.0) {
        if (!_lut.load(bin_path, json_path)) {
        std::cout<<"标定表加载失败\n";
        }
      }

bool ControlCMD::set_steer(const double& steer){
    _steer=steer;
    change_steer();
    return true;
}
bool ControlCMD::set_acceleration(const double& acceleration,const double &speed){
    _acceleration=acceleration;
    _vehicle_speed=speed;
    change_throttle_brake();
    return true;
}

void ControlCMD::change_throttle_brake(){
    // 查表构建加速度与油门刹车的映射,通过实验标定
    // thr or brk =f(v,a)
    if(_lut.can_use()){
        float thr,brk;
        _lut.query_throttle_brake(_vehicle_speed, _acceleration, thr, brk);
        _carla_throttle=thr;
        _carla_brake=brk;
    }
    else{
        if(_acceleration>0){
            _carla_brake=0;
            _carla_throttle=std::max(std::min(_acceleration, 1.0),0.0);
        }
        else{
            _carla_throttle=0;
            _carla_brake=std::max(std::min(-_acceleration * 0.1, 1.0),0.0);
        }
    }
}

void ControlCMD::change_steer(){
    // _carla_steer = _steer;

    double max_u = 45.0 * M_PI / 180.0;
    // 将前轮转角 u 转换为 CARLA 的归一化 steer 指令[-1,1]
    double steer_cmd = -_steer/ max_u;

    // 确保在 [-1, 1] 范围内（虽然上面 clamp 后理论上不会超）
    _carla_steer = std::min(std::max(steer_cmd,-1.0),1.0);
}

double ControlCMD::get_throttle() const{
    return _carla_throttle;
}
double ControlCMD::get_brake() const{
    return _carla_brake;
}
double ControlCMD::get_steer() const{
    return _carla_steer;
}

// const 版本：只读访问，0，1，2分别是方向盘转角，油门刹车
const double& ControlCMD::operator[](size_t index) const {
    switch (index) {
        case 0: return _carla_steer;
        case 1: return _carla_throttle;
        case 2: return _carla_brake;
        default: throw std::out_of_range("ControlCMD[]: index out of range (0-2)");
    }
}
}}
