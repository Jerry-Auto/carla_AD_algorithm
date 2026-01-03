#include "lattice/lattice_collision_detection.h"
#include "general_modules/collision_detection.h"
#include <iostream>

namespace AD_algorithm {
namespace planner {

latticeCollisionDetection::latticeCollisionDetection() {
     vehicle_params_=AD_algorithm::general::VehicleParams();
     params_=PlannerParams();
}

void latticeCollisionDetection::setParameters(const PlannerParams& params) {
    params_=params; 
}

void latticeCollisionDetection::Update(const std::vector<Obstacle>& detected_objects,
                                       const std::shared_ptr<AD_algorithm::general::FrenetFrame>& ref_path,double start_time) {
    detected_objects_.clear();
    detected_objects_ = detected_objects;
    frenet_frame_ = ref_path;
    // 记录本次规划周期的起始时间，用于预测数组的时间对齐
    plan_start_time_ = start_time;
    StaticObstacleClassification();
    predicted_obstacles_.clear();
    PredictObstacles();
}

void latticeCollisionDetection::StaticObstacleClassification() {
    // 这里的静态障碍物是全局frenet框架下的静态障碍物
    static_obstacle_list_.clear();
    if (!frenet_frame_) return;

    // collect frenet polygons for static obstacles (speed small)
    std::vector<std::vector<AD_algorithm::general::FrenetPoint>> frenet_obs;
    for (const auto &obs : detected_objects_) {
        double speed = std::hypot(obs.vx, obs.vy);
        if (speed <=params_.collision.static_speed_threshold) {
            auto proj = frenet_frame_->project_obstacle_to_frenet(obs);
            if (!proj.empty()) frenet_obs.push_back(std::move(proj));
        }
    }

    if (!frenet_obs.empty()) {
        auto sls = AD_algorithm::general::FrenetFrame::convertToSLObstacles(frenet_obs, params_.collision.safe_distance);
        for (auto &s : sls) static_obstacle_list_.push_back(std::move(s));
    }
}

void latticeCollisionDetection::PredictObstaclesAtTime(double t_abs) {
    // t_abs 是绝对时间戳：预测在绝对时间 t_abs 时刻的障碍物位置
    std::vector<std::shared_ptr<AD_algorithm::general::Obstacle>> preds;
    preds.reserve(detected_objects_.size());
    for (const auto &obs : detected_objects_) {
        auto p = std::make_shared<AD_algorithm::general::Obstacle>(obs);
        // dt: 相对该障碍物测量时刻的时间差（秒）
        double dt = t_abs - obs.time_stamp;
        if (dt < 0.0) dt = 0.0;  // 防御：若时间没对齐，至少不回推
        p->x = obs.x + obs.vx * dt;
        p->y = obs.y + obs.vy * dt;
        // 预测状态的时间戳就是绝对时间 t_abs
        p->time_stamp = t_abs;
        // keep velocities unchanged for now
        // p->vx = obs.vx; p->vy = obs.vy; ... (拷贝构造已做)
        preds.push_back(std::move(p));
    }
    predicted_obstacles_.push_back(std::move(preds));
    // 生成离线表格直接查询
}

void latticeCollisionDetection::PredictObstacles() {
    predicted_obstacles_.clear();
    // 从规划时间起点开始预测：t_abs 为绝对时间
    const double t0 = plan_start_time_;
    const double t1 = plan_start_time_ + params_.sampling.sample_max_time;
    // 防御：rank_dt 必须 > 0
    if (params_.rank_dt <= 0.0) return;
    // 用 <= 时加一点点余量防止浮点误差漏掉最后一帧
    for (double t_abs = t0; t_abs <= t1 + 1e-12; t_abs += params_.rank_dt) {
        PredictObstaclesAtTime(t_abs);
    }
}

double latticeCollisionDetection::DistanceToObstaclesAtTime(const general::TrajectoryPoint& traj, double t_abs) const {
    // traj 的时间间隔与表格的时间间隔对齐：索引按 (t_abs - plan_start_time_) / dt
    if (predicted_obstacles_.empty()) return std::numeric_limits<double>::max();
    // 防御：t_abs 早于起点时刻，直接取第 0 帧
    if (t_abs <= plan_start_time_) {
        t_abs = plan_start_time_;
    }
    size_t index = static_cast<size_t>((t_abs - plan_start_time_) / params_.rank_dt);
    if (index >= predicted_obstacles_.size()) {
        index = predicted_obstacles_.size() - 1;
    }
    const auto &preds = predicted_obstacles_[index];
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &p : preds) {
        min_dist = std::min(min_dist, p->minDistanceTo(traj));
    }
    return min_dist;
}

bool latticeCollisionDetection::HasOverlapWithStaticObstacles(const LatCandidate& latcandidate,double offset_s) const {
    // 候选横向路径的s起点是0，使用的是相对于规划起点的坐标
    if (static_obstacle_list_.empty()) return false;
    if (!latcandidate.curve) return false;

    double s_end = latcandidate.param_s;
    if (s_end <= 0.0) return false;

    for (double s = 0.0; s <= s_end; s += params_.collision.collide_ds) {
        double l = latcandidate.curve->value_evaluation(s, 0);
        for (const auto &obs : static_obstacle_list_) {
            // 做一个简易的碰撞检测：检查点 (s,l) 是否在障碍物内
            if (obs.contains(s+offset_s, l, obs.safety_margin)) return true;
        }
    }
    return false;
}

} // namespace planner
} // namespace AD_algorithm
