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
    // Rebuild static obstacles
    StaticObstacleClassification();
    // clear previous predictions and regenerate using the new start time
    predicted_obstacles_.clear();
    PredictObstacles();
    std::cout << "[Collision] Update: start_time=" << plan_start_time_ << ", detected=" << detected_objects_.size()
              << ", predicted_slots=" << predicted_obstacles_.size()
              << ", time_range=[" << plan_start_time_ << "," << (plan_start_time_ + params_.sampling.sample_max_time) << "]"
              << std::endl;
    if (!predicted_obstacles_.empty() && !predicted_obstacles_.front().empty()) {
      const auto &p0 = predicted_obstacles_.front().front();
      std::cout << "[Collision] first predicted obstacle at time=" << p0->time_stamp << " pos=(" << p0->x << "," << p0->y << ")" << std::endl;
    }
}

void latticeCollisionDetection::PredictObstaclesAtTime(double t) {
    std::vector<std::shared_ptr<AD_algorithm::general::Obstacle>> preds;
    preds.reserve(detected_objects_.size());
    for (const auto &obs : detected_objects_) {
        auto p = std::make_shared<AD_algorithm::general::Obstacle>(obs);
        p->x = obs.x + obs.vx * t;
        p->y = obs.y + obs.vy * t;
        p->time_stamp = obs.time_stamp + t;
        // keep velocities unchanged for now
        preds.push_back(p);
    }
    predicted_obstacles_.push_back(std::move(preds));
}

// 笛卡尔坐标系下预测所有障碍物的SLObstacle位置
void latticeCollisionDetection::PredictObstacles(){
    for(double t=plan_start_time_; t<=params_.sampling.sample_max_time+plan_start_time_; t+=params_.rank_dt){
        PredictObstaclesAtTime(t); 
    }
}

void latticeCollisionDetection::StaticObstacleClassification() {
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

double latticeCollisionDetection::DistanceToObstaclesAtTime(const general::TrajectoryPoint& traj, double t) const{
    if (predicted_obstacles_.empty()) return std::numeric_limits<double>::max();
    // 查找对应时间点的预测障碍物
    size_t index = static_cast<size_t>((t - plan_start_time_) / params_.rank_dt);
    if (index >= predicted_obstacles_.size()) {
        index = predicted_obstacles_.size() - 1;
    }
    const auto &preds = predicted_obstacles_[index];
    
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &p : preds) {
        min_dist = std::min(min_dist,p->minDistanceTo(traj));
    }
    std::cout << "[Collision] DistanceToObstaclesAtTime t=" << t << " index=" << index << " preds=" << preds.size() << " min_dist=" << min_dist << std::endl;
    return min_dist;
}

bool latticeCollisionDetection::HasOverlapWithStaticObstacles(const LatCandidate& latcandidate,double offset_s) const {
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
