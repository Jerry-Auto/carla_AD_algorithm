#ifndef EMPLANNER_PLANNER_WEIGHTS_H
#define EMPLANNER_PLANNER_WEIGHTS_H

#include <algorithm>

namespace AD_algorithm {
namespace planner {

// 权重系数结构体
struct WeightCoefficients {
    // 路径DP权重
    double path_dp_w_ref = 1.0;
    double path_dp_w_dl = 0.1;
    double path_dp_w_ddl = 0.01;
    double path_dp_w_dddl = 0.0001;
    double path_dp_w_obs = 5.0;

    // 路径QP权重
    double path_qp_w_ref = 3.0;
    double path_qp_w_dl = 10.0;
    double path_qp_w_ddl = 20.00;
    double path_qp_w_dddl = 10.0;
    double path_qp_w_mid = 0.02;
    double path_qp_w_l_end = 0.04;
    double path_qp_w_dl_end = 0.04;
    double path_qp_w_ddl_end = 0.04;

    // 速度DP权重
    double speed_dp_w_ref_speed = 0.5;
    double speed_dp_w_a = 0.001;
    double speed_dp_w_jerk = 0.0001;
    double speed_dp_w_obs = 0.8;

    // 速度QP权重
    double speed_qp_w_ref_s = 1.0;
    double speed_qp_w_ref_speed = 10.0;
    double speed_qp_w_target_speed = 8.0;
    double speed_qp_w_a = 0.1;
    double speed_qp_w_jerk = 20.0;

    // 当前时间应该规划未来的轨迹，规划起点距离当前的时间,要与规划频率保持一致
    double forward_predict_time = 0.2;

    double safety_margin = 3.0;
    double obs_safety_margin = 0.5;
    double longitudinal_buffer = 5.0;
};

struct PathPlannerConfig {
    double s_sample_distance = 10;
    int s_sample_number = 20;
    int dp_poly_pnt_num = 5;

    int l_sample_number = 13;

    double lane_width = 10.0;
    double car_width = 2.0;

    double qp_dense_path_interval = 1.0;
    double final_path_interval = 0.1;

    bool validate() const {
        if (s_sample_distance <= 0 || s_sample_number <= 0) return false;
        if (final_path_interval <= 0 || l_sample_number <= 0) return false;
        if (lane_width <= 0 || car_width <= 0) return false;
        if (qp_dense_path_interval <= 0) return false;
        return true;
    }
};

struct SpeedPlannerConfig {
    // DP sampling
    double t_step_init = 0.5;
    double s_step_init = 0.3;
    double t_max = 8.0;
    double increase_ratio = 0.2;
    double s_max = 1000.0;  // Increased from 10 to allow longer distance planning
    int dp_poly_pnt_num = 3;

    // QP densify
    double qp_dense_path_interval = 0.1;
    double final_path_interval = 0.1;

    // ===== Physical limits (make them realistic for stability) =====
    // max lateral acceleration: use a realistic value (m/s^2)
    // Typical passenger car comfortable ~2-3, aggressive maybe 6-8.
    double max_lateral_acc = 10.0;   // 实际应该小于10，这里为了提高过弯速度
    double min_curve_speed =120.0/3.6;
    double max_speed =60;

    // Longitudinal acceleration bounds (m/s^2)
    double max_acceleration = 100.0;  // was 100.0
    double max_deceleration = -100.0; // was -100.0

    // Curvature/jerk bounds used by validators (keep reasonable)
    double max_curvature = 3.0;     // ~1/3m, tune to your vehicle & map
    double max_jerk = 10000.0;         // m/s^3, 10左右

    bool validate() const {
        if (t_step_init <= 0 || t_max <= 0) return false;
        if (s_step_init <= 0) return false;
        if (increase_ratio < 0.0) return false;

        if (max_lateral_acc <= 0) return false;
        if (max_speed <= 0) return false;

        // Ensure accel bounds are sane
        if (!(max_deceleration <= 0.0 && max_acceleration >= 0.0)) return false;
        if (max_acceleration - max_deceleration <= 1e-6) return false;

        if (qp_dense_path_interval <= 0 || final_path_interval <= 0) return false;

        if (max_jerk <= 0.0) return false;
        if (max_curvature <= 0.0) return false;

        return true;
    }
};

} // namespace planner
} // namespace AD_algorithm

#endif // EMPLANNER_PLANNER_WEIGHTS_H