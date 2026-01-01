#ifndef EMPLANNER_PLANNER_WEIGHTS_H
#define EMPLANNER_PLANNER_WEIGHTS_H

namespace AD_algorithm {
namespace planner {

// 权重系数结构体
struct WeightCoefficients {
    // 路径DP权重
    double path_dp_w_ref = 1.0;
    double path_dp_w_dl = 0.01;
    double path_dp_w_ddl = 0.001;
    double path_dp_w_dddl = 0.0001;
    double path_dp_w_obs = 5.0;
    
    // 路径QP权重
    double path_qp_w_ref = 3.0;      // 降低参考线权重，允许为了避障提前偏离
    double path_qp_w_dl = 0.5;       // 降低航向误差权重，允许更容易地改变航向
    double path_qp_w_ddl = 10.00;     // 提高平滑度权重，促使轨迹更早开始弯曲以保持平滑
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
    double speed_qp_w_ref_s = 1.0;      // 位置跟踪权重（新增！）
    double speed_qp_w_ref_speed = 5.0;
    double speed_qp_w_target_speed = 10.0; // 目标速度跟踪权重
    double speed_qp_w_a = 0.05;
    double speed_qp_w_jerk = 0.2;


    //当前时间应该规划未来的轨迹，规划起点距离当前的时间,要与规划频率保持一致
    double forward_predict_time=0.1;   
    double safety_margin = 3.0;      // 计算障碍物代价的安全边界，边界内代价较高
    double obs_safety_margin = 0.5;      // 状态是否可行的安全边界，边界内视为不可行
};


struct PathPlannerConfig {
    double s_sample_distance = 10;   // s方向采样间隔
    int s_sample_number = 20;          // s方向采样层数
    int dp_poly_pnt_num = 5;           // DP两点间多项式拟合点数

    int l_sample_number = 13;          // l方向采样点数,应为奇数

    double lane_width = 10.0;          // 车道宽度
    double car_width = 2.0;          // 车辆宽度

    double qp_dense_path_interval=1.0;  // QP规划输入点间隔，两个点之间的距离
    double final_path_interval = 0.1;  // 最终路径点密度间隔，两个点之间的距离
    
    // 验证配置
    bool validate() const {
        if (s_sample_distance <= 0 || s_sample_number <= 0) {
            return false;
        }
        if (final_path_interval <= 0 || l_sample_number <= 0) {
            return false;
        }
        if (lane_width <= 0 || car_width <= 0) {
            return false;
        }
        return true;
    }
};


struct SpeedPlannerConfig {
    // 速度规划中，时间的采样间距不能太短，如果距离采样间距相对于时间来说太短会导致分母很小，计算得到的速度、加速度、jerk都会很大，然后不满足转移约束
    // 也就是说，距离采样可以尽量细化，时间的可以粗糙一点
    double t_step_init = 0.5;    // 时间采样初始间隔
    double s_step_init = 0.3;    // 距离采样初始间隔
    double t_max = 8.0;                // 规划时间长度
    double increase_ratio = 0.2;      // 采样间隔递增比例
    double s_max = 10;             // 最大规划距离,通过路径规划的配置来计算
    int dp_poly_pnt_num=3;        // DP两点间多项式拟合点数

    double qp_dense_path_interval=0.1;  // QP规划输入点间隔，两个点之间的距离
    double final_path_interval = 0.02;  // 最终路径点密度间隔，两个点之间的距离
 
    double max_lateral_acc = 1.5 * 9.8; // 最大横向加速度，直接影响过弯速度
    double max_speed = 120.0/3.6;          // 最大速度
    double max_acceleration = 4.0;
    double max_deceleration = -6.0;
    double max_curvature = 100.0;
    double max_jerk = 10.0;

    // 验证配置
    bool validate() const {
        if (t_step_init <= 0 || t_max <= 0) {
            return false;
        }
        if (max_lateral_acc <= 0) {
            return false;
        }
        return true;
    }
};

} // namespace planner
} // namespace AD_algorithm

#endif // EMPLANNER_PLANNER_WEIGHTS_H