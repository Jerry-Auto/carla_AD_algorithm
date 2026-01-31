#ifndef CILQR_PLANNER_WEIGHTS_H
#define CILQR_PLANNER_WEIGHTS_H
#include <memory>
#include "general_modules/cilqr.h"
#include "general_modules/common_types.h"


namespace AD_algorithm {
namespace planner {

struct CILQRPlannerparams {
    CILQRPlannerparams() {
        ilqr_params = std::make_shared<AD_algorithm::general::ILQR_params>();
        ilqr_params->N =30;            // horizon length，不包括终端状态
        ilqr_params->dt = 0.1;       // time step
        ilqr_params->max_iter = 100;      // 最大迭代次数
        ilqr_params->use_DDP = true;  // 使用DDP
        ilqr_params->nx = 4;             // state dimension
        ilqr_params->nu = 2;             // control dimension

        AD_algorithm::general::VehicleParams vp;
        vehicle_width = vp.width;
        vehicle_length = vp.length;
        wheelbase = vp.lf + vp.lr;
        safety_distance = vp.safety_distance;
    }

    bool for_control =true; // 是否用于控制跟踪,否则用于轨迹规划
    std::shared_ptr<AD_algorithm::general::ILQR_params> ilqr_params;

    bool use_alm = false;    // 是否使用ALM
    // 边界约束的惩罚系数初始化
    double alm_rho_init=20;
    double alm_beta = 2;  // 惩罚系数增长率
    double alm_rho_max=100; // 惩罚系数上限
    double alm_mu_max=100;  // 乘子裁剪上限
    // 使用指数障碍函数法，两个参数(q1）exp(q2*c),q1越大惩罚越大，q2越大惩罚越大
    double barrier_q1_init=3.0;  //内部的惩罚系数
    double barrier_beta1 = 1; // 障碍函数惩罚系数缩减率
    double barrier_q2_init=3.6;  // 外部的惩罚系数
    double barrier_beta2 = 1; // 障碍函数惩罚系数缩减率
    double barrier_q1_max = 3.0;  // 障碍系数上限
    double barrier_q2_max = 3.6;  // 障碍系数上限

    // 障碍物避免惩罚函数参数
    // ALM parameters for obstacles
    double obstacle_alm_rho_init=30.0;
    double obstacle_alm_rho_beta=4.0;
    double obstacle_alm_rho_max=300.0;
    double obstacle_alm_mu_max=200.0;
    // expotential barrier parameters for obstacles
    double obstacle_barrier_q1_init=5.5;
    double obstacle_barrier_beta1=1; 
    double obstacle_barrier_q2_init=5.75;
    double obstacle_barrier_beta2=1;
    double obstacle_barrier_q1_max = 5.5;  // 障碍系数上限
    double obstacle_barrier_q2_max = 5.75;  // 障碍系数上限

    // 不等式边界默认值
    double road_upper_bound = 5.0;    //左侧道路边界
    double road_lower_bound = -5.0;   //右侧道路边界
    double max_a = 6.0;
    double min_a = -6.0;
    double max_v = 80.0;  // m/s
    double min_v = 0.0;
    double max_steer = 0.12;
    double min_steer = -0.12;

    // 车辆几何参数
    double vehicle_width = 2.0;
    double vehicle_length = 4.5;
    double wheelbase = 2.8;
    double safety_distance = 0.9;
};

} // namespace planner
} // namespace AD_algorithm

#endif // CILQR_PLANNER_WEIGHTS_H