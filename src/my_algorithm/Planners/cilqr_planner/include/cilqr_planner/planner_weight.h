#ifndef LATTICE_PLANNER_WEIGHTS_H
#define LATTICE_PLANNER_WEIGHTS_H
#include <memory>
#include "general_modules/cilqr.h"

namespace AD_algorithm {
namespace planner {

struct CILQRPlannerparams {
    CILQRPlannerparams() {
        ilqr_params = std::make_shared<AD_algorithm::general::ILQR_params>();
        ilqr_params->N = 160;            // horizon length，不包括终端状态
        ilqr_params->dt = 0.05;       // time step
        ilqr_params->max_iter = 60;      // 最大迭代次数
        ilqr_params->use_DDP = true;  // 使用DDP
        ilqr_params->nx = 4;             // state dimension
        ilqr_params->nu = 2;             // control dimension
    }
    
    std::shared_ptr<AD_algorithm::general::ILQR_params> ilqr_params;
    bool use_alm = true;    // 是否使用ALM
    double alm_rho_init=1.0;
    double alm_beta = 10;  // 惩罚系数增长率
    double barrier_rho_init=1.0;
    double barrier_beta = 0.8; // 障碍函数惩罚系数缩减率

    // 不等式边界默认值
    double road_upper_bound = 10.0;
    double road_lower_bound = -10.0;
    double max_a = 10.0;
    double min_a = -10.0;
    double max_v = 150.0;
    double min_v = -1.0;
    double max_steer = 0.5;
    double min_steer = -0.5;
};

} // namespace planner
} // namespace AD_algorithm

#endif // LATTICE_PLANNER_WEIGHTS_H