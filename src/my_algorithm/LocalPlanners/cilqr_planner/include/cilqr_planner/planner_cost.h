#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "general_modules/cilqr.h"
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"
#include "cilqr_planner/planner_weight.h"

namespace AD_algorithm {
namespace planner {

class planner_cost : public AD_algorithm::general::cost_factory {
public:
    planner_cost(const std::shared_ptr<CILQRPlannerparams>& params, int n_x=4, int n_u=2)
        : planner_params_(params),
          n_x_(n_x),
          n_u_(n_u),
          obs_preds_(),
          ref_velo_(0.0),
          lower_bound_(Eigen::Vector4d(params->road_lower_bound, params->min_a, params->min_v,
                                       params->min_steer)),
          upper_bound_(Eigen::Vector4d(params->road_upper_bound, params->max_a, params->max_v,
                                       params->max_steer)),
          Q_(Eigen::MatrixXd::Identity(n_x, n_x)),
          R_(Eigen::MatrixXd::Identity(n_u, n_u)),
          Qf_(Eigen::MatrixXd::Identity(n_x, n_x)),
          initial_state_(),
          frenet_frame_(nullptr),
          shut_down_obstacles_(false) {}

    ~planner_cost() = default;
    
    virtual void set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles) = 0;
 
    void set_frenet_frame(std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame) {
        frenet_frame_ = frenet_frame;
        is_first_run_ = true;
    }
    void set_reference_speed(double v_ref) { ref_velo_ = v_ref; }
    void set_initial_state(const general::VehicleState& x0) { initial_state_ = x0; }
    void set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf) {
        Q_ = Q;
        R_ = R;
        Qf_ = Qf;
    }
    void set_road_bounds(const double& lower_bound, const double& upper_bound) {
        planner_params_->road_lower_bound = lower_bound;
        planner_params_->road_upper_bound = upper_bound;
        lower_bound_[0] = lower_bound;
        upper_bound_[0] = upper_bound;
    }
    
    
    void get_initial_trajectory(Eigen::MatrixXd& x_init, Eigen::MatrixXd& u_init) override
    {
        const int N = planner_params_->ilqr_params->N;
        x_init = Eigen::MatrixXd::Zero(N + 1, n_x_);
        u_init = Eigen::MatrixXd::Zero(N, n_u_);

        Eigen::Vector4d cur_x(initial_state_.x, initial_state_.y, initial_state_.heading,
                            initial_state_.v);
        x_init.row(0) = cur_x.transpose();
        if (!is_first_run_ && last_u_.rows() == N && last_u_.cols() == n_u_) {
            u_init.block(0, 0, N - 1, n_u_) = last_u_.block(1, 0, N - 1, n_u_);
            u_init.block(N - 1, 0, 1, n_u_) = last_u_.block(N - 1, 0, 1, n_u_);
        }
        is_first_run_ = false;

        for (int k = 0; k < N; ++k) {
            Eigen::VectorXd state_and_control(n_x_ + n_u_);
            state_and_control << cur_x, u_init.row(k).transpose();
            cur_x = state_transition(state_and_control);
            x_init.row(k + 1) = cur_x.transpose();
        }
    }

protected:
    std::shared_ptr<CILQRPlannerparams> planner_params_;
    int n_x_, n_u_;
    std::vector<std::vector<general::Obstacle>> obs_preds_;
    double ref_velo_;
    //不等式边界约束项[road_upper_bound, a_max, v_max, steer_max],每次只需要更新道路边界即可
    Eigen::VectorXd lower_bound_, upper_bound_;  
    Eigen::MatrixXd Q_, R_, Qf_;
    general::VehicleState initial_state_;
    std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame_;
    bool shut_down_obstacles_;
    bool is_first_run_ = true;
};

}  // namespace planner
}  // namespace AD_algorithm
