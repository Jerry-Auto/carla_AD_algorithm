#include "cilqr_planner/cost_factory.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace AD_algorithm {
namespace planner {

cilqr_eigen::cilqr_eigen(const std::shared_ptr<CILQRPlannerparams>& params)
    : planner_cost(params, params->ilqr_params->nx, params->ilqr_params->nu),
    alm_mu_must_(),
    alm_mu_must_next_(),
    alm_mu_obstacles_(),
    alm_mu_obstacles_next_(),
    alm_rho_(params->alm_rho_init),
    obstacle_alm_rho_(params->obstacle_alm_rho_init),
    solve_type_(params->use_alm ? SolveType::ALM : SolveType::BARRIER),
    barrier_q1_(params->barrier_q1_init),
    barrier_q2_(params->barrier_q2_init),
    obstacle_barrier_q1_(params->obstacle_barrier_q1_init),
    obstacle_barrier_q2_(params->obstacle_barrier_q2_init),
    num_c_(8),
    reference_point_(ReferencePoint::GravityCenter),
    l_x_(Eigen::MatrixXd::Zero(params->ilqr_params->N + 1, n_x_)),
    l_u_(Eigen::MatrixXd::Zero(params->ilqr_params->N, n_u_)),
    l_xx_(Eigen::MatrixXd::Zero((params->ilqr_params->N + 1) * n_x_, n_x_)),
    l_uu_(Eigen::MatrixXd::Zero(params->ilqr_params->N * n_u_, n_u_)),
    l_xu_(Eigen::MatrixXd::Zero((params->ilqr_params->N + 1) * n_x_, n_u_)),
    l_ux_(Eigen::MatrixXd::Zero(params->ilqr_params->N * n_u_, n_x_)) {
    if (solve_type_ == SolveType::ALM) {
        alm_rho_ = planner_params_->alm_rho_init;
        obstacle_alm_rho_ = planner_params_->obstacle_alm_rho_init;
        alm_mu_must_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
        alm_mu_must_next_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
        alm_mu_obstacles_ =
            Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, 2 * obs_preds_.size());
        alm_mu_obstacles_next_ =
            Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, 2 * obs_preds_.size());
    } else {
        barrier_q1_ = planner_params_->barrier_q1_init;
        barrier_q2_ = planner_params_->barrier_q2_init;
        obstacle_barrier_q1_ = planner_params_->obstacle_barrier_q1_init;
        obstacle_barrier_q2_ = planner_params_->obstacle_barrier_q2_init;
        alm_mu_must_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
        alm_mu_must_next_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, num_c_);
        alm_mu_obstacles_ =
            Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, 2 * obs_preds_.size());
        alm_mu_obstacles_next_ =
            Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N, 2 * obs_preds_.size());
    }
}

void cilqr_eigen::set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles) {
    obs_preds_ = obstacles;
    shut_down_obstacles_ = obs_preds_.empty();

    const int N = planner_params_->ilqr_params->N;
    if (solve_type_ == SolveType::ALM) {
        alm_rho_ = planner_params_->alm_rho_init;
        obstacle_alm_rho_ = planner_params_->obstacle_alm_rho_init;
        alm_mu_must_ = Eigen::MatrixXd::Zero(N, num_c_);
        alm_mu_must_next_ = Eigen::MatrixXd::Zero(N, num_c_);
        alm_mu_obstacles_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
        alm_mu_obstacles_next_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
    } else {
        barrier_q1_ = planner_params_->barrier_q1_init;
        barrier_q2_ = planner_params_->barrier_q2_init;
        obstacle_barrier_q1_ = planner_params_->obstacle_barrier_q1_init;
        obstacle_barrier_q2_ = planner_params_->obstacle_barrier_q2_init;
        alm_mu_must_ = Eigen::MatrixXd::Zero(N, num_c_);
        alm_mu_must_next_ = Eigen::MatrixXd::Zero(N, num_c_);
        alm_mu_obstacles_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
        alm_mu_obstacles_next_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
    }
}


Eigen::VectorXd cilqr_eigen::state_transition(const Eigen::VectorXd& state_and_control) {
    Eigen::Vector4d x = state_and_control.head(n_x_);
    Eigen::Vector2d u = state_and_control.tail(n_u_);
    const double dt = planner_params_->ilqr_params->dt;
    const double beta = atan(tan(u[1]) / 2.0);

    Eigen::Vector4d x_next;
    if (reference_point_ == ReferencePoint::RearCenter) {
        x_next << x[0] + x[3] * cos(x[2]) * dt,
            x[1] + x[3] * sin(x[2]) * dt,
            x[2] + x[3] * tan(u[1]) * dt / planner_params_->wheelbase,
            x[3] + u[0] * dt;
    } else {
        x_next << x[0] + x[3] * cos(beta + x[2]) * dt,
            x[1] + x[3] * sin(beta + x[2]) * dt,
            x[2] + 2.0 * x[3] * sin(beta) * dt / planner_params_->wheelbase,
            x[3] + u[0] * dt;
    }
    return x_next;
}


Eigen::Tensor<double, 3,Eigen::ColMajor> cilqr_eigen::state_total_jacobian(const Eigen::MatrixXd& x_and_u) {
    const int N = planner_params_->ilqr_params->N;
    const int nz = n_x_ + n_u_;
    Eigen::Tensor<double, 3, Eigen::ColMajor> jacobians(N, n_x_, nz);
    const double dt = planner_params_->ilqr_params->dt;

    for (int k = 0; k < N; ++k) {
        const Eigen::Vector4d x = x_and_u.row(k).head(n_x_).transpose();
        const Eigen::Vector2d u = x_and_u.row(k).segment(n_x_, n_u_).transpose();
        const double delta = u[1];
        const double beta = atan(tan(delta) / 2.0);
        const double beta_over_stl =
            0.5 * (1 + std::pow(tan(delta), 2)) / (1 + 0.25 * std::pow(tan(delta), 2));

        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_x_, nz);
        if (reference_point_ == ReferencePoint::RearCenter) {
            J(0, 0) = 1.0;
            J(1, 1) = 1.0;
            J(2, 2) = 1.0;
            J(3, 3) = 1.0;

            J(0, 2) = -x[3] * sin(x[2]) * dt;
            J(0, 3) = cos(x[2]) * dt;
            J(1, 2) = x[3] * cos(x[2]) * dt;
            J(1, 3) = sin(x[2]) * dt;
            J(2, 3) = tan(delta) * dt / planner_params_->wheelbase;

            J(2, n_x_ + 1) = (x[3] * dt / planner_params_->wheelbase) / (cos(delta) * cos(delta));
            J(3, n_x_) = dt;
        } else {
            J(0, 0) = 1.0;
            J(1, 1) = 1.0;
            J(2, 2) = 1.0;
            J(3, 3) = 1.0;

            J(0, 2) = -x[3] * sin(beta + x[2]) * dt;
            J(0, 3) = cos(beta + x[2]) * dt;
            J(1, 2) = x[3] * cos(beta + x[2]) * dt;
            J(1, 3) = sin(beta + x[2]) * dt;
            J(2, 3) = 2.0 * sin(beta) * dt / planner_params_->wheelbase;

            J(0, n_x_ + 1) = x[3] * (-sin(beta + x[2])) * dt * beta_over_stl;
            J(1, n_x_ + 1) = x[3] * cos(beta + x[2]) * dt * beta_over_stl;
            J(2, n_x_ + 1) = (2.0 * x[3] * dt / planner_params_->wheelbase) * cos(beta) * beta_over_stl;
            J(3, n_x_) = dt;
        }

        for (int i = 0; i < n_x_; ++i) {
            for (int j = 0; j < nz; ++j) {
                jacobians(k, i, j) = J(i, j);
            }
        }
    }
    return jacobians;
}

Eigen::Tensor<double, 4,Eigen::ColMajor> cilqr_eigen::state_total_hessian(const Eigen::MatrixXd& x_and_u) {
    const int N = planner_params_->ilqr_params->N;
    const int nz = n_x_ + n_u_;
    Eigen::Tensor<double, 4, Eigen::ColMajor> hessians(N, n_x_, nz, nz);
    hessians.setZero();
    return hessians;
}

double cilqr_eigen::compute_total_cost(const Eigen::MatrixXd& x_and_u) {
    const int N = planner_params_->ilqr_params->N;
    const size_t num_obstacles = obs_preds_.size();
    Eigen::MatrixXd x = x_and_u.block(0, 0, N + 1, n_x_);
    Eigen::MatrixXd u = x_and_u.block(0, n_x_, N, n_u_);

    Eigen::MatrixX3d ref_exact_points = Eigen::MatrixX3d::Zero(N + 1, 3);
    for (int k = 0; k < N + 1; ++k) {
        if (frenet_frame_) {
            auto matched = frenet_frame_->get_matched_point(x(k, 0), x(k, 1), x(k, 2));
            ref_exact_points(k, 0) = matched.x;
            ref_exact_points(k, 1) = matched.y;
            ref_exact_points(k, 2) = matched.heading;
        } else {
            ref_exact_points(k, 0) = x(k, 0);
            ref_exact_points(k, 1) = x(k, 1);
            ref_exact_points(k, 2) = x(k, 2);
        }
    }
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo_);
    Eigen::MatrixXd ref_states(N + 1, n_x_);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_exact_points.col(2),
        ref_velocitys;

    double states_devt = ((x - ref_states) * Q_ * (x - ref_states).transpose()).trace();
    double ctrl_energy = (u * R_ * u.transpose()).trace();
    double J_prime = states_devt + ctrl_energy;

    double J_barrier = 0.0;
    for (int k = 1; k < N + 1; ++k) {
        Eigen::Vector2d u_k = u.row(k - 1).transpose();
        Eigen::Vector4d x_k = x.row(k).transpose();
        Eigen::Vector3d ref_x_k = ref_exact_points.row(k).transpose();

        double acc_up_constr = get_bound_constr(u_k[0], planner_params_->max_a, BoundType::UPPER);
        double acc_lo_constr = get_bound_constr(u_k[0], planner_params_->min_a, BoundType::LOWER);
        double stl_up_constr = get_bound_constr(u_k[1], planner_params_->max_steer, BoundType::UPPER);
        double stl_lo_constr = get_bound_constr(u_k[1], -planner_params_->max_steer, BoundType::LOWER);
        double velo_up_constr = get_bound_constr(x_k[3], planner_params_->max_v, BoundType::UPPER);
        double velo_lo_constr = get_bound_constr(x_k[3], planner_params_->min_v, BoundType::LOWER);

        double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) -
                        (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = (d_sign < 0.0 ? -1.0 : 1.0) *
                       hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
        double pos_up_constr =
            get_bound_constr(cur_d, upper_bound_[0] - planner_params_->vehicle_width / 2.0, BoundType::UPPER);
        double pos_lo_constr =
            get_bound_constr(cur_d, lower_bound_[0] + planner_params_->vehicle_width / 2.0, BoundType::LOWER);

        double J_barrier_k = 0.0;
        if (solve_type_ == SolveType::BARRIER) {
            J_barrier_k = exp_barrier(acc_up_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(acc_lo_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(stl_up_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(stl_lo_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(velo_up_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(velo_lo_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(pos_up_constr, barrier_q1_, barrier_q2_) +
                          exp_barrier(pos_lo_constr, barrier_q1_, barrier_q2_);
        } else if (solve_type_ == SolveType::ALM) {
            J_barrier_k =
                augmented_lagrangian_item(acc_up_constr, alm_rho_, alm_mu_must_(k - 1, 0)) +
                augmented_lagrangian_item(acc_lo_constr, alm_rho_, alm_mu_must_(k - 1, 1)) +
                augmented_lagrangian_item(stl_up_constr, alm_rho_, alm_mu_must_(k - 1, 2)) +
                augmented_lagrangian_item(stl_lo_constr, alm_rho_, alm_mu_must_(k - 1, 3)) +
                augmented_lagrangian_item(velo_up_constr, alm_rho_, alm_mu_must_(k - 1, 4)) +
                augmented_lagrangian_item(velo_lo_constr, alm_rho_, alm_mu_must_(k - 1, 5)) +
                augmented_lagrangian_item(pos_up_constr, alm_rho_, alm_mu_must_(k - 1, 6)) +
                augmented_lagrangian_item(pos_lo_constr, alm_rho_, alm_mu_must_(k - 1, 7));
        }

        for (size_t j = 0; j < num_obstacles; ++j) {
            const general::Obstacle& obs_j_pred_k = obs_preds_[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            if (solve_type_ == SolveType::BARRIER) {
                J_barrier_k += exp_barrier(obs_j_constr[0], obstacle_barrier_q1_,
                                           obstacle_barrier_q2_);
                J_barrier_k += exp_barrier(obs_j_constr[1], obstacle_barrier_q1_,
                                           obstacle_barrier_q2_);
            } else if (solve_type_ == SolveType::ALM) {
                J_barrier_k += augmented_lagrangian_item(obs_j_constr[0], obstacle_alm_rho_,
                                                         alm_mu_obstacles_(k - 1, 2 * j));
                J_barrier_k += augmented_lagrangian_item(obs_j_constr[1], obstacle_alm_rho_,
                                                         alm_mu_obstacles_(k - 1, 2 * j + 1));
            }
        }
        J_barrier += J_barrier_k;
    }

    return J_prime + J_barrier;
}



void cilqr_eigen::update(int iter, const Eigen::MatrixXd& x_and_u) {
    const int N = planner_params_->ilqr_params->N;
    Eigen::MatrixXd x = x_and_u.block(0, 0, N + 1, n_x_);
    Eigen::MatrixXd u = x_and_u.block(0, n_x_, N, n_u_);

    if (solve_type_ == SolveType::ALM) {
        if (alm_mu_must_.rows() != N || alm_mu_must_.cols() != num_c_) {
            alm_mu_must_ = Eigen::MatrixXd::Zero(N, num_c_);
        }
        if (alm_mu_obstacles_.rows() != N ||
            alm_mu_obstacles_.cols() != static_cast<int>(2 * obs_preds_.size())) {
            alm_mu_obstacles_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
        }
    }

    Eigen::MatrixX3d ref_exact_points = Eigen::MatrixX3d::Zero(N + 1, 3);
    for (int k = 0; k < N + 1; ++k) {
        if (frenet_frame_) {
            auto matched = frenet_frame_->get_matched_point(x(k, 0), x(k, 1), x(k, 2));
            ref_exact_points(k, 0) = matched.x;
            ref_exact_points(k, 1) = matched.y;
            ref_exact_points(k, 2) = matched.heading;
        } else {
            ref_exact_points(k, 0) = x(k, 0);
            ref_exact_points(k, 1) = x(k, 1);
            ref_exact_points(k, 2) = x(k, 2);
        }
    }

    if (solve_type_ == SolveType::ALM) {
        for (int k = 1; k < N + 1; ++k) {
            Eigen::Vector2d u_k = u.row(k - 1).transpose();
            Eigen::Vector4d x_k = x.row(k).transpose();
            Eigen::Vector3d ref_x_k = ref_exact_points.row(k).transpose();

            double acc_up_constr =
                get_bound_constr(u_k[0], planner_params_->max_a, BoundType::UPPER);
            double acc_lo_constr =
                get_bound_constr(u_k[0], planner_params_->min_a, BoundType::LOWER);
            double stl_up_constr =
                get_bound_constr(u_k[1], planner_params_->max_steer, BoundType::UPPER);
            double stl_lo_constr =
                get_bound_constr(u_k[1], -planner_params_->max_steer, BoundType::LOWER);
            double velo_up_constr =
                get_bound_constr(x_k[3], planner_params_->max_v, BoundType::UPPER);
            double velo_lo_constr =
                get_bound_constr(x_k[3], planner_params_->min_v, BoundType::LOWER);

            double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) -
                            (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
            double cur_d = (d_sign < 0.0 ? -1.0 : 1.0) *
                           hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
            double pos_up_constr = get_bound_constr(
                cur_d, upper_bound_[0] - planner_params_->vehicle_width / 2.0, BoundType::UPPER);
            double pos_lo_constr = get_bound_constr(
                cur_d, lower_bound_[0] + planner_params_->vehicle_width / 2.0, BoundType::LOWER);

            alm_mu_must_(k - 1, 0) =
                std::min(std::max(alm_mu_must_(k - 1, 0) + alm_rho_ * acc_up_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 1) =
                std::min(std::max(alm_mu_must_(k - 1, 1) + alm_rho_ * acc_lo_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 2) =
                std::min(std::max(alm_mu_must_(k - 1, 2) + alm_rho_ * stl_up_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 3) =
                std::min(std::max(alm_mu_must_(k - 1, 3) + alm_rho_ * stl_lo_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 4) =
                std::min(std::max(alm_mu_must_(k - 1, 4) + alm_rho_ * velo_up_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 5) =
                std::min(std::max(alm_mu_must_(k - 1, 5) + alm_rho_ * velo_lo_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 6) =
                std::min(std::max(alm_mu_must_(k - 1, 6) + alm_rho_ * pos_up_constr, 0.0),
                         planner_params_->alm_mu_max);
            alm_mu_must_(k - 1, 7) =
                std::min(std::max(alm_mu_must_(k - 1, 7) + alm_rho_ * pos_lo_constr, 0.0),
                         planner_params_->alm_mu_max);

            for (size_t j = 0; j < obs_preds_.size(); ++j) {
                const general::Obstacle& obs_j_pred_k = obs_preds_[j][k];
                Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
                alm_mu_obstacles_(k - 1, 2 * j) = std::min(
                    std::max(alm_mu_obstacles_(k - 1, 2 * j) +
                                 obstacle_alm_rho_ * obs_j_constr[0],
                             0.0),
                    planner_params_->obstacle_alm_mu_max);
                alm_mu_obstacles_(k - 1, 2 * j + 1) = std::min(
                    std::max(alm_mu_obstacles_(k - 1, 2 * j + 1) +
                                 obstacle_alm_rho_ * obs_j_constr[1],
                             0.0),
                    planner_params_->obstacle_alm_mu_max);
            }
        }
    }

    if (iter > 0) {
        if (planner_params_->use_alm) {
            alm_rho_ = std::min(alm_rho_ * planner_params_->alm_beta, planner_params_->alm_rho_max);
            obstacle_alm_rho_ = std::min(obstacle_alm_rho_ * planner_params_->obstacle_alm_rho_beta,
                                         planner_params_->obstacle_alm_rho_max);
        } else {
            barrier_q1_ = std::min(barrier_q1_ * planner_params_->barrier_beta1,
                                   planner_params_->barrier_q1_max);
            barrier_q2_ = std::min(barrier_q2_ * planner_params_->barrier_beta2,
                                   planner_params_->barrier_q2_max);
            obstacle_barrier_q1_ =
                std::min(obstacle_barrier_q1_ * planner_params_->obstacle_barrier_beta1,
                         planner_params_->obstacle_barrier_q1_max);
            obstacle_barrier_q2_ =
                std::min(obstacle_barrier_q2_ * planner_params_->obstacle_barrier_beta2,
                         planner_params_->obstacle_barrier_q2_max);
        }
    }
}

double cilqr_eigen::exp_barrier(double c, double q1, double q2) {
    return q1 * exp(q2 * c);
}

double cilqr_eigen::augmented_lagrangian_item(double c, double rho, double mu) {
    return rho * std::pow(std::max(c + mu / rho, 0.0), 2) / 2.0;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> cilqr_eigen::exp_barrier_derivative_and_Hessian(
    double c, Eigen::MatrixXd c_dot, double q1, double q2) {
    double b = exp_barrier(c, q1, q2);
    Eigen::VectorXd b_dot = q2 * b * c_dot;
    Eigen::MatrixXd b_ddot = std::pow(q2, 2) * b * (c_dot * c_dot.transpose());
    return std::make_tuple(b_dot, b_ddot);
}

Eigen::Vector2d cilqr_eigen::get_ellipsoid_obstacle_scales(const general::Obstacle& obs,
                                                           double ego_pnt_radius) {
    const double a = 0.5 * obs.length + planner_params_->safety_distance * 6.0 + ego_pnt_radius;
    const double b = 0.5 * obs.width + planner_params_->safety_distance + ego_pnt_radius;
    return Eigen::Vector2d{a, b};
}

double cilqr_eigen::ellipsoid_safety_margin(const Eigen::Vector2d& pnt,
                                            const general::Obstacle& obs_state,
                                            const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center(obs_state.x, obs_state.y);
    double theta = obs_state.heading;
    Eigen::Vector2d diff = pnt - elp_center;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;
    double result = 1.0 - (std::pow(pnt_std[0], 2) / std::pow(ellipse_ab[0], 2) +
                           std::pow(pnt_std[1], 2) / std::pow(ellipse_ab[1], 2));
    return result;
}

Eigen::Vector2d cilqr_eigen::ellipsoid_safety_margin_derivatives(
    const Eigen::Vector2d& pnt, const general::Obstacle& obs_state, const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center(obs_state.x, obs_state.y);
    Eigen::Vector2d diff = pnt - elp_center;
    double theta = obs_state.heading;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;

    Eigen::Vector2d res_over_pnt_std = {-2 * pnt_std[0] / std::pow(ellipse_ab[0], 2),
                                        -2 * pnt_std[1] / std::pow(ellipse_ab[1], 2)};
    Eigen::Matrix2d pnt_std_over_diff = rotation_matrix.transpose();
    Eigen::Matrix2d diff_over_pnt = Eigen::Matrix2d::Identity();
    Eigen::Vector2d res_over_pnt = diff_over_pnt * pnt_std_over_diff * res_over_pnt_std;

    return res_over_pnt;
}

std::tuple<Eigen::Vector4d, Eigen::Vector4d> cilqr_eigen::get_obstacle_avoidance_constr_derivatives(
    const Eigen::Vector4d& ego_state, const general::Obstacle& obs_state) {
    auto [ego_front, ego_rear] =
        get_vehicle_front_and_rear_centers(ego_state, planner_params_->wheelbase, reference_point_);
    Eigen::Vector2d ellipse_ab =
        get_ellipsoid_obstacle_scales(obs_state, 0.5 * planner_params_->vehicle_width);

    Eigen::Vector2d front_safety_margin_over_ego_front =
        ellipsoid_safety_margin_derivatives(ego_front, obs_state, ellipse_ab);
    Eigen::Vector2d rear_safety_margin_over_ego_rear =
        ellipsoid_safety_margin_derivatives(ego_rear, obs_state, ellipse_ab);

    auto [ego_front_over_state, ego_rear_over_state] =
        get_vehicle_front_and_rear_center_derivatives(ego_state[2], planner_params_->wheelbase,
                                  reference_point_);

    Eigen::Vector4d front_safety_margin_over_state =
        ego_front_over_state * front_safety_margin_over_ego_front;
    Eigen::Vector4d rear_safety_margin_over_state =
        ego_rear_over_state * rear_safety_margin_over_ego_rear;

    return std::make_tuple(front_safety_margin_over_state, rear_safety_margin_over_state);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d> cilqr_eigen::get_vehicle_front_and_rear_centers(const Eigen::Vector4d& state, double wheelbase, ReferencePoint ref_point) {
    double yaw = state[2];
    Eigen::Vector2d front_pnt;
    Eigen::Vector2d rear_pnt;
    Eigen::Vector2d whba_vec = wheelbase * Eigen::Vector2d{cos(yaw), sin(yaw)};

    if (ref_point == ReferencePoint::RearCenter) {
        front_pnt = state.head(2) + whba_vec;
        rear_pnt = state.head(2);
    } else {
        front_pnt = state.head(2) + 0.5 * whba_vec;
        rear_pnt = state.head(2) - 0.5 * whba_vec;
    }

    return std::make_tuple(front_pnt, rear_pnt);
}

std::tuple<Eigen::Matrix<double, 4, 2>, Eigen::Matrix<double, 4, 2>> cilqr_eigen::get_vehicle_front_and_rear_center_derivatives(double yaw, double wheelbase, ReferencePoint ref_point) {
    double half_whba = 0.5 * wheelbase;
    Eigen::Matrix<double, 4, 2> front_pnt_over_state;
    Eigen::Matrix<double, 4, 2> rear_pnt_over_state;
    front_pnt_over_state << 1, 0, 0, 1, half_whba * (-sin(yaw)), half_whba * cos(yaw), 0, 0;
    rear_pnt_over_state << 1, 0, 0, 1, -half_whba * (-sin(yaw)), -half_whba * cos(yaw), 0, 0;

    if (ref_point == ReferencePoint::RearCenter) {
        front_pnt_over_state(2, 0) = wheelbase * (-sin(yaw));
        front_pnt_over_state(2, 1) = wheelbase * cos(yaw);
        rear_pnt_over_state(2, 0) = 0.0;
        rear_pnt_over_state(2, 1) = 0.0;
    }

    return std::make_tuple(front_pnt_over_state, rear_pnt_over_state);
}

void cilqr_eigen::get_total_cost_derivatives_and_Hessians(const Eigen::MatrixXd& x_and_u) {
    const int N = planner_params_->ilqr_params->N;
    Eigen::MatrixXd x = x_and_u.block(0, 0, N + 1, n_x_);
    Eigen::MatrixXd u = x_and_u.block(0, n_x_, N, n_u_);

    l_x_.setZero(N + 1, n_x_);
    l_u_.setZero(N, n_u_);
    l_xx_.setZero((N + 1) * n_x_, n_x_);
    l_uu_.setZero(N * n_u_, n_u_);
    l_xu_.setZero((N + 1) * n_x_, n_u_);
    l_ux_.setZero(N * n_u_, n_x_);

    Eigen::MatrixX3d ref_exact_points = Eigen::MatrixX3d::Zero(N + 1, 3);
    for (int k = 0; k < N + 1; ++k) {
        if (frenet_frame_) {
            auto matched = frenet_frame_->get_matched_point(x(k, 0), x(k, 1), x(k, 2));
            ref_exact_points(k, 0) = matched.x;
            ref_exact_points(k, 1) = matched.y;
            ref_exact_points(k, 2) = matched.heading;
        } else {
            ref_exact_points(k, 0) = x(k, 0);
            ref_exact_points(k, 1) = x(k, 1);
            ref_exact_points(k, 2) = x(k, 2);
        }
    }
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo_);
    Eigen::MatrixXd ref_states(N + 1, n_x_);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_exact_points.col(2),
        ref_velocitys;

    Eigen::MatrixXd l_u_prime = 2 * (u * R_);
    Eigen::MatrixXd l_uu_prime = (2 * R_).replicate(N, 1);
    Eigen::MatrixXd l_x_prime = 2 * (x - ref_states) * Q_;
    Eigen::MatrixXd l_xx_prime = (2 * Q_).replicate(N + 1, 1);

    Eigen::MatrixXd l_u_barrier = Eigen::MatrixXd::Zero(N, n_u_);
    Eigen::MatrixXd l_uu_barrier = Eigen::MatrixXd::Zero(N * n_u_, n_u_);
    Eigen::MatrixXd l_x_barrier = Eigen::MatrixXd::Zero(N + 1, n_x_);
    Eigen::MatrixXd l_xx_barrier = Eigen::MatrixXd::Zero((N + 1) * n_x_, n_x_);

    if (solve_type_ == SolveType::ALM) {
        alm_mu_must_next_ = Eigen::MatrixXd::Zero(N, num_c_);
        alm_mu_obstacles_next_ = Eigen::MatrixXd::Zero(N, 2 * obs_preds_.size());
    }

    const size_t num_obstacles = obs_preds_.size();
    for (int k = 1; k < N + 1; ++k) {
        Eigen::Vector2d u_k = u.row(k - 1).transpose();
        Eigen::Vector4d x_k = x.row(k).transpose();
        Eigen::Vector3d ref_x_k = ref_exact_points.row(k).transpose();

        double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) -
                        (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = (d_sign < 0.0 ? -1.0 : 1.0) *
                       hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);

        double acc_up_constr = get_bound_constr(u_k[0], planner_params_->max_a, BoundType::UPPER);
        double acc_lo_constr = get_bound_constr(u_k[0], planner_params_->min_a, BoundType::LOWER);
        double stl_up_constr = get_bound_constr(u_k[1], planner_params_->max_steer, BoundType::UPPER);
        double stl_lo_constr = get_bound_constr(u_k[1], -planner_params_->max_steer, BoundType::LOWER);
        double velo_up_constr = get_bound_constr(x_k[3], planner_params_->max_v, BoundType::UPPER);
        double velo_lo_constr = get_bound_constr(x_k[3], planner_params_->min_v, BoundType::LOWER);
        double pos_up_constr =
            get_bound_constr(cur_d, upper_bound_[0] - planner_params_->vehicle_width / 2.0, BoundType::UPPER);
        double pos_lo_constr =
            get_bound_constr(cur_d, lower_bound_[0] + planner_params_->vehicle_width / 2.0, BoundType::LOWER);

        Eigen::Vector2d acc_up_constr_over_u = {1.0, 0.0};
        Eigen::Vector2d acc_lo_constr_over_u = {-1.0, 0.0};
        Eigen::Vector2d stl_up_constr_over_u = {0.0, 1.0};
        Eigen::Vector2d stl_lo_constr_over_u = {0.0, -1.0};
        Eigen::Vector4d velo_up_constr_over_x = {0.0, 0.0, 0.0, 1.0};
        Eigen::Vector4d velo_lo_constr_over_x = {0.0, 0.0, 0.0, -1.0};
        Eigen::Vector4d pos_up_constr_over_x = {
            (x_k[0] - ref_x_k[0]) / hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]),
            (x_k[1] - ref_x_k[1]) / hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]), 0.0, 0.0};
        if (d_sign < 0.0) {
            pos_up_constr_over_x = -1.0 * pos_up_constr_over_x;
        }
        Eigen::Vector4d pos_lo_constr_over_x = -1.0 * pos_up_constr_over_x;

        if (solve_type_ == SolveType::BARRIER) {
            auto [acc_up_barrier_over_u, acc_up_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(acc_up_constr, acc_up_constr_over_u,
                                                   barrier_q1_, barrier_q2_);
            auto [acc_lo_barrier_over_u, acc_lo_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(acc_lo_constr, acc_lo_constr_over_u,
                                                   barrier_q1_, barrier_q2_);
            auto [stl_up_barrier_over_u, stl_up_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(stl_up_constr, stl_up_constr_over_u,
                                                   barrier_q1_, barrier_q2_);
            auto [stl_lo_barrier_over_u, stl_lo_barrier_over_uu] =
                exp_barrier_derivative_and_Hessian(stl_lo_constr, stl_lo_constr_over_u,
                                                   barrier_q1_, barrier_q2_);

            l_u_barrier.row(k - 1) =
                acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                stl_up_barrier_over_u.transpose() + stl_lo_barrier_over_u.transpose();
            l_uu_barrier.block(n_u_ * (k - 1), 0, n_u_, n_u_) =
                acc_up_barrier_over_uu + acc_lo_barrier_over_uu + stl_up_barrier_over_uu +
                stl_lo_barrier_over_uu;

            auto [velo_up_barrier_over_x, velo_up_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(velo_up_constr, velo_up_constr_over_x,
                                                   barrier_q1_, barrier_q2_);
            auto [velo_lo_barrier_over_x, velo_lo_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(velo_lo_constr, velo_lo_constr_over_x,
                                                   barrier_q1_, barrier_q2_);
            auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(pos_up_constr, pos_up_constr_over_x,
                                                   barrier_q1_, barrier_q2_);
            auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
                exp_barrier_derivative_and_Hessian(pos_lo_constr, pos_lo_constr_over_x,
                                                   barrier_q1_, barrier_q2_);

            l_x_barrier.row(k) = velo_up_barrier_over_x + velo_lo_barrier_over_x +
                                 pos_up_barrier_over_x + pos_lo_barrier_over_x;
            l_xx_barrier.block(n_x_ * k, 0, n_x_, n_x_) =
                velo_up_barrier_over_xx + velo_lo_barrier_over_xx + pos_up_barrier_over_xx +
                pos_lo_barrier_over_xx;
        } else if (solve_type_ == SolveType::ALM) {
            auto [acc_up_barrier_over_u, acc_up_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(acc_up_constr, acc_up_constr_over_u, alm_rho_,
                                                  alm_mu_must_(k - 1, 0));
            auto [acc_lo_barrier_over_u, acc_lo_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(acc_lo_constr, acc_lo_constr_over_u, alm_rho_,
                                                  alm_mu_must_(k - 1, 1));
            auto [stl_up_barrier_over_u, stl_up_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(stl_up_constr, stl_up_constr_over_u, alm_rho_,
                                                  alm_mu_must_(k - 1, 2));
            auto [stl_lo_barrier_over_u, stl_lo_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(stl_lo_constr, stl_lo_constr_over_u, alm_rho_,
                                                  alm_mu_must_(k - 1, 3));

            l_u_barrier.row(k - 1) =
                acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                stl_up_barrier_over_u.transpose() + stl_lo_barrier_over_u.transpose();
            l_uu_barrier.block(n_u_ * (k - 1), 0, n_u_, n_u_) =
                acc_up_barrier_over_uu + acc_lo_barrier_over_uu + stl_up_barrier_over_uu +
                stl_lo_barrier_over_uu;

            auto [velo_up_barrier_over_x, velo_up_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(velo_up_constr, velo_up_constr_over_x, alm_rho_,
                                                  alm_mu_must_(k - 1, 4));
            auto [velo_lo_barrier_over_x, velo_lo_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(velo_lo_constr, velo_lo_constr_over_x, alm_rho_,
                                                  alm_mu_must_(k - 1, 5));
            auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_up_constr, pos_up_constr_over_x, alm_rho_,
                                                  alm_mu_must_(k - 1, 6));
            auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_lo_constr, pos_lo_constr_over_x, alm_rho_,
                                                  alm_mu_must_(k - 1, 7));

            alm_mu_must_next_(k - 1, 0) = std::min(
                std::max(alm_mu_must_(k - 1, 0) + alm_rho_ * acc_up_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 1) = std::min(
                std::max(alm_mu_must_(k - 1, 1) + alm_rho_ * acc_lo_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 2) = std::min(
                std::max(alm_mu_must_(k - 1, 2) + alm_rho_ * stl_up_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 3) = std::min(
                std::max(alm_mu_must_(k - 1, 3) + alm_rho_ * stl_lo_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 4) = std::min(
                std::max(alm_mu_must_(k - 1, 4) + alm_rho_ * velo_up_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 5) = std::min(
                std::max(alm_mu_must_(k - 1, 5) + alm_rho_ * velo_lo_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 6) = std::min(
                std::max(alm_mu_must_(k - 1, 6) + alm_rho_ * pos_up_constr, 0.0),
                planner_params_->alm_mu_max);
            alm_mu_must_next_(k - 1, 7) = std::min(
                std::max(alm_mu_must_(k - 1, 7) + alm_rho_ * pos_lo_constr, 0.0),
                planner_params_->alm_mu_max);

            l_x_barrier.row(k) = velo_up_barrier_over_x + velo_lo_barrier_over_x +
                                 pos_up_barrier_over_x + pos_lo_barrier_over_x;
            l_xx_barrier.block(n_x_ * k, 0, n_x_, n_x_) =
                velo_up_barrier_over_xx + velo_lo_barrier_over_xx + pos_up_barrier_over_xx +
                pos_lo_barrier_over_xx;
        }

        for (size_t j = 0; j < num_obstacles; ++j) {
            const general::Obstacle& obs_j_pred_k = obs_preds_[j][k];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            auto [obs_j_front_constr_over_x, obs_j_rear_constr_over_x] =
                get_obstacle_avoidance_constr_derivatives(x_k, obs_j_pred_k);

            if (solve_type_ == SolveType::BARRIER) {
                auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                    exp_barrier_derivative_and_Hessian(obs_j_constr[0], obs_j_front_constr_over_x,
                                                       obstacle_barrier_q1_, obstacle_barrier_q2_);
                auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                    exp_barrier_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_constr_over_x,
                                                       obstacle_barrier_q1_, obstacle_barrier_q2_);

                l_x_barrier.row(k) += (obs_j_front_barrier_over_x + obs_j_rear_barrier_over_x);
                l_xx_barrier.block(n_x_ * k, 0, n_x_, n_x_) +=
                    (obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx);
            } else if (solve_type_ == SolveType::ALM) {
                auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[0], obs_j_front_constr_over_x,
                                                      obstacle_alm_rho_, alm_mu_obstacles_(k - 1, 2 * j));
                auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_constr_over_x,
                                                      obstacle_alm_rho_, alm_mu_obstacles_(k - 1, 2 * j + 1));

                alm_mu_obstacles_next_(k - 1, 2 * j) = std::min(
                    std::max(alm_mu_obstacles_(k - 1, 2 * j) +
                                 obstacle_alm_rho_ * obs_j_constr[0],
                             0.0),
                    planner_params_->obstacle_alm_mu_max);
                alm_mu_obstacles_next_(k - 1, 2 * j + 1) = std::min(
                    std::max(alm_mu_obstacles_(k - 1, 2 * j + 1) +
                                 obstacle_alm_rho_ * obs_j_constr[1],
                             0.0),
                    planner_params_->obstacle_alm_mu_max);

                l_x_barrier.row(k) += (obs_j_front_barrier_over_x + obs_j_rear_barrier_over_x);
                l_xx_barrier.block(n_x_ * k, 0, n_x_, n_x_) +=
                    (obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx);
            }
        }
    }

    l_u_ = l_u_prime + l_u_barrier;
    l_uu_ = l_uu_prime + l_uu_barrier;
    l_x_ = l_x_prime + l_x_barrier;
    l_xx_ = l_xx_prime + l_xx_barrier;
}

double cilqr_eigen::get_bound_constr(double variable, double bound, BoundType bound_type) {
    if (bound_type == BoundType::UPPER) {
        return variable - bound;
    } else {
        return bound - variable;
    }
}

Eigen::Vector2d cilqr_eigen::get_obstacle_avoidance_constr(const Eigen::Vector4d& ego_state,
                                                           const general::Obstacle& obs_state) {
    auto [ego_front, ego_rear] =
        get_vehicle_front_and_rear_centers(ego_state, planner_params_->wheelbase, reference_point_);
    Eigen::Vector2d ellipse_ab =
        get_ellipsoid_obstacle_scales(obs_state, 0.5 * planner_params_->vehicle_width);
    double front_safety_margin = ellipsoid_safety_margin(ego_front, obs_state, ellipse_ab);
    double rear_safety_margin = ellipsoid_safety_margin(ego_rear, obs_state, ellipse_ab);

    return Eigen::Vector2d{front_safety_margin, rear_safety_margin};
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> cilqr_eigen::lagrangian_derivative_and_Hessian(
    double c, Eigen::MatrixXd c_dot, double rho, double mu) {
    size_t dims = c_dot.rows();
    Eigen::VectorXd b_dot = Eigen::VectorXd::Zero(dims, 1);
    Eigen::MatrixXd b_ddot = Eigen::MatrixXd::Zero(dims, dims);

    if ((c + mu / rho) > 0.0) {
        b_dot = rho * (c + mu / rho) * c_dot;
        b_ddot = b_dot * c_dot.transpose();
    }

    return std::make_tuple(b_dot, b_ddot);
}

void cilqr_eigen::get_total_jacobian_hessian(const Eigen::MatrixXd& x_and_u, Eigen::MatrixXd& total_jacbian, Eigen::Tensor<double, 3, Eigen::ColMajor>& total_hessian) {
    const int N = planner_params_->ilqr_params->N;
    const int nz = n_x_ + n_u_;

    get_total_cost_derivatives_and_Hessians(x_and_u);

    total_jacbian = Eigen::MatrixXd::Zero(N + 1, nz);
    total_hessian = Eigen::Tensor<double, 3, Eigen::ColMajor>(N + 1, nz, nz);
    total_hessian.setZero();

    for (int k = 0; k < N; ++k) {
        total_jacbian.row(k) << l_x_.row(k), l_u_.row(k);

        for (int i = 0; i < n_x_; ++i) {
            for (int j = 0; j < n_x_; ++j) {
                total_hessian(k, i, j) = l_xx_(n_x_ * k + i, j);
            }
        }
        for (int i = 0; i < n_u_; ++i) {
            for (int j = 0; j < n_u_; ++j) {
                total_hessian(k, n_x_ + i, n_x_ + j) = l_uu_(n_u_ * k + i, j);
            }
        }
    }

    total_jacbian.row(N) << l_x_.row(N), Eigen::RowVectorXd::Zero(n_u_);
    for (int i = 0; i < n_x_; ++i) {
        for (int j = 0; j < n_x_; ++j) {
            total_hessian(N, i, j) = l_xx_(n_x_ * N + i, j);
        }
    }
}

}  // namespace planner
}  // namespace AD_algorithm
