#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace AD_algorithm {
namespace general {

cilqr::cilqr(std::shared_ptr<cost_base> cost_func) : cost_func_(cost_func), params_(cost_func_->get_params()) {
    logger_ = std::make_shared<AD_algorithm::general::Logger>("CILQR");
    logger_->info("CILQR solver initializing...");
    N_ = params_->N;
    nx_ = params_->nx;
    nu_ = params_->nu;
    use_barrier_ = params_->use_Barrier;
    alm_rho_ = params_->alm_rho_init;
    alm_rho_init_ = params_->alm_rho_init;
    alm_gamma_ = params_->alm_gamma;
    max_rho_ = params_->max_rho;
    max_mu_ = params_->max_mu;
    barrier_beta_=params_->barrier_beta;
    barrier_rho_=params_->barrier_rho_init;
    alpha_ls_init_ = params_->alpha_ls_init;
    beta_ls_ = params_->beta_ls;
    init();
}

cilqr::~cilqr() {}

cilqr_params cilqr::get_parameters() const {
    return *params_;
}

void cilqr::set_params(const std::shared_ptr<cilqr_params>& params) {
    params_ = params;
    // 重新初始化以应用新参数
    N_ = params_->N;
    nx_ = params_->nx;
    nu_ = params_->nu;
    use_barrier_ = params_->use_Barrier;
    alm_rho_ = params_->alm_rho_init;
    alm_rho_init_ = params_->alm_rho_init;
    alm_gamma_ = params_->alm_gamma;
    max_rho_ = params_->max_rho;
    max_mu_ = params_->max_mu;
    barrier_beta_=params_->barrier_beta;
    barrier_rho_=params_->barrier_rho_init;
    alpha_ls_init_ = params_->alpha_ls_init;
    beta_ls_ = params_->beta_ls;
    init();
}

void cilqr::init() {
    // Combined [x; u] form
    casadi::SX z = casadi::SX::sym("z", nx_ + nu_);
    casadi::SX state = z(casadi::Slice(0, nx_));
    casadi::SX control = z(casadi::Slice(nx_, nx_ + nu_));
    casadi::SX k=casadi::SX::sym("k",1); // time index for constraints
    // Dynamics
    casadi::SX next_state = cost_func_->dynamics(state, control);
    f_ = casadi::Function("f", std::vector<casadi::SX>{z, k}, std::vector<casadi::SX>{next_state});
    
    casadi::SX f_jac_sx = casadi::SX::jacobian(next_state, z);
    f_jacobian_ = casadi::Function("f_jac", std::vector<casadi::SX>{z}, std::vector<casadi::SX>{f_jac_sx});
    
    // Partial derivative for DDP: d(v^T * f)/dz = v^T * df/dz. Hessian = d(df/dz^T * v)/dz
    // Equivalent to SX::hessian(dot(next_state, v), z)
    casadi::SX v_x = casadi::SX::sym("v_x", nx_);
    casadi::SX f_weighted = casadi::SX::dot(v_x, next_state);
    casadi::SX f_hess_sx = casadi::SX::hessian(f_weighted, z);
    f_hessian_ = casadi::Function("f_hess", std::vector<casadi::SX>{z, v_x}, std::vector<casadi::SX>{f_hess_sx});
    
    // Cost functions
    casadi::SX l_stage = cost_func_->cost_function(state, control,k);
    l_ = casadi::Function("l", std::vector<casadi::SX>{z,k}, std::vector<casadi::SX>{l_stage});
    
    casadi::SX l_jac_sx = casadi::SX::jacobian(l_stage, z);
    l_jacobian_ = casadi::Function("l_jac", std::vector<casadi::SX>{z,k}, std::vector<casadi::SX>{l_jac_sx});
    
    casadi::SX l_hess_sx = casadi::SX::hessian(l_stage, z);
    l_hessian_ = casadi::Function("l_hess", std::vector<casadi::SX>{z,k}, std::vector<casadi::SX>{l_hess_sx});


    // Terminal cost
    casadi::SX term_state = casadi::SX::sym("term_state", nx_);
    casadi::SX l_term = cost_func_->terminal_cost(term_state);
    terminal_l_ = casadi::Function("terminal_l", std::vector<casadi::SX>{term_state}, std::vector<casadi::SX>{l_term});
    
    casadi::SX term_jac_sx = casadi::SX::jacobian(l_term, term_state);
    terminal_l_x_ = casadi::Function("term_l_x", std::vector<casadi::SX>{term_state}, std::vector<casadi::SX>{term_jac_sx});
    
    casadi::SX term_hess_sx = casadi::SX::hessian(l_term, term_state);
    terminal_l_xx_ = casadi::Function("term_l_xx", std::vector<casadi::SX>{term_state}, std::vector<casadi::SX>{term_hess_sx});

    // Constraints (for ALM). Infer constraint count
    casadi::SX c_vec = cost_func_->constraints(state, control,k);
    c_ = casadi::Function("c", std::vector<casadi::SX>{z,k}, std::vector<casadi::SX>{c_vec});
    
    try {
        casadi::DM z0_dm = casadi::DM::zeros(nx_ + nu_, 1);
        casadi::DM k0_dm = casadi::DM::zeros(1,1);
        auto c0 = c_(std::vector<casadi::DM>{z0_dm,k0_dm});
        if (c0.size() > 0) {
            casadi::DM c0dm = c0[0];
            if (c0dm.size2() == 1) {
                num_c_ = static_cast<int>(c0dm.size1());
            } else {
                num_c_ = static_cast<int>(c0dm.numel());
            }
        } else {
            num_c_ = 0;
        }
    } catch (...) {
        num_c_ = 0;
    }

    // Build constraint cost vector functions for barrier and ALM
    if (num_c_ > 0) {
        // 定义每个时间步的约束代价向量函数和累加函数
        casadi::SX rho_b = casadi::SX::sym("rho_b", 1);
        casadi::SX mu_vec = casadi::SX::sym("mu_vec", num_c_);
        casadi::SX rho_alm = casadi::SX::sym("rho_alm", 1);

        // 障碍函数的向量形式
        casadi::SX barrier_cost_vec = rho_b * (-casadi::SX::log(-c_vec));
        cost_vec_barrier_ = casadi::Function(
            "cost_vec_barrier", std::vector<casadi::SX>{z, k, rho_b},
            std::vector<casadi::SX>{barrier_cost_vec});
        
        // ALM的向量形式
        casadi::SX pos_c = casadi::SX::fmax(0.0, c_vec + mu_vec / rho_alm);
        casadi::SX alm_cost_vec = mu_vec * c_vec + rho_alm * 0.5 * casadi::SX::pow(pos_c, 2);
        cost_vec_alm_ = casadi::Function(
            "cost_vec_alm", std::vector<casadi::SX>{z, k, mu_vec, rho_alm},
            std::vector<casadi::SX>{alm_cost_vec});
        
        // 障碍函数的累加形式，输出是标量
        casadi::SX barrier_cost_sum = casadi::SX::sum1(barrier_cost_vec);
        cost_sum_barrier_ = casadi::Function(
            "cost_sum_barrier", std::vector<casadi::SX>{z, k, rho_b},
            std::vector<casadi::SX>{barrier_cost_sum});

        // Jacobian，是一个向量
        casadi::SX barrier_jac = casadi::SX::jacobian(barrier_cost_sum, z);
        cost_sum_barrier_jac_ = casadi::Function(
            "cost_sum_barrier_jac", std::vector<casadi::SX>{z, k, rho_b},
            std::vector<casadi::SX>{barrier_jac});

        // Hessian，是一个矩阵
        casadi::SX barrier_hess = casadi::SX::jacobian(barrier_jac, z);
        cost_sum_barrier_hess_ = casadi::Function(
            "cost_sum_barrier_hess", std::vector<casadi::SX>{z, k, rho_b},
            std::vector<casadi::SX>{barrier_hess});

        // ALM的累加形式
        casadi::SX alm_cost_sum = casadi::SX::sum1(alm_cost_vec);
        cost_sum_alm_ = casadi::Function(
            "cost_sum_alm", std::vector<casadi::SX>{z, k, mu_vec, rho_alm},
            std::vector<casadi::SX>{alm_cost_sum});
        // Jacobian，是一个向量
        casadi::SX alm_jac = casadi::SX::jacobian(alm_cost_sum, z);
        cost_sum_alm_jac_ = casadi::Function(
            "cost_sum_alm_jac", std::vector<casadi::SX>{z, k, mu_vec, rho_alm},
            std::vector<casadi::SX>{alm_jac});
        // Hessian，是一个矩阵
        casadi::SX alm_hess = casadi::SX::jacobian(alm_jac, z);
        cost_sum_alm_hess_ = casadi::Function(
            "cost_sum_alm_hess", std::vector<casadi::SX>{z, k, mu_vec, rho_alm},
            std::vector<casadi::SX>{alm_hess});
    }

    // Initialize matrices
    u_ = Eigen::MatrixXd::Zero(N_, nu_);
    x_ = Eigen::MatrixXd::Zero(N_+1, nx_);
    d_ = Eigen::MatrixXd::Zero(N_, nu_);
    K_ = Eigen::MatrixXd::Zero(N_, nu_*nx_);

    V_x_ = Eigen::MatrixXd::Zero(nx_, 1);
    V_xx_ = Eigen::MatrixXd::Zero(nx_, nx_);

    // per-block storages (kept for compatibility)
    l_grad_vals_ = Eigen::MatrixXd::Zero(N_, nx_ + nu_);
    l_hess_vals_ = Eigen::MatrixXd::Zero(N_, (nx_ + nu_) * (nx_ + nu_));

    f_jac_vals_ = Eigen::MatrixXd::Zero(N_, nx_ * (nx_ + nu_));
    f_hess_vals_ = Eigen::MatrixXd::Zero(N_, nx_ * (nx_ + nu_) * (nx_ + nu_));

    c_vals_ = Eigen::MatrixXd::Zero(N_, num_c_);
    c_jac_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_));
    c_hess_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_) * (nx_ + nu_));

    if(!params_->use_Barrier) { 
        alm_mu_ = Eigen::MatrixXd::Zero(N_, num_c_);
        alm_mu_next_ = Eigen::MatrixXd::Zero(N_, num_c_);
    }
}


void cilqr::get_init_traj() {
    // Initialize controls to zero and propagate states using the learned dynamics f_
    u_.setZero();
    // Ensure the first state x_.row(0) has been set by the caller via set_initial_state
    for (int i = 0; i < N_; ++i) {
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x_.row(i), u_.row(i);
        casadi::DM k_dm = casadi::DM(i); // time index for constraints
        auto next_res = f_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(z_row.data(), z_row.data() + z_row.size())), k_dm});
        Eigen::Map<Eigen::VectorXd> next_x_vec(next_res[0].ptr(), nx_);
        x_.row(i + 1) = next_x_vec.transpose();
    }
}

void cilqr::get_init_traj_increment() {
    // Shift previous control forward by one step and keep the last control as-is
    Eigen::MatrixXd new_u = Eigen::MatrixXd::Zero(N_, nu_);
    if (u_.rows() == N_ && u_.cols() == nu_) {
        if (N_ > 1) {
            new_u.block(0, 0, N_ - 1, nu_) = u_.block(1, 0, N_ - 1, nu_);
            new_u.row(N_ - 1) = u_.row(N_ - 1);
        } else {
            new_u.row(0) = u_.row(0);
        }
    }
    u_ = new_u;

    // Forward propagate states using f_
    for (int i = 0; i < N_; ++i) {
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x_.row(i), u_.row(i);
        casadi::DM k_dm = casadi::DM(i); // time index for constraints
        auto next_res = f_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(z_row.data(), z_row.data() + z_row.size())), k_dm});
        Eigen::Map<Eigen::VectorXd> next_x_vec(next_res[0].ptr(), nx_);
        x_.row(i + 1) = next_x_vec.transpose();
    }
}

double cilqr::get_total_cost(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) {
    double cost = 0.0;
    for (int k = 0; k < N_; ++k) {
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x.row(k), u.row(k);
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        casadi::DM k_dm = casadi::DM(k); // time index for constraints
        auto res = l_(std::vector<casadi::DM>{z_row_dm,k_dm});
        cost += static_cast<double>(res[0]);

        if (num_c_ > 0) {
            if (!params_->use_Barrier) {
                casadi::DM mu_dm(std::vector<double>(alm_mu_.row(k).data(), alm_mu_.row(k).data() + num_c_));
                casadi::DM rho_dm(alm_rho_);
                auto cost_sum_res =
                    cost_sum_alm_(std::vector<casadi::DM>{z_row_dm, k_dm, mu_dm, rho_dm});
                if (!cost_sum_res.empty()) {
                    casadi::DM cost_sum_dm = casadi::DM::densify(cost_sum_res[0]);
                    cost += static_cast<double>(cost_sum_dm(0));
                }
            } else {
                casadi::DM rho_dm(barrier_rho_);
                auto cost_sum_res =
                    cost_sum_barrier_(std::vector<casadi::DM>{z_row_dm, k_dm, rho_dm});
                if (!cost_sum_res.empty()) {
                    casadi::DM cost_sum_dm = casadi::DM::densify(cost_sum_res[0]);
                    cost += static_cast<double>(cost_sum_dm(0));
                }
            }
        }
    }
    Eigen::RowVectorXd x_N = x.row(N_);
    auto term_res = terminal_l_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(x_N.data(), x_N.data() + x_N.size()))});
    cost += static_cast<double>(term_res[0]);
    return cost;
}

void cilqr::get_cost_derivatives_and_Hessians() {
    for (int k = 0; k < N_; ++k) {
        // Use combined gradient/Hessian on z = [x; u], then split into blocks.
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x_.row(k), u_.row(k);
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        casadi::DM k_dm = casadi::DM(k); // time index for constraints

        // l gradient / Hessian (scalar stage cost)
        // CasADi calls require std::vector<double> -> DM conversion
        auto l_grad_res = l_jacobian_(std::vector<casadi::DM>{z_row_dm, k_dm});
        casadi::DM l_grad_dm = casadi::DM::densify(l_grad_res[0]);
        Eigen::Map<Eigen::VectorXd> l_grad_vec(l_grad_dm.ptr(), nx_ + nu_);
        l_grad_vals_.row(k) = l_grad_vec.transpose();

        auto l_hess_res = l_hessian_(std::vector<casadi::DM>{z_row_dm, k_dm});
        casadi::DM l_hess_dm = casadi::DM::densify(l_hess_res[0]);
        Eigen::Map<Eigen::MatrixXd> l_hess_mat(l_hess_dm.ptr(), nx_ + nu_, nx_ + nu_);
        Eigen::MatrixXd L = l_hess_mat;
        // blocks: L = [ L_xx  L_xu;
        //               L_ux  L_uu ] where L_xu is nx x nu and L_ux is nu x nx
        l_hess_vals_.block(k, 0, 1, (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(L.data(), (nx_ + nu_) * (nx_ + nu_));
        
        // f Jacobian (nx x (nx+nu)) split into f_x and f_u
        auto f_jac_res = f_jacobian_(std::vector<casadi::DM>{z_row_dm});
        casadi::DM f_jac_dm = casadi::DM::densify(f_jac_res[0]);
        Eigen::Map<Eigen::MatrixXd> f_jac_mat(f_jac_dm.ptr(), nx_, nx_ + nu_);
        f_jac_vals_.block(k, 0, 1, nx_ * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(f_jac_mat.data(), nx_ * (nx_ + nu_));

        // f Hessian computation (tensor contraction) is done in backward_pass if needed.
        // if(params_->use_DDP) {
        //     // 使用运动学模型的二阶导数信息
        //     auto f_hess_res = f_hessian_(std::vector<casadi::DM>{z_row_dm});
        //     casadi::DM f_hess_dm = casadi::DM::densify(f_hess_res[0]);
        //     Eigen::Map<Eigen::MatrixXd> f_hess_mat(f_hess_dm.ptr(), nx_, (nx_ + nu_) * (nx_ + nu_));
        //     f_hess_vals_.block(k, 0, 1, nx_ * (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(f_hess_mat.data(), nx_ * (nx_ + nu_) * (nx_ + nu_));
        // }

        if (num_c_ > 0) {
            Eigen::VectorXd l_grad_k = l_grad_vals_.row(k).transpose();
            Eigen::MatrixXd l_hess_k = Eigen::Map<Eigen::MatrixXd>(L.data(), nx_ + nu_, nx_ + nu_);

            if (params_->use_Barrier) {
                casadi::DM rho_dm(barrier_rho_);
                auto grad_res = cost_sum_barrier_jac_(std::vector<casadi::DM>{z_row_dm, k_dm, rho_dm});
                auto hess_res = cost_sum_barrier_hess_(std::vector<casadi::DM>{z_row_dm, k_dm, rho_dm});

                if (!grad_res.empty()) {
                    casadi::DM grad_dm = casadi::DM::densify(grad_res[0]);
                    Eigen::Map<Eigen::VectorXd> grad_vec(grad_dm.ptr(), nx_ + nu_);
                    l_grad_k += grad_vec;
                }

                if (!hess_res.empty()) {
                    casadi::DM hess_dm = casadi::DM::densify(hess_res[0]);
                    Eigen::Map<Eigen::MatrixXd> hess_mat(hess_dm.ptr(), nx_ + nu_, nx_ + nu_);
                    l_hess_k += hess_mat;
                }
            } else {
                casadi::DM mu_dm(std::vector<double>(alm_mu_.row(k).data(), alm_mu_.row(k).data() + num_c_));
                casadi::DM rho_dm(alm_rho_);
                auto grad_res = cost_sum_alm_jac_(std::vector<casadi::DM>{z_row_dm, k_dm, mu_dm, rho_dm});
                auto hess_res = cost_sum_alm_hess_(std::vector<casadi::DM>{z_row_dm, k_dm, mu_dm, rho_dm});

                if (!grad_res.empty()) {
                    casadi::DM grad_dm = casadi::DM::densify(grad_res[0]);
                    Eigen::Map<Eigen::VectorXd> grad_vec(grad_dm.ptr(), nx_ + nu_);
                    l_grad_k += grad_vec;
                }

                if (!hess_res.empty()) {
                    casadi::DM hess_dm = casadi::DM::densify(hess_res[0]);
                    Eigen::Map<Eigen::MatrixXd> hess_mat(hess_dm.ptr(), nx_ + nu_, nx_ + nu_);
                    l_hess_k += hess_mat;
                }

                // update constraint values for multiplier update
                if (!c_.is_null()) {
                    auto c_val_res = c_(std::vector<casadi::DM>{z_row_dm, k_dm});
                    if (!c_val_res.empty()) {
                        casadi::DM c_val_dm = casadi::DM::densify(c_val_res[0]);
                        for (int i = 0; i < std::min(num_c_, static_cast<int>(c_val_dm.numel())); ++i) {
                            c_vals_(k, i) = static_cast<double>(c_val_dm(i));
                        }
                    }
                }
            }

            l_grad_vals_.row(k) = l_grad_k.transpose();
            l_hess_vals_.block(k, 0, 1, (nx_ + nu_) * (nx_ + nu_)) =
                Eigen::Map<Eigen::RowVectorXd>(l_hess_k.data(), (nx_ + nu_) * (nx_ + nu_));
        }
    }

    // Terminal
    Eigen::RowVectorXd x_N = x_.row(N_);
    auto term_l_x_res = terminal_l_x_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(x_N.data(), x_N.data() + x_N.size()))});
    casadi::DM term_l_x_dm = casadi::DM::densify(term_l_x_res[0]);
    Eigen::Map<Eigen::VectorXd> term_l_x_vec(term_l_x_dm.ptr(), nx_);
    V_x_ = term_l_x_vec;

    auto term_l_xx_res = terminal_l_xx_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(x_N.data(), x_N.data() + x_N.size()))});
    casadi::DM term_l_xx_dm = casadi::DM::densify(term_l_xx_res[0]);
    Eigen::Map<Eigen::MatrixXd> term_l_xx_mat(term_l_xx_dm.ptr(), nx_, nx_);
    V_xx_ = term_l_xx_mat;
}

void cilqr::solve() {
    logger_->info("Starting solve()");
    // Assume x0 is set; initialize trajectory using dynamics f_
    get_init_traj();
    logger_->info("get_init_traj done");
    prev_cost_ = get_total_cost(x_, u_);
    logger_->info("Initial total cost: {}", prev_cost_);
    for (int iter = 0; iter < params_->max_iter; ++iter) {
        logger_->info("Iteration: {}", iter);
        if(!iter_step()) {
            logger_->warn("Iteration step failed at iteration {}", iter);
            break;
        }
        logger_->info("Iteration step {} done,cost {}", iter,curr_cost_);
        // 代价函数下降得很小就停止
        if (std::abs(curr_cost_ - prev_cost_) < params_->cost_tol) 
        {
            logger_->info("Cost change below tolerance, converged at iteration {}", iter);
            break;
        }
        // 控制增量和状态增量都很小就停止
        if (max_delta_u_ < params_->delta_u_tol && max_delta_x_ < params_->delta_x_tol) 
        {
            logger_->info("State and control changes below tolerance, converged at iteration {}", iter);
            break;
        }
        prev_cost_ = curr_cost_;
    }
}

bool cilqr::iter_step() {
    logger_->info("Getting cost derivatives and Hessians");
    get_cost_derivatives_and_Hessians();
    logger_->info("Starting backward_pass");
    double reg = params_->reg_init;
    bool success = false;
    while (true) {
        if (backward_pass(reg)) {
            logger_->info("Backward pass successful with reg {}", reg);
            if (forward_pass()) {
                logger_->info("Forward pass successful");
                success = true;
                break;
            }
        }
        if(reg >= params_->reg_max){
            logger_->warn("Regularization exceeded maximum limit during iter_step");
            success = false;
            break;
        }
        // Increase regularization and retry
        reg = std::min(reg * params_->reg_factor, params_->reg_max);
        logger_->info("Increasing regularization to {}", reg); 
    }
    if(success){
        // Update ALM parameters
        if (!params_->use_Barrier) {
            // c_vals_就是代价对乘子的导数，增大步长，原理不可行域
            alm_mu_next_ = (alm_mu_ + alm_rho_ * c_vals_).cwiseMax(0);
            alm_mu_ = alm_mu_next_;
            alm_rho_ = std::min(alm_gamma_ * alm_rho_, max_rho_);
        }
        else {
            barrier_rho_ *= barrier_beta_; // 每次减小障碍函数系数，增大惩罚力度
        }
    }
    return success;
}


bool cilqr::backward_pass(double reg_init) {
    double reg = reg_init;
    for(int retry =0; retry < params_->max_reg_iter; ++retry){
        bool success = false;
        // 清空初始值
        K_.setZero();
        d_.setZero();
        // 终端导数初始值
        auto V_x_k= V_x_;
        auto V_xx_k= V_xx_;
        for (int i = N_-1; i >= 0; --i) {
            // Retrieve Jacobian and reshape safely
            Eigen::VectorXd f_jac_flat = f_jac_vals_.row(i);
            Eigen::MatrixXd f_jaco = Eigen::Map<Eigen::MatrixXd>(f_jac_flat.data(), nx_, nx_ + nu_);
            
            // use combined gradient/hessian representation for l
            // Retrieve L Hessian and reshape safely
            Eigen::VectorXd l_hess_flat = l_hess_vals_.row(i);
            Eigen::MatrixXd l_hess = Eigen::Map<Eigen::MatrixXd>(l_hess_flat.data(), nx_ + nu_, nx_ + nu_);
            
            Eigen::VectorXd l_grad = l_grad_vals_.row(i).transpose();
        
            Eigen::VectorXd Q_jaco =l_grad+f_jaco.transpose()*V_x_k;
            Eigen::MatrixXd Q_hess = l_hess + f_jaco.transpose() * V_xx_k * f_jaco;
            // if(params_->use_DDP){
            //     // DDP second order term: \sum V_x * Hess(f)
            //     Eigen::RowVectorXd z_row(nx_ + nu_);
            //     z_row << x_.row(i), u_.row(i);
            //     casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
                
            //     // Weight vector V_x_k
            //     casadi::DM v_x_dm(std::vector<double>(V_x_k.data(), V_x_k.data() + V_x_k.size()));
                
            //     auto res = f_hessian_(std::vector<casadi::DM>{z_row_dm, v_x_dm});
            //     Eigen::Map<Eigen::MatrixXd> tensor_term(res[0].ptr(), nx_ + nu_, nx_ + nu_);
                
            //     Q_hess += tensor_term;
            // }

            // 分块
            Eigen::VectorXd Q_x = Q_jaco.segment(0, nx_);
            Eigen::VectorXd Q_u = Q_jaco.segment(nx_, nu_);
            Eigen::MatrixXd Q_xx = Q_hess.block(0, 0, nx_, nx_);
            Eigen::MatrixXd Q_uu = Q_hess.block(nx_, nx_, nu_, nu_);
            Eigen::MatrixXd Q_ux = Q_hess.block(nx_, 0, nu_, nx_);
            
            Eigen::MatrixXd Q_uu_reg = Q_uu + reg * Eigen::MatrixXd::Identity(nu_, nu_);
            auto Q_uu_det = Q_uu_reg.determinant();
            if(Q_uu_det <= 1e-6){
                // 如果矩阵不可逆，增加正则化,重启迭代
                reg *= 10;
                success = false;
                break;
            }

            d_.row(i) = Q_uu_reg.ldlt().solve(-Q_u).transpose();
            
            Eigen::MatrixXd K_mat = Q_uu_reg.ldlt().solve(-Q_ux);
            // Store K flattened
            Eigen::VectorXd K_flat = Eigen::Map<Eigen::VectorXd>(K_mat.data(), K_mat.size());
            K_.row(i) = K_flat.transpose();

            // Comput V_x, V_xx
            V_x_k = Q_x + Q_ux.transpose() * d_.row(i).transpose();
            V_xx_k = Q_xx + Q_ux.transpose() * K_mat;
            // 确保V_xx_k是对称的
            V_xx_k = 0.5 * (V_xx_k + V_xx_k.transpose());

            success = true;
        }
        if (success)
        {
            return true; // 成功完成反向传递
        }
    }
    return false;
}

bool cilqr::forward_pass() {
    double alpha = alpha_ls_init_; 
    bool success = false;
    while(true){
        success = linear_search(alpha);
        if(success){
            logger_->info("Line search success! ");
            break;
        }
        if(alpha < params_->min_alpha){
            logger_->warn("Line search failed in forward pass!alpha: {}", alpha);
            success = false; // 强制退出
            break;
        }
        alpha *= params_->beta_ls;
    }
    return success;
}

bool cilqr::linear_search(double alpha) {
    Eigen::MatrixXd new_x = Eigen::MatrixXd::Zero(N_+1, nx_);
    Eigen::MatrixXd new_u = Eigen::MatrixXd::Zero(N_, nu_);
    new_x.row(0) = x_.row(0);
    double max_delta_x = std::numeric_limits<double>::lowest();
    double max_delta_u = std::numeric_limits<double>::lowest();
    for (int i = 0; i < N_; ++i) {
        Eigen::VectorXd delta_x = (new_x.row(i) - x_.row(i)).transpose();
        // Unflatten K
        Eigen::VectorXd K_flat = K_.row(i);
        Eigen::MatrixXd K_mat = Eigen::Map<Eigen::MatrixXd>(K_flat.data(), nu_, nx_);
        auto du=+ alpha * d_.row(i) + (K_mat * delta_x).transpose();
        new_u.row(i) = u_.row(i) +du;
        // 记录最大变化量，作为收敛判断依据
        if(du.norm() > max_delta_u) {
            max_delta_u = du.norm();
        }
        if(delta_x.norm() > max_delta_x) {
            max_delta_x = delta_x.norm();
        }

        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << new_x.row(i), new_u.row(i);
        casadi::DM k_dm = casadi::DM(i); // time index for constraints
        auto next_x_res = f_(std::vector<casadi::DM>{casadi::DM(std::vector<double>(z_row.data(), z_row.data() + z_row.size())), k_dm});
        Eigen::Map<Eigen::VectorXd> next_x_vec(next_x_res[0].ptr(), nx_);
        new_x.row(i+1) = next_x_vec;
    }
    max_delta_u_=max_delta_u;
    max_delta_x_=max_delta_x;
    // 计算新轨迹的代价
    double new_cost = get_total_cost(new_x, new_u);
    double cost_decay = prev_cost_ - new_cost;
    if (prev_cost_ > new_cost) {
        x_ = new_x;
        u_ = new_u;
        curr_cost_ = new_cost;
        return true;
    } else if (alpha < 1e-3 && std::fabs(cost_decay) < params_->cost_tol) {
        // 如果步长很小且代价变化很小，认为已经收敛，接受更新
        logger_->info("Accepting small step due to convergence: alpha={}, cost_decay={}", alpha, cost_decay);
        x_ = new_x;
        u_ = new_u;
        curr_cost_ = new_cost;
        return true;
    } else {
        return false;
    }
}

} // namespace general
} // namespace AD_algorithm