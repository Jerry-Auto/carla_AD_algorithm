#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

namespace AD_algorithm {
namespace general {

cilqr::cilqr(std::shared_ptr<cost_base> cost_func) : cost_func_(cost_func), params_(std::make_shared<cilqr_params>()) {
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
    alpha_ls_init_ = params_->alpha_ls;
    beta_ls_ = params_->beta_ls;
    init();
}

cilqr::~cilqr() {}

void cilqr::set_parameters(const cilqr_params& params) {
    *params_ = params;
    // update internal dimensions and ALM/barrier flags
    N_ = params_.get()->N;
    nx_ = params_.get()->nx;
    nu_ = params_.get()->nu;
    use_barrier_ = params_.get()->use_Barrier;
    alm_rho_ = params_.get()->alm_rho_init;
    alm_rho_init_ = params_.get()->alm_rho_init;
    alm_gamma_ = params_.get()->alm_gamma;
    max_rho_ = params_.get()->max_rho;
    max_mu_ = params_.get()->max_mu;
    barrier_beta_=params_.get()->barrier_beta;
    barrier_rho_=params_.get()->barrier_rho_init;
    alpha_ls_init_ = params_.get()->alpha_ls;
    beta_ls_ = params_.get()->beta_ls;

    // Reinit functions/matrices if cost_func_ already set
    if (cost_func_ != nullptr) {
        init();
    }
    else{
        std::cerr<<"Warning: cost function is not set when setting parameters in cilqr!"<<std::endl;
    }
}

cilqr_params cilqr::get_parameters() const {
    return *params_;
}

void cilqr::init() {
    // Combined [x; u] form
    casadi::SX z = casadi::SX::sym("z", nx_ + nu_);
    casadi::SX state = z(casadi::Slice(0, nx_));
    casadi::SX control = z(casadi::Slice(nx_, nx_ + nu_));

    // Dynamics
    casadi::SX next_state = cost_func_->dynamics(state, control);
    f_ = casadi::Function("f", {z}, {next_state});
    f_jacobian_ = f_.jacobian(0, 0);
    f_hessian_ = f_.hessian(0, 0);
    // Cost functions
    // Keep the original split functions for compatibility, but also build a combined
    // stage-cost function that takes a single vector z = [x; u], so we can compute
    // gradient/Hessian once and then split into blocks later.
    casadi::SX l_stage = cost_func_->cost_function(state, control);
    l_ = casadi::Function("l", {z}, {l_stage});
    l_jacobian_ = l_.jacobian(0, 0);
    l_hessian_ = l_.hessian(0, 0);


    // Terminal cost
    casadi::SX term_state = casadi::SX::sym("term_state", nx_);
    casadi::SX l_term = cost_func_->terminal_cost(term_state);
    terminal_l_ = casadi::Function("terminal_l", {term_state}, {l_term});
    terminal_l_x_ = terminal_l_.jacobian(0, 0);
    terminal_l_xx_ = terminal_l_.hessian(0, 0);

    // Constraints (for ALM). Infer constraint count by evaluating at zero state/control
    casadi::SX c_vec = cost_func_->constraints(state, control);
    auto g = casadi::Function("g", {z}, {c_vec});
    // Infer number of scalar constraints by evaluating c at zeros (safe fallback if eval fails)
    try {
        casadi::DM x0_dm = casadi::DM::zeros(nx_, 1);
        casadi::DM u0_dm = casadi::DM::zeros(nu_, 1);
        auto c0 = g({x0_dm, u0_dm});
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
        // In case evaluation fails (e.g., undefined at zero), fall back to 0 and user can set manually
        num_c_ = 0;
    }

    if(params_->use_Barrier) {
        // 如果使用障碍函数，定义log barrier相关函数,障碍参数是变化的，在迭代过程中更新，这里是纯log项
        c_ = casadi::Function("c", {z}, {-log(-c_vec)});
        c_jacobian_ = c_.jacobian(0, 0);
        c_hessian_ = c_.hessian(0, 0);
    }
    else
    {
        // 如果使用ALM，由于有两项，一项是外点罚函数，一项是拉格朗日乘子项，系数不同，所以要分开存储
        c_ = casadi::Function("c", {z}, {c_vec});
        c_jacobian_ = c_.jacobian(0, 0);
        c_hessian_ = c_.hessian(0, 0);
        // 外点罚函数项
        auto alm_cost=max(0,c_vec)*max(0,c_vec)*0.5;// 由于松弛替代项处理比较复杂，这里用的是原始ALM的形式
        alm_c_ = casadi::Function("alm_c", {z}, {alm_cost});
        alm_c_jacobian_ = alm_c_.jacobian(0, 0);
        alm_c_hessian_ = alm_c_.hessian(0, 0);
    }

    // Initialize matrices
    u_ = Eigen::MatrixXd::Zero(N_, nu_);
    x_ = Eigen::MatrixXd::Zero(N_+1, nx_);
    d_ = Eigen::MatrixXd::Zero(N_, nu_);
    K_ = Eigen::MatrixXd::Zero(N_, nu_*nx_);

    V_x_ = Eigen::VectorXd::Zero(nx_);
    V_xx_ = Eigen::MatrixXd::Zero(nx_, nx_);

    // per-block storages (kept for compatibility)
    l_x_vals_ = Eigen::MatrixXd::Zero(N_, nx_);
    l_u_vals_ = Eigen::MatrixXd::Zero(N_, nu_);
    l_xx_vals_ = Eigen::MatrixXd::Zero(N_*nx_, nx_);
    l_uu_vals_ = Eigen::MatrixXd::Zero(N_*nu_, nu_);
    l_ux_vals_ = Eigen::MatrixXd::Zero(N_*nx_, nu_);

    // combined storages for z=[x;u]
    l_grad_vals_ = Eigen::MatrixXd::Zero(N_, nx_ + nu_);
    l_hess_vals_ = Eigen::MatrixXd::Zero(N_, (nx_ + nu_)*(nx_ + nu_));

    f_x_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nx_);
    f_u_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_);
    f_xx_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nx_*nx_);
    f_uu_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_*nu_);
    f_ux_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_*nx_);

    f_jac_vals_ = Eigen::MatrixXd::Zero(N_, nx_*(nx_ + nu_));
    f_hess_vals_ = Eigen::MatrixXd::Zero(N_, nx_ * (nx_ + nu_) * (nx_ + nu_));

    f_x_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nx_);
    f_u_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_);
    f_xx_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nx_*nx_);
    f_uu_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_*nu_);
    f_ux_vals_ = Eigen::MatrixXd::Zero(N_, nx_*nu_*nx_);

    c_vals_ = Eigen::MatrixXd::Zero(N_, num_c_);
    c_x_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_);
    c_u_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_);
    c_xx_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_*nx_);
    c_uu_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nu_);
    c_ux_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nx_);

    // combined constraint jac/hess storages
    c_jac_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_));
    c_hess_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_) * (nx_ + nu_));

    if(!params_->use_Barrier) { 
        alm_c_vals_ = Eigen::MatrixXd::Zero(N_, num_c_);
        alm_c_x_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_);
        alm_c_u_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_);
        alm_c_xx_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_*nx_);
        alm_c_uu_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nu_);
        alm_c_ux_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nx_);

        // combined alm_c storages
        alm_c_jac_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_));
        alm_c_hess_vals_ = Eigen::MatrixXd::Zero(N_, num_c_ * (nx_ + nu_) * (nx_ + nu_));

        alm_mu_ = Eigen::MatrixXd::Zero(N_, num_c_);
        alm_mu_next_ = Eigen::MatrixXd::Zero(N_, num_c_);
    }
}

void cilqr::solve() {
    // Assume x0 is set; initialize trajectory using dynamics f_
    get_init_traj();
    prev_cost_ = get_total_cost(x_, u_);
    for (int iter = 0; iter < params_->max_iter; ++iter) {
        iter_step();
        double new_J = get_total_cost(x_, u_);
        if (std::abs(new_J - prev_cost_) < params_->cost_tol) break;
        prev_cost_ = new_J;
    }
}

void cilqr::get_init_traj() {
    // Initialize controls to zero and propagate states using the learned dynamics f_
    u_.setZero();
    // Ensure the first state x_.row(0) has been set by the caller via set_initial_state
    for (int i = 0; i < N_; ++i) {
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x_.row(i), u_.row(i);
        auto next_res = f_({z_row});
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
        auto next_res = f_({z_row});
        Eigen::Map<Eigen::VectorXd> next_x_vec(next_res[0].ptr(), nx_);
        x_.row(i + 1) = next_x_vec.transpose();
    }
}

void cilqr::iter_step() {
    get_cost_derivatives_and_Hessians();
    backward_pass();
    forward_pass();
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

double cilqr::get_total_cost(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) {
    double cost = 0.0;
    for (int k = 0; k < N_; ++k) {
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x.row(k), u.row(k);
        auto res = l_({z_row});
        cost += static_cast<double>(res[0]);

        if (!params_->use_Barrier) {
            auto c_res = c_({z_row});
            auto alm_c_barier_res = alm_c_({z_row});
            casadi::DM c_dm = c_res[0];
            for (int i = 0; i < num_c_; ++i) {
                double c_val = static_cast<double>(c_dm(i));
                double alm_c_barier = static_cast<double>(alm_c_barier_res[i]);
                if (c_val > 0) {
                    // 这一项是拉格朗日乘子项
                    cost +=  alm_mu_(k, i) * c_val;
                }
                if(alm_c_barier > 0){
                    // 这一项是外点罚函数项，alm_rho_是每次迭代的通用系数
                    cost += alm_rho_ * alm_c_barier;
                }
            }
        } else {
            auto c_res = c_({z_row});
            casadi::DM c_dm = c_res[0];
            for (int i = 0; i < num_c_; ++i) {
                double c_val = static_cast<double>(c_dm(i));
                if (c_val > 0) {
                    // 这一项是障碍函数法系数
                    cost += barrier_rho_ * c_val;
                }
            }
        }
    }
    auto term_res = terminal_l_({x.row(N_)});
    cost += static_cast<double>(term_res[0]);
    return cost;
}

void cilqr::get_cost_derivatives_and_Hessians() {
    for (int k = 0; k < N_; ++k) {
        // Use combined gradient/Hessian on z = [x; u], then split into blocks.
        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << x_.row(k), u_.row(k);

        // l gradient / Hessian (scalar stage cost)
        auto l_grad_res = l_jacobian_({z_row});
        Eigen::Map<Eigen::VectorXd> l_grad_vec(l_grad_res[0].ptr(), nx_ + nu_);
        l_x_vals_.row(k) = l_grad_vec.head(nx_);
        l_u_vals_.row(k) = l_grad_vec.tail(nu_);
        // store combined grad
        l_grad_vals_.row(k) = lz_grad_vec.transpose();

        auto l_hess_res = l_hessian_({z_row});
        Eigen::Map<Eigen::MatrixXd> l_hess_mat(l_hess_res[0].ptr(), nx_ + nu_, nx_ + nu_);
        Eigen::MatrixXd L = l_hess_mat;
        // blocks: L = [ L_xx  L_xu;
        //               L_ux  L_uu ] where L_xu is nx x nu and L_ux is nu x nx
        l_xx_vals_.block(k*nx_, 0, nx_, nx_) = L.block(0, 0, nx_, nx_);
        l_uu_vals_.block(k*nu_, 0, nu_, nu_) = L.block(nx_, nx_, nu_, nu_);
        l_ux_vals_.block(k*nx_, 0, nx_, nu_) = L.block(0, nx_, nx_, nu_);
        // store combined Hessian flattened row
        l_hess_vals_.block(k, 0, 1, (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(L.data(), (nx_ + nu_) * (nx_ + nu_));
        // f Jacobian (nx x (nx+nu)) split into f_x and f_u
        auto f_jac_res = f_jacobian_({z_row});
        Eigen::Map<Eigen::MatrixXd> f_jac_mat(f_jac_res[0].ptr(), nx_, nx_ + nu_);
        Eigen::MatrixXd f_x_mat = f_jac_mat.block(0, 0, nx_, nx_);
        Eigen::MatrixXd f_u_mat = f_jac_mat.block(0, nx_, nx_, nu_);
        f_x_vals_.block(k, 0, 1, nx_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_x_mat.data(), nx_*nx_);
        f_u_vals_.block(k, 0, 1, nx_*nu_) = Eigen::Map<Eigen::RowVectorXd>(f_u_mat.data(), nx_*nu_);
        // store combined jacobian flattened row
        f_jac_vals_.block(k, 0, 1, nx_ * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(f_jac_mat.data(), nx_ * (nx_ + nu_));

        if(params_->use_DDP){
            // f Hessian: returns nx rows and (nx+nu)*(nx+nu) columns
            auto f_hess_res = f_hessian_({z_row});
            Eigen::Map<Eigen::MatrixXd> f_hess_mat(f_hess_res[0].ptr(), nx_, (nx_ + nu_)*(nx_ + nu_));
            // store flattened full f_hessian rows
            f_hess_vals_.block(k, 0, 1, nx_ * (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(f_hess_mat.data(), nx_ * (nx_ + nu_) * (nx_ + nu_));
            // Extract f_xx (nx x nx*nx), f_uu (nx x nu*nu), f_ux (nx x nu*nx)
            Eigen::MatrixXd f_xx_mat = Eigen::MatrixXd::Zero(nx_, nx_*nx_);
            Eigen::MatrixXd f_uu_mat = Eigen::MatrixXd::Zero(nx_, nu_*nu_);
            Eigen::MatrixXd f_ux_mat = Eigen::MatrixXd::Zero(nx_, nu_*nx_);
            int n = nx_ + nu_;
            // f_xx: i (0..nx-1), j (0..nx-1) => column = i*n + j
            for (int i = 0; i < nx_; ++i) {
                for (int j = 0; j < nx_; ++j) {
                    int col = i * n + j;
                    int dest = i * nx_ + j;
                    f_xx_mat.col(dest) = f_hess_mat.col(col);
                }
            }
            // f_uu: i in nx..n-1, j in nx..n-1
            for (int i = nx_; i < n; ++i) {
                for (int j = nx_; j < n; ++j) {
                    int col = i * n + j;
                    int dest = (i - nx_) * nu_ + (j - nx_);
                    f_uu_mat.col(dest) = f_hess_mat.col(col);
                }
            }
            // f_ux: i in 0..nx-1 (rows for u index selection)??? we want derivative w.r.t u then x -> (nu x nx) per output
            for (int i = nx_; i < n; ++i) {
                for (int j = 0; j < nx_; ++j) {
                    int col = i * n + j; // u index i, x index j
                    int dest = (i - nx_) * nx_ + j;
                    f_ux_mat.col(dest) = f_hess_mat.col(col);
                }
            }
            f_xx_vals_.block(k, 0, 1, nx_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_xx_mat.data(), nx_*nx_*1);
            f_uu_vals_.block(k, 0, 1, nx_*nu_*nu_) = Eigen::Map<Eigen::RowVectorXd>(f_uu_mat.data(), nx_*nu_*nu_);
            f_ux_vals_.block(k, 0, 1, nx_*nu_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_ux_mat.data(), nx_*nu_*nx_);
        }

        // constraints: jacobian over z then split to x/u blocks
        auto c_val_res = c_({z_row});
        Eigen::Map<Eigen::VectorXd> c_val_vec(c_val_res[0].ptr(), num_c_);
        c_vals_.row(k) = c_val_vec;

        auto c_jac_res = c_jacobian_({z_row});
        Eigen::Map<Eigen::MatrixXd> c_jac_mat(c_jac_res[0].ptr(), num_c_, nx_ + nu_);
        c_x_vals_.block(k*num_c_, 0, num_c_, nx_) = c_jac_mat.block(0, 0, num_c_, nx_);
        c_u_vals_.block(k*num_c_, 0, num_c_, nu_) = c_jac_mat.block(0, nx_, num_c_, nu_);
        // store combined jacobian flattened
        c_jac_vals_.block(k, 0, 1, num_c_ * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(c_jac_mat.data(), num_c_ * (nx_ + nu_));

        // constraints hessian: num_c_ x (n*n) -> extract x-x, u-u, u-x blocks
        auto c_hess_res = c_hessian_({z_row});
        Eigen::Map<Eigen::MatrixXd> c_hess_mat(c_hess_res[0].ptr(), num_c_, (nx_ + nu_)*(nx_ + nu_));
        c_hess_vals_.block(k, 0, 1, num_c_ * (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(c_hess_mat.data(), num_c_ * (nx_ + nu_) * (nx_ + nu_));
        // ALM external penalty values and derivatives
        if (!params_->use_Barrier) {
            auto alm_c_val_res = alm_c_({z_row});
            Eigen::Map<Eigen::VectorXd> alm_c_val_vec(alm_c_val_res[0].ptr(), num_c_);
            alm_c_vals_.row(k) = alm_c_val_vec;

            // alm_c jacobian and split to x/u
            auto alm_c_jac_res = alm_c_jacobian_({z_row});
            Eigen::Map<Eigen::MatrixXd> alm_c_jac_mat(alm_c_jac_res[0].ptr(), num_c_, nx_ + nu_);
            alm_c_x_vals_.block(k*num_c_, 0, num_c_, nx_) = alm_c_jac_mat.block(0, 0, num_c_, nx_);
            alm_c_u_vals_.block(k*num_c_, 0, num_c_, nu_) = alm_c_jac_mat.block(0, nx_, num_c_, nu_);
            // store combined alm_c jacobian flattened
            alm_c_jac_vals_.block(k, 0, 1, num_c_ * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(alm_c_jac_mat.data(), num_c_ * (nx_ + nu_));

            // alm_c hessian (num_c_ x (n*n)) -> extract blocks
            auto alm_c_hess_res = alm_c_hessian_({z_row});
            Eigen::Map<Eigen::MatrixXd> alm_c_hess_mat(alm_c_hess_res[0].ptr(), num_c_, (nx_ + nu_)*(nx_ + nu_));
            alm_c_hess_vals_.block(k, 0, 1, num_c_ * (nx_ + nu_) * (nx_ + nu_)) = Eigen::Map<Eigen::RowVectorXd>(alm_c_hess_mat.data(), num_c_ * (nx_ + nu_) * (nx_ + nu_));
            Eigen::MatrixXd alm_c_xx_mat = Eigen::MatrixXd::Zero(num_c_, nx_*nx_);
            Eigen::MatrixXd alm_c_uu_mat = Eigen::MatrixXd::Zero(num_c_, nu_*nu_);
            Eigen::MatrixXd alm_c_ux_mat = Eigen::MatrixXd::Zero(num_c_, nu_*nx_);
            int n = nx_ + nu_;
            for (int i = 0; i < nx_; ++i) {
                for (int j = 0; j < nx_; ++j) {
                    int col = i * n + j;
                    int dest = i * nx_ + j;
                    alm_c_xx_mat.col(dest) = alm_c_hess_mat.col(col);
                }
            }
            for (int i = nx_; i < n; ++i) {
                for (int j = nx_; j < n; ++j) {
                    int col = i * n + j;
                    int dest = (i - nx_) * nu_ + (j - nx_);
                    alm_c_uu_mat.col(dest) = alm_c_hess_mat.col(col);
                }
            }
            for (int i = nx_; i < n; ++i) {
                for (int j = 0; j < nx_; ++j) {
                    int col = i * n + j;
                    int dest = (i - nx_) * nx_ + j;
                    alm_c_ux_mat.col(dest) = alm_c_hess_mat.col(col);
                }
            }
            alm_c_xx_vals_.block(k*num_c_, 0, num_c_, nx_*nx_) = alm_c_xx_mat;
            alm_c_uu_vals_.block(k*num_c_, 0, num_c_, nu_*nu_) = alm_c_uu_mat;
            alm_c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_) = alm_c_ux_mat;
        }

        // constraints hessian: num_c_ x (n*n) -> extract x-x, u-u, u-x blocks
        auto c_hess_res = c_hessian_({z_row});
        Eigen::Map<Eigen::MatrixXd> c_hess_mat(c_hess_res[0].ptr(), num_c_, (nx_ + nu_)*(nx_ + nu_));
        Eigen::MatrixXd c_xx_mat = Eigen::MatrixXd::Zero(num_c_, nx_*nx_);
        Eigen::MatrixXd c_uu_mat = Eigen::MatrixXd::Zero(num_c_, nu_*nu_);
        Eigen::MatrixXd c_ux_mat = Eigen::MatrixXd::Zero(num_c_, nu_*nx_);
        int n = nx_ + nu_;
        // x-x
        for (int i = 0; i < nx_; ++i) {
            for (int j = 0; j < nx_; ++j) {
                int col = i * n + j;
                int dest = i * nx_ + j;
                c_xx_mat.col(dest) = c_hess_mat.col(col);
            }
        }
        // u-u
        for (int i = nx_; i < n; ++i) {
            for (int j = nx_; j < n; ++j) {
                int col = i * n + j;
                int dest = (i - nx_) * nu_ + (j - nx_);
                c_uu_mat.col(dest) = c_hess_mat.col(col);
            }
        }
        // u-x (rows indices u, columns x)
        for (int i = nx_; i < n; ++i) {
            for (int j = 0; j < nx_; ++j) {
                int col = i * n + j;
                int dest = (i - nx_) * nx_ + j;
                c_ux_mat.col(dest) = c_hess_mat.col(col);
            }
        }
        c_xx_vals_.block(k*num_c_, 0, num_c_, nx_*nx_) = c_xx_mat;
        c_uu_vals_.block(k*num_c_, 0, num_c_, nu_*nu_) = c_uu_mat;
        c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_) = c_ux_mat;



    }

    // Terminal
    auto term_l_x_res = terminal_l_x_({x_.row(N_)});
    Eigen::Map<Eigen::VectorXd> term_l_x_vec(term_l_x_res[0].ptr(), nx_);
    V_x_ = term_l_x_vec;

    auto term_l_xx_res = terminal_l_xx_({x_.row(N_)});
    Eigen::Map<Eigen::MatrixXd> term_l_xx_mat(term_l_xx_res[0].ptr(), nx_, nx_);
    V_xx_ = term_l_xx_mat;
}

// 组合代价函数项，使用障碍函数法或者ALM法
void cilqr::compose_l_derivatives(Eigen::VectorXd &l_grad, Eigen::MatrixXd &l_hess, int k) const{
    int n = nx_ + nu_;
    if (num_c_ <= 0) return;

    // Map c_jac and c_hess for this time-step
    Eigen::Map<const Eigen::MatrixXd> c_jac_mat(c_jac_vals_.row(k).data(), num_c_, n);
    Eigen::Map<const Eigen::MatrixXd> c_hess_mat(c_hess_vals_.row(k).data(), num_c_, n * n);

    if (!params_->use_Barrier) {
        // ALM: l_grad += c_jac^T * mu + rho * alm_c_jac^T
        Eigen::VectorXd mu_row = alm_mu_.row(k).transpose();
        l_grad += c_jac_mat.transpose() * mu_row;

        // alm_c contributions
        Eigen::Map<const Eigen::MatrixXd> alm_c_jac_mat(alm_c_jac_vals_.row(k).data(), num_c_, n);
        l_grad += alm_rho_ * alm_c_jac_mat.transpose() * Eigen::VectorXd::Ones(num_c_);

        // Hessian assembly: sum_i mu_i * H_c_i + rho * sum_i H_almc_i
        Eigen::MatrixXd H_add = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < num_c_; ++i) {
            Eigen::Map<const Eigen::MatrixXd> Hi(c_hess_mat.row(i).data(), n, n);
            H_add += mu_row(i) * Hi;
        }
        Eigen::Map<const Eigen::MatrixXd> alm_c_hess_mat(alm_c_hess_vals_.row(k).data(), num_c_, n * n);
        for (int i = 0; i < num_c_; ++i) {
            Eigen::Map<const Eigen::MatrixXd> Hi(alm_c_hess_mat.row(i).data(), n, n);
            H_add += alm_rho_ * Hi;
        }
        l_hess += H_add;
    } else {
        // Barrier: directly add barrier contributions (c_ is already the barrier form)
        Eigen::VectorXd ones = Eigen::VectorXd::Ones(num_c_);
        l_grad += barrier_rho_ * c_jac_mat.transpose() * ones;
        Eigen::MatrixXd H_add = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < num_c_; ++i) {
            Eigen::Map<const Eigen::MatrixXd> Hi(c_hess_mat.row(i).data(), n, n);
            H_add += barrier_rho_ * Hi;
        }
        l_hess += H_add;
    }
}

void cilqr::backward_pass() {
    double reg = params_->reg_init;
    for(int retry =0; retry < params_->max_reg_iter; ++retry){
        // 清空初始值
        K_.setZero();
        d_.setZero();
        V_x_.setZero();
        V_xx_.setZero();
        for (int i = N_-1; i >= 0; --i) {
            Eigen::MatrixXd f_x = Eigen::Map<Eigen::MatrixXd>(f_x_vals_.row(i).data(), nx_, nx_);
            Eigen::MatrixXd f_u = Eigen::Map<Eigen::MatrixXd>(f_u_vals_.row(i).data(), nx_, nu_);
            if(params_->use_DDP){
                Eigen::MatrixXd f_xx_ = Eigen::Map<Eigen::MatrixXd>(f_xx_vals_.row(i).data(), nx_, nx_*nx_);
                Eigen::MatrixXd f_uu_ = Eigen::Map<Eigen::MatrixXd>(f_uu_vals_.row(i).data(), nx_, nu_*nu_);
                Eigen::MatrixXd f_ux_ = Eigen::Map<Eigen::MatrixXd>(f_ux_vals_.row(i).data(), nx_, nu_*nx_);
            }

            // use combined gradient/hessian representation for l
            Eigen::VectorXd l_grad = l_grad_vals_.row(i).transpose();
            Eigen::MatrixXd l_hess = Eigen::Map<Eigen::MatrixXd>(l_hess_vals_.row(i).data(), nx_ + nu_, nx_ + nu_);
            // compose ALM / Barrier contributions into l_grad and l_hess
            compose_l_derivatives(l_grad, l_hess, i);
            // split into blocks expected by backward pass
            Eigen::VectorXd l_x = l_grad.head(nx_);
            Eigen::VectorXd l_u = l_grad.tail(nu_);
            Eigen::MatrixXd l_xx = l_hess.block(0, 0, nx_, nx_);
            Eigen::MatrixXd l_uu = l_hess.block(nx_, nx_, nu_, nu_);
            Eigen::MatrixXd l_ux = l_hess.block(0, nx_, nx_, nu_).transpose();

            Eigen::VectorXd Q_x = l_x + f_x.transpose() * V_x_;
            Eigen::VectorXd Q_u = l_u + f_u.transpose() * V_x_;
            Eigen::MatrixXd Q_xx = l_xx + f_x.transpose() * V_xx_ * f_x;
            Eigen::MatrixXd Q_uu = l_uu + f_u.transpose() * V_xx_ * f_u;
            Eigen::MatrixXd Q_ux = l_ux + f_u.transpose() * V_xx_ * f_x;
            if(params_->use_DDP){
                // DDP二阶项
                Q_xx+=Eigen::Map<Eigen::MatrixXd>(f_xx_.data(), nx_, nx_*nx_).transpose().reshaped(nx_, nx_) * V_x_;
                Q_uu+=Eigen::Map<Eigen::MatrixXd>(f_uu_.data(), nx_, nu_*nu_).transpose().reshaped(nu_, nu_) * V_x_;
                Q_ux+=Eigen::Map<Eigen::MatrixXd>(f_ux_.data(), nx_, nu_*nx_).transpose().reshaped(nu_, nx_) * V_x_;
            }

            
            Eigen::MatrixXd Q_uu_reg = Q_uu + reg * Eigen::MatrixXd::Identity(nu_, nu_);
            auto Q_uu_det = Q_uu_reg.determinant();
            if(Q_uu_det <= 1e-6){
                // 如果矩阵不可逆，增加正则化,重启迭代
                reg *= 10;
                break;
            }

            d_.row(i) = Q_uu_reg.ldlt().solve(-Q_u);
            K_.block(i, 0, nu_, nx_) = Q_uu_reg.ldlt().solve(-Q_ux);
            V_x_ = Q_x + Q_ux.transpose() * d_.row(i);
            V_xx_ = Q_xx + Q_ux.transpose() * K_.block(i, 0, nu_, nx_);
        }

        break; // 成功完成反向传递
    }
}

void cilqr::forward_pass() {
    double alpha = alpha_ls_init_; 
    double old_to_new_cost = get_total_cost(x_, u_);
    while(linear_search(alpha) == false){
        alpha *= beta_ls_;
        if(alpha < 1e-4){
            std::cerr<<"Warning: line search failed in cilqr forward pass!"<<std::endl;
            break;
        }
    }
}

bool cilqr::linear_search(double alpha) {
    Eigen::MatrixXd new_x = Eigen::MatrixXd::Zero(N_+1, nx_);
    Eigen::MatrixXd new_u = Eigen::MatrixXd::Zero(N_, nu_);
    new_x.row(0) = x_.row(0);
    for (int i = 0; i < N_; ++i) {
        Eigen::VectorXd delta_x = new_x.row(i) - x_.row(i);
        new_u.row(i) = u_.row(i) + alpha * d_.row(i) + K_.block(i, 0, nu_, nx_) * delta_x;

        Eigen::RowVectorXd z_row(nx_ + nu_);
        z_row << new_x.row(i), new_u.row(i);
        auto next_x_res = f_({z_row});
        Eigen::Map<Eigen::VectorXd> next_x_vec(next_x_res[0].ptr(), nx_);
        new_x.row(i+1) = next_x_vec;
    }
    // 计算新轨迹的代价
    double new_cost = get_total_cost(new_x, new_u);
    if (prev_cost_ > new_cost) {
        x_ = new_x;
        u_ = new_u;
        return true;
    } else {
        return false;
    }
}

} // namespace general
} // namespace AD_algorithm