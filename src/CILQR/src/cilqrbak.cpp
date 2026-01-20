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
    // Define symbolic variables
    casadi::SX state = casadi::SX::sym("state", nx_);
    casadi::SX control = casadi::SX::sym("control", nu_);

    // Dynamics
    casadi::SX next_state = cost_func_->dynamics(state, control);
    f_ = casadi::Function("f", {state, control}, {next_state});
    f_x_ = f_.jacobian(0, 0);
    f_u_ = f_.jacobian(1, 0);
    f_xx_ = f_.hessian(0, 0);
    f_uu_ = f_.hessian(1, 1);
    f_ux_ = f_.hessian(1, 0);

    // Cost functions
    casadi::SX l_stage = cost_func_->cost_function(state, control);
    l_ = casadi::Function("l", {state, control}, {l_stage});
    l_x_ = l_.jacobian(0, 0);
    l_u_ = l_.jacobian(1, 0);
    l_xx_ = l_.hessian(0, 0);
    l_uu_ = l_.hessian(1, 1);
    l_ux_ = l_.hessian(1, 0);

    // Terminal cost
    casadi::SX term_state = casadi::SX::sym("term_state", nx_);
    casadi::SX l_term = cost_func_->terminal_cost(term_state);
    terminal_l_ = casadi::Function("terminal_l", {term_state}, {l_term});
    terminal_l_x_ = terminal_l_.jacobian(0, 0);
    terminal_l_xx_ = terminal_l_.hessian(0, 0);

    // Constraints (for ALM). Infer constraint count by evaluating at zero state/control
    casadi::SX c_vec = cost_func_->constraints(state, control);
    auto g = casadi::Function("g", {state, control}, {c_vec});
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
        c_ = casadi::Function("c", {state, control}, {-log(-c_vec)});
        c_x_ = c_.jacobian(0, 0);
        c_u_ = c_.jacobian(1, 0);
        c_xx_ = c_.hessian(0, 0);
        c_uu_ = c_.hessian(1, 1);
        c_ux_ = c_.hessian(1, 0);
    }
    else
    {
        // 如果使用ALM，由于有两项，一项是外点罚函数，一项是拉格朗日乘子项，系数不同，所以要分开存储
        c_ = casadi::Function("c", {state, control}, {c_vec});
        c_x_ = c_.jacobian(0, 0);
        c_u_ = c_.jacobian(1, 0);
        c_xx_ = c_.hessian(0, 0);
        c_uu_ = c_.hessian(1, 1);
        c_ux_ = c_.hessian(1, 0);
        // 外点罚函数项
        auto alm_cost=max(0,c_vec)*max(0,c_vec)*0.5;// 由于松弛替代项处理比较复杂，这里用的是原始ALM的形式
        alm_c_ = casadi::Function("alm_c", {state, control}, {alm_cost});
        alm_c_x_ = alm_c_.jacobian(0, 0);
        alm_c_u_ = alm_c_.jacobian(1, 0);
        alm_c_xx_ = alm_c_.hessian(0, 0);
        alm_c_uu_ = alm_c_.hessian(1, 1);
        alm_c_ux_ = alm_c_.hessian(1, 0);
    }

    // Initialize matrices
    u_ = Eigen::MatrixXd::Zero(N_, nu_);
    x_ = Eigen::MatrixXd::Zero(N_+1, nx_);
    d_ = Eigen::MatrixXd::Zero(N_, nu_);
    K_ = Eigen::MatrixXd::Zero(N_, nu_*nx_);

    V_x_ = Eigen::VectorXd::Zero(nx_);
    V_xx_ = Eigen::MatrixXd::Zero(nx_, nx_);

    l_x_vals_ = Eigen::MatrixXd::Zero(N_, nx_);
    l_u_vals_ = Eigen::MatrixXd::Zero(N_, nu_);
    l_xx_vals_ = Eigen::MatrixXd::Zero(N_*nx_, nx_);
    l_uu_vals_ = Eigen::MatrixXd::Zero(N_*nu_, nu_);
    l_ux_vals_ = Eigen::MatrixXd::Zero(N_*nx_, nu_);

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

    if(!params_->use_Barrier) { 
        alm_c_vals_ = Eigen::MatrixXd::Zero(N_, num_c_);
        alm_c_x_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_);
        alm_c_u_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_);
        alm_c_xx_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nx_*nx_);
        alm_c_uu_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nu_);
        alm_c_ux_vals_ = Eigen::MatrixXd::Zero(N_*num_c_, nu_*nx_);

        alm_mu_ = Eigen::MatrixXd::Zero(N_, num_c_);
        alm_mu_next_ = Eigen::MatrixXd::Zero(N_, num_c_);
    }
}

void cilqr::solve() {
    // Assume x0 is set
    prev_cost_ = get_total_cost(x_, u_);
    for (int iter = 0; iter < params_->max_iter; ++iter) {
        iter_step();
        double new_J = get_total_cost(x_, u_);
        if (std::abs(new_J - prev_cost_) < params_->cost_tol) break;
        prev_cost_ = new_J;
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
        auto res = l_({x.row(k), u.row(k)});
        cost += static_cast<double>(res[0]);

        if (!params_->use_Barrier) {
            auto c_res = c_({x.row(k), u.row(k)});
            auto alm_c_barier_res = alm_c_({x.row(k), u.row(k)});
            casadi::DM c_dm = c_res[0];
            auto alm_c_barier = alm_c_barier_res[0];
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
        }else{
            auto c_res = c_({x.row(k), u.row(k)});
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
        // l_x
        auto l_x_res = l_x_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::VectorXd> l_x_vec(l_x_res[0].ptr(), nx_);
        l_x_vals_.row(k) = l_x_vec;

        // l_u
        auto l_u_res = l_u_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::VectorXd> l_u_vec(l_u_res[0].ptr(), nu_);
        l_u_vals_.row(k) = l_u_vec;

        // l_xx
        auto l_xx_res = l_xx_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> l_xx_mat(l_xx_res[0].ptr(), nx_, nx_);
        l_xx_vals_.block(k*nx_, 0, nx_, nx_) = l_xx_mat;

        // l_uu
        auto l_uu_res = l_uu_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> l_uu_mat(l_uu_res[0].ptr(), nu_, nu_);
        l_uu_vals_.block(k*nu_, 0, nu_, nu_) = l_uu_mat;

        // l_ux
        auto l_ux_res = l_ux_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> l_ux_mat(l_ux_res[0].ptr(), nx_, nu_);
        l_ux_vals_.block(k*nx_, 0, nx_, nu_) = l_ux_mat;

        // f_x
        auto f_x_res = f_x_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> f_x_mat(f_x_res[0].ptr(), nx_, nx_);
        f_x_vals_.block(k, 0, 1, nx_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_x_mat.data(), nx_*nx_);

        // f_u
        auto f_u_res = f_u_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> f_u_mat(f_u_res[0].ptr(), nx_, nu_);
        f_u_vals_.block(k, 0, 1, nx_*nu_) = Eigen::Map<Eigen::RowVectorXd>(f_u_mat.data(), nx_*nu_);

        if(params_->use_DDP){
            // f_xx
            auto f_xx_res = f_xx_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> f_xx_mat(f_xx_res[0].ptr(), nx_, nx_*nx_);
            f_xx_vals_.block(k, 0, 1, nx_*nx_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_xx_mat.data(), nx_*nx_*nx_);

            // f_uu
            auto f_uu_res = f_uu_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> f_uu_mat(f_uu_res[0].ptr(), nx_, nu_*nu_);
            f_uu_vals_.block(k, 0, 1, nx_*nu_*nu_) = Eigen::Map<Eigen::RowVectorXd>(f_uu_mat.data(), nx_*nu_*nu_);

            // f_ux
            auto f_ux_res = f_ux_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> f_ux_mat(f_ux_res[0].ptr(), nx_, nu_*nx_);
            f_ux_vals_.block(k, 0, 1, nx_*nu_*nx_) = Eigen::Map<Eigen::RowVectorXd>(f_ux_mat.data(), nx_*nu_*nx_);
        }

        // constraints
        // c_x
        auto c_x_res = c_x_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> c_x_mat(c_x_res[0].ptr(), num_c_, nx_);
        c_x_vals_.block(k*num_c_, 0, num_c_, nx_) = c_x_mat;

        // c_u
        auto c_u_res = c_u_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> c_u_mat(c_u_res[0].ptr(), num_c_, nu_);
        c_u_vals_.block(k*num_c_, 0, num_c_, nu_) = c_u_mat;

        // c_xx
        auto c_xx_res = c_xx_({x_.row(k), u_.row(k)});
        Eigen::Map<Eigen::MatrixXd> c_xx_mat(c_xx_res[0].ptr(), num_c_, nx_*nx_);
        c_xx_vals_.block(k*num_c_, 0, num_c_, nx_*nx_) = c_xx_mat;

        // c_uu
        auto c_uu_res = c_uu_({x_.row(k), u_.row(k)});  
        Eigen::Map<Eigen::MatrixXd> c_uu_mat(c_uu_res[0].ptr(), num_c_, nu_*nu_);
        c_uu_vals_.block(k*num_c_, 0, num_c_, nu_*nu_) = c_uu_mat;

        // c_ux
        auto c_ux_res = c_ux_({x_.row(k), u_.row(k)});  
        Eigen::Map<Eigen::MatrixXd> c_ux_mat(c_ux_res[0].ptr(), num_c_, nu_*nx_);
        c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_) = c_ux_mat;

        if(!params_->use_Barrier){
            // ALM部分
            // 外点罚项alm_c_x
            auto alm_c_x_res = alm_c_x_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> alm_c_x_mat(alm_c_x_res[0].ptr(), num_c_, nx_);
            alm_c_x_vals_.block(k*num_c_, 0, num_c_, nx_) = alm_c_x_mat;

            // 外点罚项alm_c_u
            auto alm_c_u_res = alm_c_u_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> alm_c_u_mat(alm_c_u_res[0].ptr(), num_c_, nu_);
            alm_c_u_vals_.block(k*num_c_, 0, num_c_, nu_) = alm_c_u_mat;

            // 外点罚项alm_c_xx
            auto alm_c_xx_res = alm_c_xx_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> alm_c_xx_mat(alm_c_xx_res[0].ptr(), num_c_, nx_*nx_);
            alm_c_xx_vals_.block(k*num_c_, 0, num_c_, nx_*nx_) = alm_c_xx_mat;

            // 外点罚项alm_c_uu
            auto alm_c_uu_res = alm_c_uu_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> alm_c_uu_mat(alm_c_uu_res[0].ptr(), num_c_, nu_*nu_);
            alm_c_uu_vals_.block(k*num_c_, 0, num_c_, nu_*nu_) = alm_c_uu_mat;

            // 外点罚项alm_c_ux
            auto alm_c_ux_res = alm_c_ux_({x_.row(k), u_.row(k)});
            Eigen::Map<Eigen::MatrixXd> alm_c_ux_mat(alm_c_ux_res[0].ptr(), num_c_, nu_*nx_);
            alm_c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_) = alm_c_ux_mat;
        }
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
Eigen::VectorXd cilqr::compose_l_derivatives(Eigen::VectorXd &l_x,Eigen::VectorXd &l_u,Eigen::VectorXd &l_xx,Eigen::VectorXd &l_uu,Eigen::VectorXd &l_ux,int k) const{
    if(!params_->use_Barrier){
        // 使用ALM
        l_x += c_x_vals_.row(k).transpose().cwiseProduct(alm_mu_.row(k).transpose())+ alm_rho_ * alm_c_x_vals_.row(k).transpose();
        l_u += c_u_vals_.row(k).transpose().cwiseProduct(alm_mu_.row(k).transpose())+ alm_rho_ * alm_c_u_vals_.row(k).transpose();
        l_xx += c_xx_vals_.block(k*num_c_, 0, num_c_, nx_).transpose() * alm_mu_.row(k).transpose() + alm_rho_ * alm_c_xx_vals_.block(k*num_c_, 0, num_c_, nx_).transpose();
        l_uu += c_uu_vals_.block(k*num_c_, 0, num_c_, nu_).transpose() * alm_mu_.row(k).transpose() + alm_rho_ * alm_c_uu_vals_.block(k*num_c_, 0, num_c_, nu_).transpose();
        l_ux += c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_).transpose() * alm_mu_.row(k).transpose() + alm_rho_ * alm_c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_).transpose();
    }
    else{
        // 使用障碍函数法
        l_x += barrier_rho_ * c_x_vals_.row(k).transpose();
        l_u += barrier_rho_ * c_u_vals_.row(k).transpose();
        l_xx += barrier_rho_ * c_xx_vals_.block(k*num_c_, 0, num_c_, nx_).transpose();
        l_uu += barrier_rho_ * c_uu_vals_.block(k*num_c_, 0, num_c_, nu_).transpose();
        l_ux += barrier_rho_ * c_ux_vals_.block(k*num_c_, 0, num_c_, nu_*nx_).transpose();
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

            Eigen::VectorXd l_x = l_x_vals_.row(i);
            Eigen::VectorXd l_u = l_u_vals_.row(i);
            Eigen::MatrixXd l_xx = l_xx_vals_.block(i*nx_, 0, nx_, nx_);
            Eigen::MatrixXd l_uu = l_uu_vals_.block(i*nu_, 0, nu_, nu_);
            Eigen::MatrixXd l_ux = l_ux_vals_.block(i*nu_, 0, nu_, nx_);
            // 组合代价函数项，使用障碍函数法或者ALM法
            compose_l_derivatives(l_x,l_u,l_xx,l_uu,l_ux,i);

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

            d_.row(i) =Q_uu_reg.ldlt().solve(-Q_u);bang
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

        auto next_x_res = f_({new_x.row(i), new_u.row(i)});
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