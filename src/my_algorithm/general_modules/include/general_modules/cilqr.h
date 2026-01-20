#pragma once
#include <vector>
#include <tuple>
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace general {

struct cilqr_params {
    bool use_DDP = false;      // true for DDP, false for iLQR
    bool use_Barrier = false;  // true for barrier, false for ALM

    // 全局迭代参数
    int max_iter = 50;      // 最大迭代次数
    int N = 50;            // horizon length
    int nx = 4;             // state dimension
    int nu = 2;          // control dimension
    double dt=0.1;       // time step

    // 正则化参数
    double reg_max = 1e10;
    double reg_init = 1.0;
    double reg_factor = 10.0;
    int max_reg_iter = 10; // 最大正则化尝试次数

    // 收敛判定参数
    double delta_u_tol = 1e-4;
    double delta_x_tol = 1e-2;
    double cost_tol = 10;

    // ALM parameters
    double alm_rho = 1.0;
    double alm_rho_init = 1.0;
    double alm_gamma = 1.1;
    double max_rho = 100.0;
    double max_mu = 100.0;

    // Barrier parameters
    double barrier_beta = 0.8;
    double barrier_rho_init = 1.0;

    // 线搜索参数
    double alpha_ls_init = 1.0;
    double beta_ls = 0.8;
    double min_alpha = 1e-4;
};

class cost_base {
    /* 这个类用于定义代价函数的基类 */
protected:
    std::shared_ptr<cilqr_params> params_;
public:
    cost_base(std::shared_ptr<cilqr_params> params = std::make_shared<cilqr_params>()) : params_(params) {}
    virtual ~cost_base()=default;
    virtual std::shared_ptr<cilqr_params> get_params(){return params_;};
    virtual void set_params(const std::shared_ptr<cilqr_params>& params){params_=params;};
    // 定义系统单步前向离散动力学
    virtual casadi::SX dynamics(casadi::SX& state, casadi::SX& control)=0;
    // 定义原始每一时间步的代价函数,输入是原始笛卡尔状态，内部实现自己与参考轨迹的误差计算
    virtual casadi::SX cost_function(const casadi::SX& state,const casadi::SX& control,casadi::SX index)=0;
    // 定义终端代价函数
    virtual casadi::SX terminal_cost(const casadi::SX& state)=0;
    // 定义不等式约束函数，内部实现障碍物安全裕度代价，状态、控制数值边界约束(导数为常数)
    virtual casadi::SX constraints(const casadi::SX& state,const casadi::SX& control,casadi::SX index)=0;
};

class cilqr
{
private:

    // 代价函数指针，方便用户自定义代价函数
    std::shared_ptr<cost_base> cost_func_;
    std::shared_ptr<cilqr_params> params_;

    // 直接把[x, u]合并成z来定义函数，方便求导

    // 定义动力学相关函数接口方便调用
    casadi::Function f_, f_jacobian_,f_hessian_;
    // 定义代价函数相关接口方便调用
    casadi::Function l_, l_jacobian_, l_hessian_;
    // 终端代价函数及其一阶/二阶导数函数
    casadi::Function terminal_l_, terminal_l_x_, terminal_l_xx_;

    casadi::Function c_;
    // 约束代价累加后的导数函数,里面已经计入乘子和rho
    casadi::Function cost_vec_barrier_;
    casadi::Function cost_vec_alm_;
    casadi::Function cost_sum_barrier_;
    casadi::Function cost_sum_alm_;
    casadi::Function cost_sum_barrier_jac_;
    casadi::Function cost_sum_barrier_hess_;
    casadi::Function cost_sum_alm_jac_;
    casadi::Function cost_sum_alm_hess_;


    // 另外增加合并的 Jacobian/Hessian 块存储，用来按 z=[x;u] 的形式整体存储并加速求导
    Eigen::MatrixXd l_grad_vals_, l_hess_vals_;           // l_grad: N x (nx+nu), l_hess: N x (n*n)

    Eigen::MatrixXd f_jac_vals_, f_hess_vals_;           // f_jac: N x (nx*(n)), f_hess: N x (nx*(n*n))

    // 约束值存储,拿来更新乘子
    Eigen::MatrixXd c_vals_;

    // 约束的合并 Jacobian/Hessian 存储
    Eigen::MatrixXd c_jac_vals_, c_hess_vals_;

    // ALM 外点罚的合并 jac/hess 存储
    Eigen::MatrixXd alm_c_jac_vals_, alm_c_hess_vals_;

    // 存储拉格朗日乘子，时间步x约束个数
    Eigen::MatrixXd alm_mu_, alm_mu_next_;
    // 所有时间步的状态、控制、扰动、反馈增益矩阵、价值函数一阶二阶导数
    Eigen::MatrixXd u_, x_, d_;
    Eigen::MatrixXd K_, V_x_, V_xx_;
    

    int N_, nx_, nu_, num_c_;
    // bool parameter
    bool use_barrier_;
    // ALM parameters
    double alm_rho_, alm_rho_init_, alm_gamma_, max_rho_, max_mu_;
    // Barrier parameters
    double barrier_rho_,barrier_beta_;
    // 线搜索参数
    double alpha_ls_init_, beta_ls_;
    // 上一迭代代价
    double prev_cost_,curr_cost_,max_delta_u_,max_delta_x_;
    std::shared_ptr<AD_algorithm::general::Logger> logger_;

public:
    cilqr(std::shared_ptr<cost_base> cost_func);
    ~cilqr();
    cilqr_params get_parameters() const;
    void set_params(const std::shared_ptr<cilqr_params>& params);
    // 求解问题
    void solve();
    void set_cost_function(std::shared_ptr<cost_base> cost_func) { cost_func_ = cost_func; }
    void set_initial_state(const Eigen::VectorXd& x0) { if (x0.size() == nx_) x_.row(0) = x0; }
    const Eigen::MatrixXd& get_u() const { return u_; }
    const Eigen::MatrixXd& get_x() const { return x_; }

private:
    // 初始化
    void init();
    // 
    void get_init_traj_increment();
    // 
    void get_init_traj();
    // 定义一个迭代步，返回更新后的控制u、状态x和成本J
    bool iter_step();
    // 计算整条轨迹代价
    double get_total_cost(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u);
    // 计算代价函数的导数信息
    void get_cost_derivatives_and_Hessians();
    // 反向传播
    bool backward_pass(double reg_init);
    // 前向传播
    bool forward_pass();
    // 线搜索
    bool linear_search(double alpha);
};


}}