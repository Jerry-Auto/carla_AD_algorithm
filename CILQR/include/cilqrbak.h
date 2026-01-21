#pragma once
#include <vector>
#include <tuple>
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>

namespace AD_algorithm {
namespace general {
struct cilqr_params {
    bool use_DDP = false;      // true for DDP, false for iLQR
    bool use_Barrier = false;  // true for barrier, false for ALM
    int max_iter = 50;      // 最大迭代次数
    int max_reg_iter = 10; // 最大正则化尝试次数
    int N = 50;            // horizon length
    int nx = 4;             // state dimension
    int nu = 2;          // control dimension

    double dt=0.1;       // time step
    double tol = 1e-6;     // 数值稳定性容限
    double reg_min = 1e-6;
    double reg_max = 1e10;
    double reg_init = 1.0;
    double reg_factor = 10.0;
    double alpha = 0.2;
    double beta = 0.5;
    double mu = 0.0;
    double mu_min = 1e-6;
    double mu_max = 1e10;
    double mu_factor = 10.0;
    double cost_tol = 1e-4;
    double diverge_thres = 1e6;

    // ALM parameters
    double alm_rho = 1.0;
    double alm_rho_init = 1.0;
    double alm_gamma = 1.1;
    double max_rho = 100.0;
    double max_mu = 100.0;

    // Barrier parameters
    double barrier_beta = 0.5;
    double barrier_rho_init = 1.0;
    // 线搜索参数
    double alpha_ls_init = 1.0;
    double beta_ls = 0.5;
};

class cost_base {
    /* 这个类用于定义代价函数的基类 */
private:
    std::shared_ptr<cilqr_params> params_;
public:
    cost_base()=default;
    virtual ~cost_base()=default;
    virtual cilqr_params get_params(){return params_;};
    virtual void set_params(const cilqr_params& params){params_=params;};
    // 定义系统单步前向离散动力学
    virtual casadi::SX dynamics(casadi::SX& state, casadi::SX& control)=0;
    // 定义原始每一时间步的代价函数,输入是原始笛卡尔状态，内部实现自己与参考轨迹的误差计算
    virtual casadi::SX cost_function(const casadi::SX& state,const casadi::SX& control)=0;
    // 定义终端代价函数
    virtual casadi::SX terminal_cost(const casadi::SX& state)=0;
    // 定义不等式约束函数，内部实现障碍物安全裕度代价，状态、控制数值边界约束(导数为常数)
    virtual casadi::SX constraints(const casadi::SX& state,const casadi::SX& control)=0;
};

class cilqr
{
private:
    // 代价函数指针，方便用户自定义代价函数
    std::shared_ptr<cost_base> cost_func_;
    std::shared_ptr<cilqr_params> params_;
    

    // 定义动力学相关函数接口方便调用
    casadi::Function f_, f_x_, f_u_,f_xx_,f_uu_,f_ux_;
    // 定义代价函数相关接口方便调用
    casadi::Function l_, l_x_, l_u_, l_xx_, l_ux_, l_uu_;
    // 终端代价函数及其一阶/二阶导数函数
    casadi::Function terminal_l_, terminal_l_x_, terminal_l_xx_;
    // 定义约束相关函数，使用障碍函数时，就是除了乘子之后的log项,使用ALM时，就是原始约束函数，同时另外加外点罚函数导数
    casadi::Function c_, c_x_, c_u_, c_xx_, c_uu_, c_ux_;
    // 使用ALM时，存储外点罚函数项的函数及导数
    casadi::Function alm_c_, alm_c_x_, alm_c_u_, alm_c_xx_, alm_c_uu_, alm_c_ux_;

    // 所有时间步的状态、控制、扰动、反馈增益矩阵、价值函数一阶二阶导数
    Eigen::MatrixXd u_, x_, d_, K_, V_x_, V_xx_;
    // 存储原始代价函数导数信息
    Eigen::MatrixXd l_x_vals_, l_u_vals_, l_xx_vals_, l_uu_vals_, l_ux_vals_, f_x_vals_, f_u_vals_;
    // ALM存储乘子项代价函数导数，barrier存储障碍函数项代价函数导数
    Eigen::MatrixXd c_vals_, c_x_vals_, c_u_vals_,c_xx_vals_,c_uu_vals_,c_ux_vals_;
    // 使用ALM时，存储外点罚函数项的值及导数
    Eigen::MatrixXd alm_c_vals_, alm_c_x_vals_, alm_c_u_vals_,alm_c_xx_vals_,alm_c_uu_vals_,alm_c_ux_vals_;
    // 存储拉格朗日乘子，时间步x约束个数
    Eigen::MatrixXd alm_mu_, alm_mu_next_;


    int N_, nx_, nu_, num_c_;
    // ALM parameters
    double alm_rho_, alm_rho_init_, alm_gamma_, max_rho_, max_mu_;
    // Barrier parameters
    double barrier_rho_,barrier_beta_;
    // 线搜索参数
    double alpha_ls_init_, beta_ls_;
    // 上一迭代代价
    double prev_cost_;
public:
    cilqr(std::shared_ptr<cost_base> cost_func);
    ~cilqr();
    void set_parameters(const cilqr_params& params);
    cilqr_params get_parameters() const;
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
    void iter_step();
    // 计算整条轨迹代价
    double get_total_cost(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u);
    // 计算代价函数的导数信息
    void get_cost_derivatives_and_Hessians();
    // 反向传播
    void backward_pass();
    // 前向传播
    void forward_pass();
    // 线搜索
    bool linear_search(double alpha);
    // 组合代价函数项，使用障碍函数法或者ALM法
    void compose_l_derivatives(
        Eigen::VectorXd &l_x,Eigen::VectorXd &l_u,
        Eigen::VectorXd &l_xx,Eigen::VectorXd &l_uu,
        Eigen::VectorXd &l_ux,int k) const;
};


}}