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
    cost_base() : params_(std::make_shared<cilqr_params>()) {}
    virtual ~cost_base()=default;
    virtual std::shared_ptr<cilqr_params> get_params(){return params_;};
    virtual void set_params(const std::shared_ptr<cilqr_params>& params){params_=params;};
    // 定义系统单步前向离散动力学
    virtual casadi::SX dynamics(casadi::SX& state, casadi::SX& control)=0;
    // 定义原始每一时间步的代价函数,输入是原始笛卡尔状态，内部实现自己与参考轨迹的误差计算
    virtual casadi::SX cost_function(const casadi::SX& state,const casadi::SX& control, const casadi::SX& Q, const casadi::SX& R, const casadi::SX& ref)=0;
    // 定义终端代价函数
    virtual casadi::SX terminal_cost(const casadi::SX& state, const casadi::SX& Qf, const casadi::SX& terminal_ref)=0;
    // 定义不等式约束函数，内部实现障碍物安全裕度代价，状态、控制数值边界约束(导数为常数)
    virtual casadi::SX constraints(const casadi::SX& state,const casadi::SX& control, const casadi::SX& obstacles_vector, casadi::SX num_obstacles)=0;
};


class cost_factory {
    /* 这个类提供以下功能：
    1.单步状态转移函数
    2.总代价计算，包括原始代价，以及乘子不等式代价
    3.0->N-1步总hessian和梯度计算，包括原始代价，以及乘子不等式代价
    4.终端hessian和梯度计算
    5.拉格朗日乘子及障碍系数管理，在每次计算完hessian后更新乘子和障碍系数
    由于乘子数量是由约束的数量决定的，所以实际上代价计算、乘子更新与问题定义是耦合的，
    这里将此类作为虚基类，提供被ILQR调用的接口，具体实现由子类完成 */
public:
    cost_factory()=default;
    ~cost_factory()=default; 
    // 状态转移函数，输入是z=[x;u]合并后的向量
    virtual Eigen::MatrixXd state_transition(const Eigen::VectorXd& state_and_control)=0;
    // 计算总代价，包含原始代价和约束代价，输入是整个轨迹的状态和控制序列{[x,u],...}
    virtual double compute_total_cost(const Eigen::MatrixXd& x_and_u)=0;
    // 计算所有时间步的梯度,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的梯度按z=[x,u]合并后的形式存储
    virtual Eigen::MatrixXd compute_total_gradient(const Eigen::MatrixXd& x_and_u)=0;
    // 计算所有时间步的Hessian,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的Hessian按z=[x,u]合并后的形式存储
    virtual Eigen::MatrixXd compute_total_hessian(const Eigen::MatrixXd& x_and_u)=0;
    // 终端代价的梯度计算
    virtual Eigen::VectorXd compute_terminal_gradient(const Eigen::VectorXd& x_N)=0;
    // 终端代价的Hessian计算
    virtual Eigen::MatrixXd compute_terminal_hessian(const Eigen::VectorXd& x_N)=0;
};

class ILQR
{
    /* 实现标准的ILQR算法，约束的处理与乘子有关，最终只影响代价的数值，不会改变迭代的流程
    ，这里就假设代价、导数和hessian数值都是调用另一个类来处理，另一个类同时管理乘子的更新，
    在ILQR类中，实现前向反向传播逻辑，收敛判定及给出最优解,所有的数值使用Eigen，不涉及CasADi */
private:
    // 所有时间步的状态、控制、扰动、反馈增益矩阵、价值函数一阶二阶导数
    Eigen::MatrixXd u_, x_, d_;
    Eigen::MatrixXd K_, V_x_, V_xx_;

    // 线搜索参数
    double alpha_ls_init_, beta_ls_;
    // 上一迭代代价
    double prev_cost_,curr_cost_,max_delta_u_,max_delta_x_;
    std::shared_ptr<AD_algorithm::general::Logger> logger_;
public:
    ILQR();
    ~ILQR();
    void solve();
    const Eigen::MatrixXd& get_u() const { return u_; }
    const Eigen::MatrixXd& get_x() const { return x_; }
private:
    // 首次运行产生初始轨迹
    void get_init_traj();
    // 不是首次运行，使用上次结果作为初始轨迹
    void get_init_traj_increment();
    // 定义一个迭代步，返回更新后的控制u、状态x和成本J
    bool iter_step();
    // 计算整条轨迹代价
    double get_total_cost(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u);
    // 反向传播
    bool backward_pass(double reg_init);
    // 前向传播
    bool forward_pass();
    // 线搜索
    bool linear_search(double alpha);
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
    
    // Parameters for cost functions
    Eigen::MatrixXd Q_, R_, Qf_, terminal_ref_, ref_;
    Eigen::VectorXd obstacles_vector_;
    int num_obstacles_;
    

    int N_, nx_, nu_, num_c_;
    // bool parameter
    bool use_barrier_;
    // ALM parameters
    double alm_rho_, alm_gamma_, max_rho_, max_mu_;
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
    void set_Q(const Eigen::MatrixXd& Q) { Q_ = Q; }
    void set_R(const Eigen::MatrixXd& R) { R_ = R; }
    void set_Qf(const Eigen::MatrixXd& Qf) { Qf_ = Qf; }
    void set_terminal_ref(const Eigen::VectorXd& terminal_ref) { terminal_ref_ = terminal_ref; }
    void set_ref(const Eigen::MatrixXd& ref) { ref_ = ref; }
    void set_obstacles(const Eigen::VectorXd& obstacles_vector, int num_obstacles) { obstacles_vector_ = obstacles_vector; num_obstacles_ = num_obstacles; }

private:
    // 初始化
    void init();
    // 构建 casadi 函数
    void build_function();
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