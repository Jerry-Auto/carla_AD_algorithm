#pragma once
#include <vector>
#include <tuple>
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/logger.h"
#include <unsupported/Eigen/CXX11/Tensor>
#include "iostream"

namespace AD_algorithm {
namespace general {

struct ILQR_params {
    // 线搜索参数
    double alpha_ls_init = 1.0;
    double beta_ls = 0.8;
    double min_alpha = 1e-4;
    double EPS = 1e-6;

    // 正则化参数
    double reg_max = 1e10;
    double reg_init = 1.0;
    double reg_factor = 10.0;
    int max_reg_iter = 10; // 最大正则化尝试次数

    // 收敛判定参数
    double delta_u_tol = 1e-4;
    double delta_x_tol = 1e-2;
    double cost_tol = 10;

    // 全局迭代参数
    int max_iter = 50;      // 最大迭代次数
    int N = 200;            // horizon length，不包括终端状态
    int nx = 4;             // state dimension
    int nu = 2;             // control dimension
    double dt = 0.05;       // time step

    // DDP
    bool use_DDP = true;
};

class cost_factory {
    /* 这个类提供以下功能：
    1.单步状态转移函数
    2.每一步的状态转移函数的导数(矩阵)及Hessian计算(张量)
    3.总代价计算，包括原始代价，以及乘子不等式代价
    4.0->N-1步总hessian(矩阵)和梯度(向量)计算，包括原始代价，以及乘子不等式代价
    5.终端hessian和梯度计算
    6.拉格朗日乘子及障碍系数管理，在每次计算完hessian后更新乘子和障碍系数
    由于乘子数量是由约束的数量决定的，所以实际上代价计算、乘子更新与问题定义是耦合的，
    这里将此类作为虚基类，提供被ILQR调用的接口，具体实现由子类完成 */
public:
    cost_factory()=default;
    ~cost_factory()=default; 
    // 状态转移函数，输入是z=[x;u]合并后的向量
    virtual Eigen::VectorXd state_transition(const Eigen::VectorXd& state_and_control)=0;
    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,输出是雅可比矩阵堆成的张量
    virtual Eigen::Tensor<double, 3,Eigen::RowMajor> state_total_jacobian(const Eigen::MatrixXd& x_and_u)=0;
    // 计算状态转移函数的Hessian张量，输入是z=[x;u]合并后的向量,输出是Hessian堆成的四维张量
    virtual Eigen::Tensor<double, 4,Eigen::RowMajor> state_total_hessian(const Eigen::MatrixXd& x_and_u)=0;
    // 计算总代价，包含原始代价和约束代价，输入是整个轨迹的状态和控制序列{[x,u],...}
    virtual double compute_total_cost(const Eigen::MatrixXd& x_and_u)=0;
    // 计算所有时间步的梯度,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的梯度按z=[x,u]合并后的形式存储
    virtual Eigen::MatrixXd compute_total_gradient(const Eigen::MatrixXd& x_and_u)=0;
    // 计算所有时间步的Hessian,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的Hessian按z=[x,u]合并后的形式存储
    virtual Eigen::Tensor<double, 3,Eigen::RowMajor> compute_total_hessian(const Eigen::MatrixXd& x_and_u)=0;
    // 终端代价的梯度计算
    virtual Eigen::VectorXd compute_terminal_gradient(const Eigen::VectorXd& x_N)=0;
    // 终端代价的Hessian计算
    virtual Eigen::MatrixXd compute_terminal_hessian(const Eigen::VectorXd& x_N)=0;
    // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
    virtual void update(int iter)=0;
    // 获取ILQR参数设定
    virtual std::shared_ptr<ILQR_params> get_params() const =0;
};


enum class LQRSolveStatus {
    RUNNING,
    CONVERGED,
    BACKWARD_PASS_FAIL,
    FORWARD_PASS_FAIL,
    FORWARD_PASS_SMALL_STEP,
};

class ILQR
{
    /* 实现标准的ILQR算法，约束的处理与乘子有关，最终只影响代价的数值，不会改变迭代的流程
    ，这里就假设代价、导数和hessian数值都是调用另一个类来处理，另一个类同时管理乘子的更新，
    在ILQR类中，实现前向反向传播逻辑，收敛判定及给出最优解,所有的数值使用Eigen，不涉及CasADi */
private:
    // 所有时间步的状态、控制、扰动、反馈增益矩阵、价值函数一阶二阶导数
    Eigen::MatrixXd u_, x_, new_u_, new_x_,last_solve_u_;
    Eigen::MatrixXd d_, K_, V_x_, V_xx_;

    // 代价函数工厂
    std::shared_ptr<cost_factory> cost_factory_;

    // 参数配置
    std::shared_ptr<ILQR_params> params_;

    // 正则化参数
    double reg_;

    // 收敛判定参数
    Eigen::Vector2d delta_V_ = {0.0, 0.0};
    double prev_cost_,curr_cost_,max_delta_u_,max_delta_x_;

    // 是否首次运行
    bool first_run_ = true;

    // 求解器状态
    LQRSolveStatus solve_status_ = LQRSolveStatus::RUNNING;
    
    // 日志记录器
    std::shared_ptr<AD_algorithm::general::Logger> logger_;

    
    template<typename... Args>
    void log(Args&&... args) const {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

public:
    ILQR(std::shared_ptr<cost_factory> cost_factory);
    ~ILQR()=default;
    void solve();
    void set_log_enable(bool enable) {
        if (logger_) logger_->set_enable(enable);
    }
    const Eigen::MatrixXd& get_u() const { return u_; }
    const Eigen::MatrixXd& get_x() const { return x_; }
    void set_initial_state(const Eigen::VectorXd& x0);

private:
    // 首次运行产生初始轨迹
    void get_init_traj();
    // 不是首次运行，使用上次结果作为初始轨迹
    void get_init_traj_increment();
    // 定义一个迭代步，返回更新后的控制u、状态x和成本J
    bool iter_step();
    // 反向传播
    bool backward_pass();
    // 前向传播
    void forward_pass(double alpha);
    // 线搜索
    bool linear_search(double alpha);
    // 计时器
    std::chrono::high_resolution_clock::time_point start_time_;
    void start_timer(){start_time_=std::chrono::high_resolution_clock::now();}
    double stop_timer(){
        auto end_time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
    }
};

}}