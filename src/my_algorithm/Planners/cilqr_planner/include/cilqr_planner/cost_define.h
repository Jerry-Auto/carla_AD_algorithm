#pragma once
#include <vector>
#include <memory>
#include <string>
#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/planner_base.h"
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"
#include "cilqr_planner/planner_weight.h"

namespace AD_algorithm {
namespace planner {

class planner_problem : public AD_algorithm::general::cost_factory {
    /* 这个类提供以下功能：
    1.单步状态转移函数
    2.每一步的状态转移函数的导数(矩阵)及Hessian计算(张量)
    3.总代价计算，包括原始代价，以及乘子不等式代价
    4.0->N-1步总hessian(矩阵)和梯度(向量)计算，包括原始代价，以及乘子不等式代价
    5.终端hessian和梯度计算
    6.拉格朗日乘子及障碍系数管理，在每次计算完hessian后更新乘子和障碍系数
     */

protected:
    int n_x_, n_u_;  // 状态和控制维度，从参数中获取

private:
    std::shared_ptr<CILQRPlannerparams> planner_params_;
    
    //不等式边界约束项[road_upper_bound, a_max, v_max, steer_max],每次只需要更新道路边界即可
    Eigen::VectorXd lower_bound_, upper_bound_;  
    double reference_speed_=8.0; // 参考速度，由外部设置
    int num_c_=8; // 除了障碍物相关的约束以外的约束数量
    double iter_=0;
    // 障碍物数量
    int num_obstacles_=0;
    std::vector<AD_algorithm::general::Obstacle> obstacles_;
    // 全局frenet坐标系
    std::shared_ptr<AD_algorithm::general::FrenetFrame> global_frenet_frame_;
    // 固定车辆参数
    AD_algorithm::general::VehicleParams vehicle_params_= AD_algorithm::general::VehicleParams();

    // 代价权重矩阵,Q的前两位是位置误差权重，第三位是速度误差权重，第四位是航向误差权重
    Eigen::MatrixXd Q_, R_, Qf_; 

    
    // 障碍惩罚系数，所有都公用一个标量
    // ALM的惩罚系数是越大惩罚越大
    double alm_rho_;
    double alm_rho_next_;
    // 障碍函数法的系数是越小惩罚越大
    double barrier_rho_;
    double barrier_rho_next_;
    
    // 障碍物相关的乘子，数量是不定的，每次求解在设置障碍物时初始化
    Eigen::MatrixXd alm_mu_obstacles_;
    Eigen::MatrixXd alm_mu_obstacles_next_;

    // 除了障碍物乘子以外的乘子，数量是固定的
    Eigen::MatrixXd alm_mu_must_; 
    Eigen::MatrixXd alm_mu_must_next_;

public:
    planner_problem(const std::shared_ptr<CILQRPlannerparams>& params);
    ~planner_problem()=default; 
    void set_frenet_frame(std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame);
    void set_obstacles(const std::vector<AD_algorithm::general::Obstacle>& obstacles);
    void set_reference_speed(double v_ref);
    void set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf);
    void set_road_bounds(const double& lower_bound, const double& upper_bound);

public:
/* 以下是重载的函数 */
    std::shared_ptr<AD_algorithm::general::ILQR_params> get_params() const override {return planner_params_->ilqr_params; }
    // 状态转移函数，输入是z=[x;u]合并后的向量
    Eigen::VectorXd state_transition(const Eigen::VectorXd& state_and_control) override;
    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,输出是雅可比矩阵堆成的张量
    Eigen::Tensor<double, 3,Eigen::RowMajor> state_total_jacobian(const Eigen::MatrixXd& x_and_u) override;
    // 计算状态转移函数的Hessian张量，输入是z=[x;u]合并后的向量,输出是Hessian堆成的四维张量
    Eigen::Tensor<double, 4,Eigen::RowMajor> state_total_hessian(const Eigen::MatrixXd& x_and_u) override;
    // 计算总代价，包含原始代价和约束代价，输入是整个轨迹的状态和控制序列{[x,u],...}
    double compute_total_cost(const Eigen::MatrixXd& x_and_u) override;
    // 计算所有时间步的梯度,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的梯度按z=[x,u]合并后的形式存储
    Eigen::MatrixXd compute_total_gradient(const Eigen::MatrixXd& x_and_u) override;
    // 计算所有时间步的Hessian,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的Hessian按z=[x,u]合并后的形式存储
    Eigen::Tensor<double, 3,Eigen::RowMajor> compute_total_hessian(const Eigen::MatrixXd& x_and_u) override;
    // 终端代价的梯度计算
    Eigen::VectorXd compute_terminal_gradient(const Eigen::VectorXd& x_N) override;
    // 终端代价的Hessian计算
    Eigen::MatrixXd compute_terminal_hessian(const Eigen::VectorXd& x_N) override;
    // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
    void update(int iter) override;

private:
    void build_functions();
    // 状态转移函数
    void build_dynamics_function();
    void build_terminal_cost_function();
    void build_origin_cost_function();
    // 由于障碍物数量是不定的，因此需要分开处理，将障碍物相关的代价函数做成逐障碍物的形式
    void build_one_step_inequal_cost_function_without_obstacle();
    void build_one_step_obstacle_cost_functions();
    // 计算单步混合代价，包含原始代价和约束代价
    double compute_one_step_cost(const Eigen::VectorXd& x_and_u,int step);
    double compute_terminal_cost(const Eigen::VectorXd& x_N);
    Eigen::VectorXd compute_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step);
    Eigen::MatrixXd compute_hessian_one_step(const Eigen::MatrixXd& x_and_u,int step);
    // 计算乘子导数，用于乘子更新
    Eigen::VectorXd compute_mu_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step);


    casadi::Function f_, f_jacobian_, f_hessian_;
    casadi::Function l_, l_jacobian_, l_hessian_;
    // 终端代价函数
    casadi::Function terminal_l_, terminal_l_jacobian_, terminal_l_hessian_;
    // 这些函数是不依赖障碍物数量的
    casadi::Function alm_cost_,alm_cost_jacobian_,alm_cost_hessian_;
    casadi::Function barrier_cost_,barrier_cost_jacobian_,barrier_cost_hessian_;
    // 计算乘子导数的函数，用于更新乘子
    casadi::Function alm_mu_derivative_,alm_obstacle_mu_derivative_;
    // 障碍物处理相关，逐障碍物处理
    casadi::Function alm_obstacle_cost_, alm_obstacle_jacobian_, alm_obstacle_hessian_;
    casadi::Function barrier_obstacle_cost_, barrier_obstacle_jacobian_, barrier_obstacle_hessian_;

};

}}