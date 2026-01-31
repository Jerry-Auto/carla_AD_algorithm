#pragma once
#include <vector>
#include <memory>
#include <string>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/planner_base.h"
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"
#include "cilqr_planner/planner_cost.h"

namespace AD_algorithm {
namespace planner {

enum class SolveType { BARRIER, ALM };
enum class BoundType { UPPER, LOWER };
enum class ReferencePoint { GravityCenter, RearCenter };

class cilqr_eigen : public planner_cost {
    /* 这个类提供以下功能：
    1.单步状态转移函数
    2.每一步的状态转移函数的导数(矩阵)及Hessian计算(张量)
    3.总代价计算，包括原始代价，以及乘子不等式代价
    4.0->N-1步总hessian(矩阵)和梯度(向量)计算，包括原始代价，以及乘子不等式代价
    5.终端hessian和梯度计算
    6.拉格朗日乘子及障碍系数管理，在每次计算完hessian后更新乘子和障碍系数
     */

public:
    cilqr_eigen(const std::shared_ptr<CILQRPlannerparams>& params);
    ~cilqr_eigen()=default; 
public:
    void set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles) override;

    std::shared_ptr<AD_algorithm::general::ILQR_params> get_params() const override {return planner_params_->ilqr_params; }
    bool use_alm() const override { return planner_params_->use_alm; }
    // 状态转移函数，输入是z=[x;u]合并后的向量
    Eigen::VectorXd state_transition(const Eigen::VectorXd& state_and_control) override;
    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,输出是雅可比矩阵堆成的张量，输入有N+1个，最后一个不管
    Eigen::Tensor<double, 3,Eigen::ColMajor> state_total_jacobian(const Eigen::MatrixXd& x_and_u) override;
    // 计算状态转移函数的Hessian张量，输入是z=[x;u]合并后的向量,输出是Hessian堆成的四维张量，输入有N+1个，最后一个不管
    Eigen::Tensor<double, 4,Eigen::ColMajor> state_total_hessian(const Eigen::MatrixXd& x_and_u) override;
    // 计算总代价，包含原始代价和约束代价，输入是整个轨迹的状态和控制序列{[x,u],...}
    double compute_total_cost(const Eigen::MatrixXd& x_and_u) override;
    // 直接获得所有时间步的代价对输入[x,u]的梯度和hessian，包括终端的，放在最后一位，序列长度N+1
    void get_total_jacobian_hessian(const Eigen::MatrixXd& x_and_u,Eigen::MatrixXd&total_jacbian,Eigen::Tensor<double, 3,Eigen::ColMajor>&total_hessian) override;
    // 计算所有时间步的梯度,输入是整个轨迹的状态和控制{[x,u],...}，输出是每个时间步的梯度按z=[x,u]合并后的形式存储，输入有N+1个，最后一个不管
    // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
    void update(int iter, const Eigen::MatrixXd& x_and_u) override;

private:
    void get_total_cost_derivatives_and_Hessians(const Eigen::MatrixXd& x_and_u);
    double get_bound_constr(double variable, double bound, BoundType bound_type);
    Eigen::Vector2d get_obstacle_avoidance_constr(const Eigen::Vector4d& ego_state,
                                                  const general::Obstacle& obs_state);
    double exp_barrier(double c, double q1, double q2);
    double augmented_lagrangian_item(double c, double rho, double mu);
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> exp_barrier_derivative_and_Hessian(
        double c, Eigen::MatrixXd c_dot, double q1, double q2);
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> lagrangian_derivative_and_Hessian(
        double c, Eigen::MatrixXd c_dot, double rho, double mu);
    std::tuple<Eigen::Vector2d, Eigen::Vector2d> get_vehicle_front_and_rear_centers(
        const Eigen::Vector4d& state, double wheelbase,
        ReferencePoint ref_point = ReferencePoint::GravityCenter);
    std::tuple<Eigen::Matrix<double, 4, 2>, Eigen::Matrix<double, 4, 2>>
    get_vehicle_front_and_rear_center_derivatives(double yaw, double wheelbase,
                                                  ReferencePoint ref_point);
    Eigen::Vector2d get_ellipsoid_obstacle_scales(const general::Obstacle& obs,
                                                  double ego_pnt_radius = 0);
    double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const general::Obstacle& obs_state,
                                   const Eigen::Vector2d& ellipse_ab);
    Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                        const general::Obstacle& obs_state,
                                                        const Eigen::Vector2d& ellipse_ab);
    std::tuple<Eigen::Vector4d, Eigen::Vector4d> get_obstacle_avoidance_constr_derivatives(
        const Eigen::Vector4d& ego_state, const general::Obstacle& obs_state);

    // Member variables
    Eigen::MatrixXd alm_mu_must_;
    Eigen::MatrixXd alm_mu_must_next_;
    Eigen::MatrixXd alm_mu_obstacles_;
    Eigen::MatrixXd alm_mu_obstacles_next_;
    double alm_rho_;
    double obstacle_alm_rho_;

    SolveType solve_type_;
    
    double barrier_q1_;
    double barrier_q2_;
    double obstacle_barrier_q1_;
    double obstacle_barrier_q2_;
    
    int num_c_;
    ReferencePoint reference_point_;
    Eigen::MatrixXd l_x_, l_u_, l_xx_, l_uu_, l_xu_, l_ux_;
};

}}