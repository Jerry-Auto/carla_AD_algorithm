#pragma once
#include <vector>
#include <memory>
#include <string>
#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include "general_modules/common_types.h"
#include "general_modules/FrenetFrame.h"

namespace AD_algorithm {
namespace controller {

    struct CilqrControllerParams {
        CilqrControllerParams() {
            ilqr_params = std::make_shared<AD_algorithm::general::ILQR_params>();
            ilqr_params->N = 20;            // horizon length，不包括终端状态
            ilqr_params->dt = 0.01;       // time step
            ilqr_params->max_iter = 100;      // 最大迭代次数
            ilqr_params->use_DDP = false;  // 使用DDP
            ilqr_params->nx = 4;             // state dimension
            ilqr_params->nu = 2;             // control dimension

            AD_algorithm::general::VehicleParams vp;
            vehicle_width = vp.width;
            vehicle_length = vp.length;
            wheelbase = vp.lf + vp.lr;
        }
        std::shared_ptr<AD_algorithm::general::ILQR_params> ilqr_params;

        bool use_alm = false;    // 是否使用ALM
        // 边界约束的惩罚系数初始化
        double alm_rho_init=20;
        double alm_beta = 2;  // 惩罚系数增长率
        double alm_rho_max=100; // 惩罚系数上限
        double alm_mu_max=100;  // 乘子裁剪上限
        // 使用指数障碍函数法，两个参数(q1）exp(q2*c),q1越大惩罚越大，q2越大惩罚越大
        double barrier_q1_init=3.0;  //内部的惩罚系数
        double barrier_q2_init=3.6;  // 外部的惩罚系数
        double barrier_q1_max = 3.0;  // 障碍系数上限
        double barrier_q2_max = 3.6;  // 障碍系数上限
        double barrier_beta1 = 1; // 障碍函数惩罚系数缩减率
        double barrier_beta2 = 1; // 障碍函数惩罚系数缩减率

        // 不等式边界默认值
        double max_a = 6.0;
        double min_a = -6.0;
        double max_v = 80.0; // m/s
        double min_v = 0.0;
        double max_steer = 0.5;
        double min_steer = -0.5;

        // 车辆几何参数
        double vehicle_width = 2.0;
        double vehicle_length = 4.5;
        double wheelbase = 2.8;
        double safety_distance = 0.9;
    };

class controller_cost : public AD_algorithm::general::cost_factory {
    /* 这个类提供以下功能：
    1.单步状态转移函数
    2.每一步的状态转移函数的导数(矩阵)及Hessian计算(张量)
    3.总代价计算，包括原始代价，以及乘子不等式代价
    4.0->N-1步总hessian(矩阵)和梯度(向量)计算，包括原始代价，以及乘子不等式代价
    5.终端hessian和梯度计算
    6.拉格朗日乘子及障碍系数管理，在每次计算完hessian后更新乘子和障碍系数
     */
private:
    int num_c_=6; // 除了障碍物相关的约束以外的约束数量
    double iter_=0;
    double cur_t_=0.0;
    // 固定车辆参数
    AD_algorithm::general::VehicleParams vehicle_params_= AD_algorithm::general::VehicleParams();

    std::shared_ptr<CilqrControllerParams> controller_params_;
    int n_x_, n_u_;
    std::vector<general::TrajectoryPoint> ref_trajectory_;
    Eigen::MatrixXd Q_, R_, Qf_;
    general::VehicleState initial_state_;
    std::shared_ptr<AD_algorithm::general::FrenetFrame> frenet_frame_;
    bool is_first_run_ = true;
    // 不等式边界约束项[a_max, v_max, steer_max]
    Eigen::VectorXd lower_bound_, upper_bound_;  

    bool shut_down_constraints_=false;

    // 障碍惩罚系数，所有都公用一个标量
    // ALM的惩罚系数是越大惩罚越大
    double alm_rho_;
    Eigen::MatrixXd alm_mu_must_; 
    // 障碍函数法(1/q2）exp(q1*c)的q1和q2，q1越大惩罚越大，q2越小惩罚越大
    double barrier_q1_;
    double barrier_q2_;
    
    // 记录各部分代价及其梯度和Hessian范数，用于调试，观察收敛情况
    double origin_cost_,constraint_cost_;
    double origin_grad_norm_,constraint_grad_norm_;
    double origin_hess_norm_,constraint_hess_norm_;
public:
    controller_cost(const std::shared_ptr<CilqrControllerParams>& params);
    ~controller_cost()=default; 
    void set_reference_trajectory(const std::vector<general::TrajectoryPoint>& trajectory);
    void set_initial_state(const general::VehicleState& x0,double cur_t) { initial_state_ = x0; cur_t_ = cur_t; }
    void set_cost_weights(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qf) {
        Q_ = Q;
        R_ = R;
        Qf_ = Qf;
    }
    void get_initial_trajectory(Eigen::MatrixXd& x_init, Eigen::MatrixXd& u_init) override;
    
public:
    bool use_alm() const override { return controller_params_->use_alm; }
    std::shared_ptr<AD_algorithm::general::ILQR_params> get_params() const override {return controller_params_->ilqr_params; }
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
    // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
    void update(int iter, const Eigen::MatrixXd& x_and_u) override;

private:

    void build_functions();
    // 状态转移函数
    void build_dynamics_function();
    void build_origin_cost_function();
    // 由于障碍物数量是不定的，因此需要分开处理，将障碍物相关的代价函数做成逐障碍物的形式
    void build_one_step_inequal_cost_function_without_obstacle();
    // 计算单步混合代价，包含原始代价和约束代价
    double compute_one_step_cost(const Eigen::VectorXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step);
    double compute_terminal_cost(const Eigen::VectorXd& x_N);
    Eigen::VectorXd compute_gradient_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step);
    Eigen::MatrixXd compute_hessian_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step);
    // 计算乘子导数，用于乘子更新
    Eigen::VectorXd compute_mu_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step);

public:
    // 安全零拷贝转换函数：从 casadi::DM 到 Eigen
    Eigen::MatrixXd to_matrix(const casadi::DM& dm) {
        casadi::DM dense = casadi::DM::densify(dm);
        int rows = dense.size1();
        int cols = dense.size2();
        return Eigen::Map<Eigen::MatrixXd>(dense.ptr(), rows, cols);
    }

    Eigen::VectorXd to_vector(const casadi::DM& dm) {
        casadi::DM dense = casadi::DM::densify(dm);
        int size = dense.size1() * dense.size2();
        return Eigen::Map<Eigen::VectorXd>(dense.ptr(), size);
    }

    // 从 Eigen 到 casadi::DM
    casadi::DM to_dm(const Eigen::MatrixXd& mat) {
        std::vector<double> data(mat.data(), mat.data() + mat.size());
        return casadi::DM::reshape(casadi::DM(data), mat.rows(), mat.cols());
    }

    casadi::DM to_dm(const Eigen::VectorXd& vec) {
        std::vector<double> data(vec.data(), vec.data() + vec.size());
        return casadi::DM::reshape(casadi::DM(data), vec.size(), 1);
    }

    casadi::DM to_dm(const Eigen::RowVectorXd& row_vec) {
        std::vector<double> data(row_vec.data(), row_vec.data() + row_vec.size());
        return casadi::DM::reshape(casadi::DM(data), 1, row_vec.size());
    }

private:

    casadi::Function f_, f_jacobian_, f_hessian_; //经过测试，与手动计算结果一致
    casadi::Function l_, l_jacobian_, l_hessian_; //经过测试，与手动计算结果一致

    // 这些函数是不依赖障碍物数量的
    casadi::Function alm_cost_,alm_cost_jacobian_,alm_cost_hessian_;
    casadi::Function barrier_cost_,barrier_cost_jacobian_,barrier_cost_hessian_;
    // 计算乘子导数的函数，用于更新乘子
    casadi::Function alm_mu_derivative_,alm_obstacle_mu_derivative_;
};


}}