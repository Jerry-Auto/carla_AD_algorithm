#pragma once

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/Vehicle.h"
#include "general_modules/logger.h"

namespace AD_algorithm {
namespace controller {

class LateralLQRController
{
public:
    LateralLQRController(std::shared_ptr<general::Logger> logger = nullptr);

    // 设置参考轨迹
    bool set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory);

    // 设置车辆参数
    void set_vehicle_params(std::shared_ptr<general::VehicleParams> params) {
        vehicle_params_ = params;
    }

    // 计算控制指令
    bool compute_control_cmd(
        const std::shared_ptr<general::VehicleState>& ego_state,
        const double dt,
        const double cur_t,
        general::ControlCMD & cmd
    );

    void set_log_enable(bool enable) {
        enable_logging_ = enable;
        if (logger_) logger_->set_enable(enable);
    }

    template<typename... Args>
    void log(Args&&... args) {
        if (logger_) logger_->log(std::forward<Args>(args)...);
    }

private:
    // 车辆参数
    std::shared_ptr<general::VehicleParams> vehicle_params_;
    double _vx; // 沿着车身轴线的速度

    // lqr参数
    int _matrix_size;
    Eigen::MatrixXd _matrix_A; // 状态矩阵
    Eigen::MatrixXd _matrix_A_bar; // 离散化的状态矩阵
    Eigen::MatrixXd _matrix_B; // 输入矩阵
    Eigen::MatrixXd _matrix_B_bar; // 离散化的矩阵
    Eigen::MatrixXd _matrix_K; // 反馈矩阵
    Eigen::VectorXd _matrix_err; // 误差向量
    Eigen::MatrixXd _matrix_Q; // Q矩阵
    Eigen::MatrixXd _matrix_R; // R矩阵
    int _iter_max; // 最大迭代次数
    double _tolerance; // 迭代精度

    double _preview_window = 5.0;
    double _dt = 0.01;
    std::shared_ptr<general::FrenetFrame>  _frenet_frame;
    
public:
    // 设置q矩阵
    void set_q_matrix(const std::vector<double>& q_vector);

    // 设置r矩阵
    void set_r_matrix(const double r);

private:
    bool compute_err_vector(const std::shared_ptr<general::VehicleState>& ego_state,double& matched_kappa);
    // 预测模块
    void predict_ego_state(const std::shared_ptr<general::VehicleState>& ego_state,
        std::shared_ptr<general::VehicleState>& ego_pre_state);

    // 计算反馈矩阵           
    bool compute_lqr_feedack(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                             const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                             const int iter_max,
                             const double tolerance);
    double _last_steer_cmd = 0.0;  // 上一次的方向盘转角（rad）
    double _last_u_fb = 0.0;       // 上一次的反馈控制量（增量式控制用）

    const double _max_delta_steer = 10.0 * M_PI / 180.0; // 最大每步变化量，

    std::shared_ptr<general::Logger> logger_;
    bool enable_logging_ = true;
};

}}