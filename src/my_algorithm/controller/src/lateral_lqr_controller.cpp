#include "controller/lateral_lqr_controller.h"
#include <unsupported/Eigen/MatrixFunctions>
using namespace AD_algorithm::general;

namespace AD_algorithm {
namespace controller {


LateralLQRController::LateralLQRController(std::shared_ptr<general::Logger> logger)
    : logger_(logger)
{
    if (!logger_) {
        logger_ = std::make_shared<general::Logger>("LateralLQRController");
    }
    //车辆静态参数初始化
    _cf = -130000;//N/rad
    _cr = -130000;
    _m = 1845.0;
    _a = 2.852/2.0;
    _b = 2.852/2.0;
    _Iz = _a*_a*(_m/2.0) + _b*_b*(_m/2.0);
    _steer_ratio = 16.0;

    _preview_window = 1.0;  // 减少预览窗口到1秒
    _dt = 0.1;

    //lqr静态矩阵初始化
    // 使用标准4维LQR: [ed, ed_dot, ephi, ephi_dot]
    _matrix_size = 4; 
    _matrix_A = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_A_bar = Eigen::MatrixXd::Zero(_matrix_size, _matrix_size);
    _matrix_B = Eigen::MatrixXd::Zero(_matrix_size, 1);
    _matrix_B_bar = Eigen::MatrixXd::Zero(_matrix_size, 1);
    
    _matrix_K = Eigen::MatrixXd::Zero(1, _matrix_size);
    _matrix_err = Eigen::VectorXd::Zero(_matrix_size);
    _matrix_Q = Eigen::MatrixXd::Identity(_matrix_size, _matrix_size);
    _matrix_R = Eigen::MatrixXd::Identity(1, 1);

    _matrix_B(1,0) = -_cf/_m;
    _matrix_B(3,0) = -_a*_cf/_Iz;


    _matrix_Q(0, 0) = 0.5;   // 横向位置误差 (降低权重)
    _matrix_Q(1, 1) = 0.0;   // 横向速度误差 (参考代码设为0)
    _matrix_Q(2, 2) = 2.0;   // 航向角误差 (提高权重)
    _matrix_Q(3, 3) = 0.0;   // 航向角速度误差 (参考代码设为0)

    _matrix_R(0, 0) = 1.0;   // 控制量权重

    _iter_max = 1500;
    _tolerance = 0.01;

}

bool LateralLQRController::set_trajectory(const std::vector<general::TrajectoryPoint>& trajectory) {
    _frenet_frame = std::make_shared<general::FrenetFrame>(trajectory, false);
    return true;
}

bool LateralLQRController::compute_control_cmd(
                        const std::shared_ptr<VehicleState>& ego_state,
                        const double dt,
                        const double cur_t,
                        ControlCMD & cmd)
{
    (void)cur_t; // unused
    if (enable_logging_) {
        logger_->info("开始计算控制指令");
    }
    _dt = dt;

    // 1. 状态矩阵初始化与更新
    // 防止除0，设置最小速度
    _vx = std::max(ego_state->v, 0.5); 

    // 动态调整航向误差权重：速度越低，航向权重越小
    double q_psi = 10.0 + 90.0 * std::min(_vx / 10.0, 1.0); 
    _matrix_Q(2,2) = q_psi;

    // 动态调整 R 矩阵权重：高速时加大对控制量的惩罚，抑制震荡
    // 基础 R=1.0, 速度每增加 10m/s, R 增加 5.0
    double r_gain = 1.0 + 10.0 * (_vx*_vx / 10.0);
    _matrix_R(0, 0) = r_gain;

    // 更新状态矩阵A (4x4)
    // x' = Ax + Bu
    // x = [ed, ed_dot, ephi, ephi_dot]^T
    _matrix_A.setZero();
    _matrix_A(0, 1) = 1.0;
    _matrix_A(1, 1) = (_cf + _cr) / (_m * _vx);
    _matrix_A(1, 2) = -(_cf + _cr) / _m;
    _matrix_A(1, 3) = (_a * _cf - _b * _cr) / (_m * _vx); 
    _matrix_A(2, 3) = 1.0;
    _matrix_A(3, 1) = -(_b * _cr - _a * _cf) / (_Iz * _vx);
    _matrix_A(3, 2) = -(_a * _cf - _b * _cr) / _Iz;
    _matrix_A(3, 3) = (_a * _a * _cf + _b * _b * _cr) / (_Iz * _vx);
    
    // 更新输入矩阵B (4x1)
    _matrix_B.setZero();
    _matrix_B(1, 0) = -_cf / _m;
    _matrix_B(3, 0) = -_a * _cf / _Iz;

    // 2. 离散化 (使用梯形积分/双线性变换)
    // 物理模型离散化 (4维)
    auto I4 = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd temp = (I4 - 0.5 * dt * _matrix_A).inverse();
    _matrix_A_bar = temp * (I4 + 0.5 * dt * _matrix_A);
    _matrix_B_bar = temp * (_matrix_B * dt); 

    // 3. 求解LQR反馈矩阵 K (针对4维系统)
    if (!compute_lqr_feedack(_matrix_A_bar, _matrix_B_bar, _matrix_Q, _matrix_R, _iter_max, _tolerance))
    {
        if (enable_logging_) {
            logger_->error("反馈矩阵K求解失败");
        }
        _matrix_K = Eigen::MatrixXd::Zero(1, _matrix_size);
    }

    // 4. 计算误差向量 err
    // 使用预测状态计算误差
    std::shared_ptr<VehicleState> predicted_state;
    predict_ego_state(ego_state, predicted_state);

    double k_m = 0.0; // 匹配点的曲率
    if (!compute_err_vector(predicted_state, k_m))
    {
        if (enable_logging_) {
            logger_->error("误差向量计算失败");
        }
        return false;
    }

    // 检查误差是否过大
    if (fabs(_matrix_err[0]) > 5.0) {
        if (enable_logging_) {
            logger_->warn("横向误差过大: {:.2f}米，可能匹配点错误", _matrix_err[0]);
        }
    }

    // 5. 计算控制量
    // 5.1 LQR反馈控制量 u_fb
    double u_fb = -(_matrix_K * _matrix_err)[0];
    
    // 5.2 前馈控制量 u_ff (基于LQR增益的动力学前馈)
    double k3 = _matrix_K(0, 2); // 航向误差对应的增益
    double wheelbase = _a + _b;
    double delta_ff = k_m * (wheelbase - _b * k3 - (_m * _vx * _vx / wheelbase) * (_b / _cf + _a * k3 / _cr - _a / _cr));

    // 总输出 (前轮转角)
    double u = u_fb + delta_ff;

    // 6. 后处理
    // 6.1 动态幅值限制
    // 低速 35度，高速(30m/s) 降至 5度
    double max_steer_deg = 35.0;
    if (_vx > 10.0) {
        max_steer_deg = 35.0 - (_vx - 10.0) * 1.5;
        max_steer_deg = std::max(max_steer_deg, 5.0);
    }
    double max_u = max_steer_deg * M_PI / 180.0;
    double u_clamped = std::min(std::max(u, -max_u), max_u);

    // 6.2 动态增量限制
    // 与速度成反比：速度越高，允许的转角变化率越小
    // 基准: 10m/s 时允许 10度/帧
    double max_rate_deg = 60.0 / std::max(_vx*_vx*_vx, 1.0);
    max_rate_deg = std::min(std::max(max_rate_deg, 1.0), 10.0);
    double current_max_delta = max_rate_deg * M_PI / 180.0;

    double delta_u = u_clamped - _last_steer_cmd;
    delta_u = std::min(std::max(delta_u, -current_max_delta), current_max_delta);
    u = _last_steer_cmd + delta_u;

    // 更新记忆
    _last_steer_cmd = u;
    _last_u_fb = u_fb;

    // 7. 设置输出
    cmd.set_steer(u); 

    if (enable_logging_) {
        logger_->info("方向盘转角:{:.3f} (FB:{:.3f}, FF:{:.3f}, dFB:{:.3f})", u, u_fb, delta_ff, delta_u);
    }
    return true;
}

//设置q矩阵
void LateralLQRController::set_q_matrix(const std::vector<double>& q_vector)
{
    if (static_cast<int>(q_vector.size()) != _matrix_size)
    {
        if (enable_logging_) {
            logger_->error("输入维度不正确 (预期 {}, 实际 {}), 设置失败", _matrix_size, q_vector.size());
        }
        return;
    }

    for (size_t i = 0; i < q_vector.size(); i++)
    {
        _matrix_Q(i,i) = q_vector[i]; 
    }
}

//设置r矩阵
void LateralLQRController::set_r_matrix(const double r)
{
    _matrix_R(0,0) = r;
}

void LateralLQRController::predict_ego_state(
    const std::shared_ptr<VehicleState>& ego_state,
    std::shared_ptr<VehicleState>& ego_pre_state)
{
    // 创建新对象，而不是共享指针
    ego_pre_state = std::make_shared<VehicleState>();
    
    // 动态预瞄距离策略: D = D_min + k * v
    // 低速时看近一点保证精度，高速时看远一点保证稳定性
    double min_preview_dist = 0.1; 
    double preview_time_gain = 0.2; // 预瞄时间系数
    
    double preview_dist = min_preview_dist + preview_time_gain * ego_state->v;
    
    // 计算等效预瞄时间 (用于预测航向角变化)
    double v_safe = std::max(ego_state->v, 1.0);
    double preview_time = preview_dist / v_safe;
    
    // 简单直线运动预测
    ego_pre_state->x = ego_state->x + preview_dist * cos(ego_state->heading);
    ego_pre_state->y = ego_state->y + preview_dist * sin(ego_state->heading);
    ego_pre_state->heading = ego_state->heading + ego_state->omega * preview_time;
    ego_pre_state->v = ego_state->v;
    ego_pre_state->omega = ego_state->omega;
    ego_pre_state->time_stamp = ego_state->time_stamp + preview_time;
}

bool LateralLQRController::compute_err_vector(
    const std::shared_ptr<VehicleState>& ego_state, double& matched_kappa)
{
    // 1. 获取匹配点
    double ego_x = ego_state->x;
    double ego_y = ego_state->y;
    double ego_heading = ego_state->heading;
    
    auto target_msg = _frenet_frame->get_matched_trj_point(
        ego_x, ego_y, ego_heading);
    const TrajectoryPoint& target_point = target_msg.first;
    
    // 2. 计算横向误差 ed
    // 车辆到匹配点的向量
    double dx = ego_x - target_point.x;
    double dy = ego_y - target_point.y;
    
    // 匹配点的切向单位向量和法向单位向量
    double cos_theta_r = cos(target_point.heading);
    double sin_theta_r = sin(target_point.heading);
    
    // 横向误差：车辆位置到参考轨迹的垂直距离
    double ed = -dx * sin_theta_r + dy * cos_theta_r;
    
    // 3. 计算航向误差 ephi
    double ephi = ego_heading - target_point.heading;
    ephi = atan2(sin(ephi), cos(ephi));
    // 使用 sin(ephi) 增加大角度下的稳定性
    double sin_ephi = std::sin(ephi);

    // 4. 计算误差变化率
    // ed_dot ≈ v * sin(ephi)
    double ed_dot = ego_state->v * sin_ephi;
    
    // ephi_dot (简化计算，减少噪声干扰)
    // 使用目标点的速度作为参考
    double ephi_dot = ego_state->omega - target_point.kappa * target_point.v;
    
    // 5. 存储误差状态
    _matrix_err[0] = ed;
    _matrix_err[1] = ed_dot;
    _matrix_err[2] = sin_ephi;
    _matrix_err[3] = ephi_dot;
    matched_kappa = target_point.kappa;
    
    if (enable_logging_) {
        logger_->info("误差状态: ed={:.3f}, ed_dot={:.3f}, ephi={:.3f}, ephi_dot={:.3f}",
                    ed, ed_dot, ephi, ephi_dot);
    }
    
    return true;
}


bool LateralLQRController::compute_lqr_feedack(
     const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
     const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
     const int iter_max,const double tolerance)
{
    // 1. 维度检查
    if(A.rows()!=A.cols() || B.rows()!=A.rows() || Q.rows()!=Q.cols() || Q.rows()!=A.rows() || R.cols()!=B.cols())
    {
        if (enable_logging_) {
            logger_->error("输入矩阵维数不匹配");
        }
        return false;
    }

    auto matrix_size = A.rows();
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::MatrixXd P_next = Eigen::MatrixXd::Zero(matrix_size, matrix_size);
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();

    // 2. 迭代求解黎卡提方程 (DARE)
    // P_{k+1} = Q + A^T * P_k * A - A^T * P_k * B * (R + B^T * P_k * B)^-1 * B^T * P_k * A
    for (int i = 0; i < iter_max; i++)
    {   
        P_next = Q + AT*P*A - AT*P*B*(R+BT*P*B).inverse()*BT*P*A;
        
        // 收敛性检查
        if (fabs((P_next-P).maxCoeff()) < tolerance)
        {
            _matrix_K = (R + BT*P*B).inverse()*(BT*P*A);
            return true;
        }
        P = P_next;
    }
    
    if (enable_logging_) {
        logger_->error("LQR迭代达到最大次数仍未收敛");
    }
    return false;
}
} // namespace controller
} // namespace AD_algorithm