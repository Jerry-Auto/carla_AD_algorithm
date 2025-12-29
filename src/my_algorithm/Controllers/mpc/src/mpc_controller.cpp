#include "mpc/mpc_controller.h"
#include "general_modules/math_tool.h"
using namespace AD_algorithm::general;

namespace AD_algorithm {
namespace controller {

MPCController::MPCController() : ControllerBase("MPCController")
{
    if (!vehicle_params_) {
        vehicle_params_ = std::make_shared<general::VehicleParams>();
    }

    // 状态矩阵初始化
    _matrix_A.setZero(_state_dim, _state_dim);
    _matrix_Ad.setZero(_state_dim, _state_dim);
    _matrix_B.setZero(_state_dim, _control_dim);
    _matrix_Bd.setZero(_state_dim, _control_dim);
    _matrix_state.setZero(_state_dim, 1);
    _matrix_control.setZero(_horizon * _control_dim, 1);
    _matrix_Q.setZero(_state_dim, _state_dim);
    _matrix_R.setZero(_control_dim, _control_dim);

    // 与速度无关的矩阵B在这里直接赋值
    // 注意：这里假设 VehicleParams 中的 cf, cr 为正值，而 MPC 方程中需要负值
    double cf = -vehicle_params_->cf;
    double m = vehicle_params_->mass;
    double lf = vehicle_params_->lf;
    double Iz = vehicle_params_->iz;

    _matrix_B(1, 0) = - cf / m;
    _matrix_B(3, 0) = -lf * cf / Iz;
    // 这里取1,纵向误差误差形式取***真实值-参考值***
    _matrix_B(5, 1) = 1;

    // 求解器
    _mpc_solver = std::make_unique<OsqpEigen::Solver>();
    _mpc_solver->settings()->setWarmStart(true);


}

bool MPCController::set_trajectory(const std::vector<TrajectoryPoint>& trajectory) {
    _trajectory = trajectory;
    return true;
}

// 计算控制指令
bool MPCController::compute_control_cmd(
    const std::shared_ptr<VehicleState>& ego_state,
    const double dt,
    const double cur_t,
    ControlCMD & cmd
)
{
    if (_trajectory.empty()) return false;

    // 1.计算横纵向误差,更新状态向量
    double predicted_time = cur_t + _preview_window*dt;
    update_state(_trajectory, ego_state, predicted_time);

    // 2.更新矩阵(A矩阵赋值,A与B矩阵的离散化)
    update_matrix(ego_state->v, dt);

    // 3.求解MPC问题
    if(!solve_mpc())
    {
        log("ERROR", "mpc二次规划求解失败!!!");
        return false;
    }

    // 4.生成控制指令
    // 4.1 提取反馈控制量
    double steer_feedback = _matrix_control(0,0);
    
    // 4.2 计算前馈控制量
    double steer_feedforward = compute_feedforward(ego_state->v);
    
    // 4.3 总转向指令
    double steer_cmd = steer_feedback + steer_feedforward;
    double steer_limit = vehicle_params_->max_steer;
    steer_cmd = clamp(steer_cmd, -steer_limit, steer_limit);
    cmd.set_steer(steer_cmd);
    
    // 4.4 加速度指令
    // 参考加速度是在更新状态矩阵的时候计算得出的
    double acc_feedback = _matrix_control(1, 0) + _ref_acc;
    double acc_cmd = acc_feedback + 1.0; // 补偿项
    cmd.set_acceleration(acc_cmd, ego_state->v);
    
    return true;
}

// 设置Q矩阵
void MPCController::set_matrix_Q(const std::vector<double>& vector_Q)
{
    vector_Q.size();
    _matrix_Q.rows();
    if (vector_Q.size() != static_cast<size_t>(_matrix_Q.rows()) ||
        vector_Q.size() != static_cast<size_t>(_matrix_Q.cols()))
    {
        log("ERROR", "输入维数不争取,设置Q矩阵失败");
    }

    for (size_t i = 0; i < vector_Q.size(); i++)
    {
        _matrix_Q(i, i) = vector_Q[i];
    }
}

// 设置R矩阵
void MPCController::set_matrix_R(const std::vector<double>& vector_R)
{
    if (vector_R.size() != static_cast<size_t>(_matrix_R.rows()) ||
        vector_R.size() != static_cast<size_t>(_matrix_R.cols()))
    {
        log("ERROR", "输入维数不争取,设置R矩阵失败");
    }
    
    for (size_t i = 0; i < vector_R.size(); i++)
    {
        _matrix_R(i, i) = vector_R[i];
    }
    
}


// 计算目标点
void MPCController::compute_target_point(
        const std::vector<TrajectoryPoint>& trajectory,
        const double predicted_time,
        TrajectoryPoint& target_point, double& target_point_s)
{
    // 这里更好的处理方式是在TrajectoryPoint里加入s成员变量
    // 但是那样做要修改的地方太多了所以还是使用index2s的方法
    std::vector<double> index2s;
    index2s.emplace_back(0.0);
    for (size_t i = 1; i < trajectory.size(); ++i)
    {
        index2s.emplace_back(
            std::hypot(trajectory[i].x - trajectory[i-1].x,
                       trajectory[i].y - trajectory[i-1].y)
            + index2s[i-1]
        );
    }

    int index_nearest = -1;
    for (int i = 0; i < static_cast<int>(trajectory.size())-1; ++i)
    {
        if (predicted_time >= trajectory[i].time_stamped && predicted_time < trajectory[i+1].time_stamped)
        {
            index_nearest = i;
        }
    }
    if (index_nearest == -1)
    {
        return ;
    }

    // 插值计算目标点
    double delta_t = (trajectory.at(index_nearest+1).time_stamped - trajectory.at(index_nearest).time_stamped);
    double dt = predicted_time - trajectory.at(index_nearest).time_stamped;

    double k_x = (trajectory.at(index_nearest+1).x - trajectory.at(index_nearest).x)/delta_t;
    target_point.x = trajectory.at(index_nearest).x + k_x*dt;

    double k_y = (trajectory.at(index_nearest+1).y - trajectory.at(index_nearest).y)/delta_t;
    target_point.y = trajectory.at(index_nearest).y + k_y*dt;

    double k_v = (trajectory.at(index_nearest+1).v - trajectory.at(index_nearest).v)/delta_t;
    target_point.v = trajectory.at(index_nearest).v + k_v*dt;


    double k_heading = (trajectory.at(index_nearest+1).heading - trajectory.at(index_nearest).heading)/delta_t;
    target_point.heading = trajectory.at(index_nearest).heading + k_heading*dt;
    target_point.heading = normalize_angle(target_point.heading);

    double k_a_tau = (trajectory.at(index_nearest+1).a_tau - trajectory.at(index_nearest).a_tau)/delta_t;    
    target_point.a_tau = trajectory.at(index_nearest).a_tau + k_a_tau*dt; 

    double k_s = (index2s[index_nearest+1] - index2s[index_nearest])/delta_t;
    target_point_s = index2s[index_nearest] + k_s*dt;
    
}

void MPCController::update_state(
                const std::vector<TrajectoryPoint>& trajectory,
                const std::shared_ptr<VehicleState>& ego_state,
                const double /*predicted_time*/)
{
    // 1.计算目标点
    FrenetFrame ref_line(trajectory);
    auto target=ref_line.get_matched_trj_point(ego_state->x,ego_state->y,ego_state->heading);
    // TrajectoryPoint target_point;
    double target_point_s=target.second;
    auto target_point=target.first;
    // compute_target_point(trajectory, predicted_time, target_point, target_point_s);

    // 2.计算车辆投影点
    FrenetPoint ego_pro=ref_line.cartesian_to_frenet(*ego_state);

    // 注:为了形式统一,所有误差均采用***真实值-参考值***
    // 3.计算纵向误差
    double station_error = ego_pro.s - target_point_s;
    double speed_error = ego_pro.s_dot - target_point.v;
    
    // 4.计算横向误差
    // 目标点的切向量与法向量
    Eigen::Vector2d tan_target(std::cos(target_point.heading), std::sin(target_point.heading));
    Eigen::Vector2d nor_target(-std::sin(target_point.heading), std::cos(target_point.heading));
    // 误差向量
    Eigen::Vector2d vector_error(ego_state->x - target_point.x, ego_state->y - target_point.y);
    double d_error = vector_error.dot(nor_target);
    double heading_error = ego_state->heading - target_point.heading;
    heading_error = normalize_angle(heading_error);
    double d_dot_error = ego_state->v * std::sin(heading_error);
    double heading_dot_error = ego_state->omega - target_point.kappa * ego_pro.s_dot;

    // 5.更新状态向量
    _matrix_state(0, 0) = d_error;
    _matrix_state(1, 0) = d_dot_error;
    _matrix_state(2, 0) = heading_error;
    _matrix_state(3, 0) = heading_dot_error;
    _matrix_state(4, 0) = station_error;
    _matrix_state(5, 0) = speed_error;

    // 将其他环节需要使用的信息存储起来
    _ref_acc = target_point.a_tau;
    _ref_kappa = target_point.kappa;
    _real_acc = ego_pro.s_dot_dot;

    log("INFO", "目标位置: x:", target_point.x, "y:", target_point.y, "v:", target_point.v, "heading:", target_point.heading);
    log("INFO", "实际位置: x:", ego_state->x, "y:", ego_state->y, "v:", ego_state->v, "heading:", ego_state->heading);
}

void MPCController::update_matrix(const double ego_v, const double dt)
{
    double v = std::max(ego_v, 1e-3); //提供最小速度保护

    double cf = -vehicle_params_->cf;
    double cr = -vehicle_params_->cr;
    double m = vehicle_params_->mass;
    double lf = vehicle_params_->lf;
    double lr = vehicle_params_->lr;
    double Iz = vehicle_params_->iz;

    // 更新A矩阵
    _matrix_A(0, 1) = 1.0;
    _matrix_A(1, 1) = (cf + cr)/(m * v);
    _matrix_A(1, 2) = -(cf + cr)/(m);
    _matrix_A(1, 3) = (lf*cf - lr*cr)/(m * v);
    _matrix_A(2, 3) = 1.0;
    _matrix_A(3, 1) = (lf*cf - lr*cr)/(Iz * v);
    _matrix_A(3, 2) = -(lf*cf - lr*cr)/(Iz);
    _matrix_A(3, 3) = (lf*lf*cf + lr*lr*cr)/(Iz * v);
    _matrix_A(4, 5) = 1.0;

    // 离散化矩阵
    Eigen::MatrixXd I;
    I.setIdentity(_state_dim, _state_dim);
    _matrix_Ad = (I - _matrix_A*dt/2.0).inverse() * (I + _matrix_A*dt/2.0);
    _matrix_Bd = _matrix_B * dt;
}

bool MPCController::solve_mpc()
{
    // matrix_Q_bar = diag{Q, Q,..., Q}
    Eigen::MatrixXd matrix_Q_bar;
    matrix_Q_bar.setZero((_horizon + 1) * _state_dim, (_horizon + 1) * _state_dim);
    for (int i = 0; i <= _horizon; i++)
    {
        for (int j = 0; j < _state_dim; j++)
        {
            matrix_Q_bar(i*_state_dim + j, i*_state_dim + j)  = _matrix_Q(j, j);     
        }
    }

    // matrix_R_bar = diag{R, R,.., R}
    Eigen::MatrixXd matrix_R_bar;
    matrix_R_bar.setZero(_horizon*_control_dim, _horizon*_control_dim);
    for (int i = 0; i < _horizon; i++)
    {
        for(int j = 0; j < _control_dim; j++)
        {
            matrix_R_bar(i*_control_dim + j, i*_control_dim + j) = _matrix_R(j, j);
        }
    }

    // X = M*x_k + C*U 中的M
    Eigen::MatrixXd matrix_M;
    matrix_M.setZero((_horizon + 1) * _state_dim, _state_dim);
    Eigen::MatrixXd matrix_M_block = Eigen::MatrixXd::Identity(_state_dim, _state_dim);
    // 利用M_block初始化M
    for (int i = 0; i <= _horizon; i++)
    {
        matrix_M.middleRows(i*_state_dim, _state_dim) = matrix_M_block;
        matrix_M_block = matrix_M_block * _matrix_Ad;
    }

    // X = M*x_k + C*U 中的C
    Eigen::MatrixXd matrix_C;
    matrix_C.setZero((_horizon + 1) * _state_dim, _horizon * _control_dim);
    Eigen::MatrixXd matrix_C_block(_matrix_Bd);
    for (int i = 1; i <= _horizon; i++)
    {
        // 这里的赋值时每一行的第一块单独赋值,剩余部分利用上一行赋值
        matrix_C.middleRows(i*_state_dim, _state_dim) 
        << matrix_C_block,
           matrix_C.block((i-1)*_state_dim, 0, _state_dim, (_horizon-1)*_control_dim);

        matrix_C_block = _matrix_Ad * matrix_C_block;
    }
    
    // J =  0.5*U'*H*U + 2*E*U
    Eigen::MatrixXd matrix_H = 2 * (matrix_C.transpose() * matrix_Q_bar * matrix_C + matrix_R_bar); 
    Eigen::VectorXd matrix_E = ((2.0 * _matrix_state.transpose() * matrix_M.transpose() *
                                 matrix_Q_bar * matrix_C).row(0)).transpose();
    // hessian矩阵要用稀疏矩阵这里转换一下
    Eigen::SparseMatrix<double> matrix_H_sparse(_horizon * _control_dim, _horizon * _control_dim);
    for (int i = 0; i < _horizon * _control_dim; i++)
    {
        for(int j = 0; j < _horizon * _control_dim; j++)
        {
            matrix_H_sparse.insert(i, j) = matrix_H(i, j);
        }
    }

    // 边界条件
    Eigen::SparseMatrix<double> matrix_constraint(_horizon * _control_dim, _horizon * _control_dim);
    for (int i = 0; i < _horizon * _control_dim; i++)
    {
        matrix_constraint.insert(i, i) = 1;
    }
    Eigen::VectorXd low_boundary, up_boundary;
    low_boundary.resize(_horizon * _control_dim);
    up_boundary.resize(_horizon * _control_dim);
    for (int i = 0; i < _horizon; i++)
    {
        low_boundary(2*i) = -vehicle_params_->max_steer;
        low_boundary(2*i + 1) = -vehicle_params_->max_decel;
        up_boundary(2*i) = vehicle_params_->max_steer;
        up_boundary(2*i + 1) = vehicle_params_->max_accel; 
    }

    // 求解二次规划
    _mpc_solver->data()->setNumberOfVariables(_horizon * _control_dim);
    _mpc_solver->data()->setNumberOfConstraints(_horizon * _control_dim);
    if(!_mpc_solver->data()->setHessianMatrix(matrix_H_sparse)) {return false;}
    if(!_mpc_solver->data()->setGradient(matrix_E)) {return false;}
    if(!_mpc_solver->data()->setLinearConstraintsMatrix(matrix_constraint)){return false;}
    if(!_mpc_solver->data()->setUpperBound(up_boundary)) {return false;}
    if(!_mpc_solver->data()->setLowerBound(low_boundary)) {return false;}
    if(!_mpc_solver->initSolver()) {return false;}
    if(_mpc_solver->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {return false;}

    auto solution = _mpc_solver->getSolution();
    for (int i = 0; i < _horizon * _control_dim; i++)
    {
        _matrix_control(i,0) = solution(i);
    }
    
    _mpc_solver->clearSolverVariables();
    _mpc_solver->data()->clearHessianMatrix();
    _mpc_solver->data()->clearLinearConstraintsMatrix();
    _mpc_solver->clearSolver();

    // debug信息
    log("DEBUG", "--------------最优解-----------------");
    log("DEBUG", "\n", solution);
    log("DEBUG", "\n", _matrix_control);

    return true;
}

// 求解前馈
double MPCController::compute_feedforward(const double ego_v)
{
    // 这里的前馈是一个不完整的前馈,因为没有反馈矩阵K可以使用,
    // 所有和反馈K有关项,都取为0.
    double v = std::max(ego_v, 1e-3);// 除0保护
    double cf = -vehicle_params_->cf;
    double cr = -vehicle_params_->cr;
    double m = vehicle_params_->mass;
    double lf = vehicle_params_->lf;
    double lr = vehicle_params_->lr;

    double k_v = - m / (lf + lr) * (lr / cf - lf / cr);
    return _ref_kappa * (lf + lr + k_v * v * v);
}


}}