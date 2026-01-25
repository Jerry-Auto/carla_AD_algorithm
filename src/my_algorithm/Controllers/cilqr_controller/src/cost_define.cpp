#include "cilqr_controller/cost_define.h"

namespace AD_algorithm {
namespace controller {
    
controller_cost::controller_cost(const std::shared_ptr<CilqrControllerParams>& params)
    : controller_params_(params),
      n_x_(params->ilqr_params->nx),
      n_u_(params->ilqr_params->nu),
      ref_trajectory_(),
      Q_(Eigen::MatrixXd::Identity(n_x_, n_x_)),
      R_(Eigen::MatrixXd::Identity(n_u_, n_u_)),
      Qf_(Eigen::MatrixXd::Identity(n_x_, n_x_)),
      initial_state_(),
      frenet_frame_(nullptr),
      is_first_run_(true),
      lower_bound_(Eigen::Vector3d(params->min_a, params->min_v, params->min_steer)),
      upper_bound_(Eigen::Vector3d(params->max_a, params->max_v, params->max_steer)),
      shut_down_constraints_(true),
      alm_rho_(params->alm_rho_init),
      alm_mu_must_(Eigen::MatrixXd::Zero(params->ilqr_params->N + 1, num_c_)),
      barrier_q1_(params->barrier_q1_init),
      barrier_q2_(params->barrier_q2_init),
      origin_cost_(0.0),
      constraint_cost_(0.0),
      origin_grad_norm_(0.0),
      constraint_grad_norm_(0.0),
      origin_hess_norm_(0.0),
      constraint_hess_norm_(0.0) {
    build_functions();
}

void controller_cost::get_initial_trajectory(Eigen::MatrixXd& x_init, Eigen::MatrixXd& u_init)
{
    const int N = controller_params_->ilqr_params->N;
    x_init = Eigen::MatrixXd::Zero(N + 1, n_x_);
    u_init = Eigen::MatrixXd::Zero(N, n_u_);

    Eigen::Vector4d cur_x(initial_state_.x, initial_state_.y, initial_state_.heading,
                        initial_state_.v);
    x_init.row(0) = cur_x.transpose();
    if (!is_first_run_ && last_u_.rows() == N && last_u_.cols() == n_u_) {
        u_init.block(0, 0, N - 1, n_u_) = last_u_.block(1, 0, N - 1, n_u_);
        u_init.block(N - 1, 0, 1, n_u_) = last_u_.block(N - 1, 0, 1, n_u_);
    }
    is_first_run_ = false;

    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd state_and_control(n_x_ + n_u_);
        state_and_control << cur_x, u_init.row(k).transpose();
        cur_x = state_transition(state_and_control);
        x_init.row(k + 1) = cur_x.transpose();
    }
}

void controller_cost::set_reference_trajectory(const std::vector<general::TrajectoryPoint>& trajectory){
    ref_trajectory_ = trajectory;
    // 使用参考轨迹构建frenet坐标系
    frenet_frame_ = std::make_shared<general::FrenetFrame>(ref_trajectory_,false);
    // 每次重设参考轨迹后，重置初始运行标志
    is_first_run_ = true;
}

    void controller_cost::update(int iter, const Eigen::MatrixXd& x_and_u){
        // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
        // 对于ALM方法，更新乘子和惩罚系数：alm_mu_must_, alm_rho_
        // 对于障碍函数法，更新障碍系数：barrier_q1_, barrier_q2_
        iter_=iter;
        if(iter==0){
            // 第一轮迭代不更新乘子，只初始化为0
            alm_mu_must_.setZero();
            alm_rho_=controller_params_->alm_rho_init;
            barrier_q1_=controller_params_->barrier_q1_init;
            barrier_q2_=controller_params_->barrier_q2_init;
            return;
        }

        if(controller_params_->use_alm && !shut_down_constraints_){
            // 更新乘子，使用传入的轨迹,终端状态也有乘子，一并更新
            int N = x_and_u.rows() - 1;
            for(int k=0; k<=N; ++k){
                Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
                x_u_vec << x_and_u.row(k).transpose();
                auto mu_grad = compute_mu_gradient_one_step(x_u_vec, k);
                // mu_next = rho * derivative =rho * max(c + mu/rho, 0)=max(rho*c + mu, 0)，使得互补松弛天然满足
                alm_mu_must_.row(k) = (alm_rho_ * mu_grad.head(num_c_))
                    .cwiseMin(controller_params_->alm_mu_max * Eigen::VectorXd::Ones(num_c_));
            }
        }
    }

    Eigen::VectorXd controller_cost::state_transition(const Eigen::VectorXd& state_and_control){
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << state_and_control.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        auto next_state = f_(std::vector<casadi::DM>{z_row_dm});
        casadi::DM next_state_dm = next_state.at(0);
        Eigen::VectorXd next_state_vec = to_vector(next_state_dm);
        return next_state_vec;
    }

    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,有N+1个，最后一个不管，输出是雅可比矩阵堆成的张量
    Eigen::Tensor<double, 3,Eigen::ColMajor> controller_cost::state_total_jacobian(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows()-1;
        int nz = n_x_ + n_u_;
        Eigen::Tensor<double, 3,Eigen::ColMajor> jacobian_tensor(N, n_x_, nz);
        for(int i=0;i<N;++i){
            Eigen::RowVectorXd z_row(nz);
            z_row << x_and_u.row(i);
            casadi::DM z_row_dm = to_dm(z_row);
            auto jacobian_res = f_jacobian_(std::vector<casadi::DM>{z_row_dm});
            Eigen::MatrixXd jacobian_mat = to_matrix(jacobian_res[0]);
            // Copy to tensor
            for(int r=0; r<n_x_; ++r){
                for(int c=0; c<nz; ++c){
                    jacobian_tensor(i, r, c) = jacobian_mat(r, c);
                }
            }
        }
        return jacobian_tensor;
    }

    // 计算状态转移函数的Hessian张量，输入是z=[x;u]合并后的向量,输出是Hessian堆成的四维张量，输入有N+1个，最后一个不管
    Eigen::Tensor<double, 4,Eigen::ColMajor> controller_cost::state_total_hessian(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows()-1;
        int nz = n_x_ + n_u_;
        Eigen::Tensor<double, 4,Eigen::ColMajor> hessian_tensor(N, n_x_, nz, nz);
        for(int i=0;i<N;++i){
            Eigen::RowVectorXd z_row(nz);
            z_row << x_and_u.row(i);
            casadi::DM z_row_dm = to_dm(z_row);
            auto hessian_res = f_hessian_(std::vector<casadi::DM>{z_row_dm});
            for(int r=0;r<n_x_;++r){
                Eigen::MatrixXd h_mat = to_matrix(hessian_res[r]);
                // Copy to tensor
                for(int c1=0; c1<nz; ++c1){
                    for(int c2=0; c2<nz; ++c2){
                        hessian_tensor(i, r, c1, c2) = h_mat(c1, c2);
                    }
                }
            }
        }
        return hessian_tensor;
    }

    double controller_cost::compute_total_cost(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows() - 1;
        double total_cost = 0.0;
        origin_cost_=0.0;
        constraint_cost_=0.0;
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_cost += compute_one_step_cost(x_u_vec,Q_,R_,k);
        }
        // std::cout<<"origin cost: "<<origin_cost_<<", constraint cost: "<<constraint_cost_<<std::endl;
        Eigen::VectorXd x_N = x_and_u.row(N).head(n_x_).transpose();
        total_cost += compute_terminal_cost(x_N);
        return total_cost;
    }

    double controller_cost::compute_terminal_cost(const Eigen::VectorXd& x_N){
        Eigen::VectorXd z_N=Eigen::VectorXd(n_x_+n_u_);
        z_N<<x_N, Eigen::VectorXd::Zero(n_u_);
        return compute_one_step_cost(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), controller_params_->ilqr_params->N); // 终端代价不考虑控制量,设为0，不会影响结果
    }


    double controller_cost::compute_one_step_cost(const Eigen::VectorXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
        double total_cost = 0.0;
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        // 计算阶段代价
        casadi::DM Q_dm = to_dm(Q);
        casadi::DM R_dm = to_dm(R);
        auto ref_time = cur_t_ + step * controller_params_->ilqr_params->dt + controller_params_->ilqr_params->dt;  // 预测时刻
        auto matched = frenet_frame_->get_matched_trj_point(ref_time);
        auto matched_point = matched.first;
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,matched_point.v});
        auto res = l_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        total_cost+= static_cast<double>(res[0]);
        origin_cost_+=total_cost;
        if(!shut_down_constraints_){
        // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_);
            std::vector<casadi::DM> res_must_c;

            if(controller_params_->use_alm){
                Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
                casadi::DM mu_dm = to_dm(mu_vec);
                casadi::DM rho_dm(alm_rho_);
                res_must_c = alm_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, mu_dm, rho_dm});
            }
            else{
                casadi::DM barrier_q1_dm(barrier_q1_);
                casadi::DM barrier_q2_dm(barrier_q2_);
                res_must_c = barrier_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, barrier_q1_dm, barrier_q2_dm});
            }
            
            total_cost += static_cast<double>(res_must_c[0]);
            constraint_cost_+=static_cast<double>(res_must_c[0]);
        }

        return total_cost;
    }

    void controller_cost::get_total_jacobian_hessian(const Eigen::MatrixXd& x_and_u,Eigen::MatrixXd&total_jacbian,Eigen::Tensor<double, 3,Eigen::ColMajor>&total_hessian){
        // 传入的是从时刻0到N的所有状态和控制量，每一行是一个时间步的[x,u]，注意，有一个终端状态，但是不计算
        int N = x_and_u.rows() - 1;
        int nz = n_x_ + n_u_;
        origin_grad_norm_=0.0;
        constraint_grad_norm_=0.0;
        total_jacbian.setZero();
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_jacbian.row(k) = compute_gradient_one_step(x_u_vec,Q_,R_,k).transpose();
        }
        Eigen::VectorXd z_N=Eigen::VectorXd(n_x_+n_u_);
        z_N<<x_and_u.row(N).transpose();
        total_jacbian.row(N)=compute_gradient_one_step(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), controller_params_->ilqr_params->N);
        // std::cout<<"origin grad norm: "<<origin_grad_norm_<<", constraint grad norm: "<<constraint_grad_norm_<<std::endl;
         
        origin_hess_norm_=0.0;
        constraint_hess_norm_=0.0;
        total_hessian.setZero();
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            Eigen::MatrixXd hess_mat = compute_hessian_one_step(x_u_vec,Q_,R_,k);
            // Block assign the nz x nz Hessian matrix into the tensor slice (zero-copy)
            Eigen::TensorMap<Eigen::Tensor<const double, 2, Eigen::ColMajor>> src_tensor_map(hess_mat.data(), nz, nz);
            total_hessian.chip(k,0) = src_tensor_map;
        }
        auto terminal_hessian=compute_hessian_one_step(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), controller_params_->ilqr_params->N);
        Eigen::TensorMap<Eigen::Tensor<const double, 2, Eigen::ColMajor>> src_tensor_map(terminal_hessian.data(), nz, nz);
        total_hessian.chip(N,0) = src_tensor_map;
        // std::cout<<"origin hess norm: "<<origin_hess_norm_<<", constraint hess norm: "<<constraint_hess_norm_<<std::endl;
        
    }

    Eigen::VectorXd controller_cost::compute_mu_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(num_c_);
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        if(!shut_down_constraints_){
            // 输入输出{"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost"}
            // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_); 
            Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
            casadi::DM mu_dm = to_dm(mu_vec);
            casadi::DM rho_dm(alm_rho_);
            auto res_must_c = alm_mu_derivative_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, mu_dm, rho_dm});
            Eigen::VectorXd c_must_jac_mat = to_vector(res_must_c[0]);
            total_gradient.head(num_c_) += c_must_jac_mat;
        }

        return total_gradient;
    }


    Eigen::VectorXd controller_cost::compute_gradient_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(n_x_ + n_u_);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "barrier_rho"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = to_dm(Q);
        casadi::DM R_dm = to_dm(R);
        double ref_time = cur_t_ + step * controller_params_->ilqr_params->dt+controller_params_->ilqr_params->dt;  // 预测时刻
        auto matched = frenet_frame_->get_matched_trj_point(ref_time);
        auto matched_point=matched.first;
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,matched_point.v});
        auto res = l_jacobian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        Eigen::VectorXd l_jac_mat = to_vector(res[0]);
        total_gradient+= l_jac_mat;
        origin_grad_norm_ += l_jac_mat.norm();
        if(!shut_down_constraints_){
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_);
            std::vector<casadi::DM> res_must_c;
            if(controller_params_->use_alm){
                Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
                casadi::DM mu_dm = to_dm(mu_vec);
                casadi::DM rho_dm(alm_rho_);
                res_must_c = alm_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, mu_dm, rho_dm});
            }
            else{
                casadi::DM barrier_q1_dm(barrier_q1_);
                casadi::DM barrier_q2_dm(barrier_q2_);
                res_must_c = barrier_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, barrier_q1_dm, barrier_q2_dm});
            }
            Eigen::VectorXd c_must_jac_mat = to_vector(res_must_c[0]);
            total_gradient+= c_must_jac_mat;
            constraint_grad_norm_ += c_must_jac_mat.norm();
        }

        return total_gradient;
    }

    Eigen::MatrixXd controller_cost::compute_hessian_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
        Eigen::MatrixXd total_hessian = Eigen::MatrixXd::Zero(n_x_ + n_u_ ,n_x_ + n_u_);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_q1", "barrier_q2"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = to_dm(Q);
        casadi::DM R_dm = to_dm(R);
        // 参考点需要通过frenet坐标系转换得到匹配点
        double ref_time = cur_t_ + step * controller_params_->ilqr_params->dt+controller_params_->ilqr_params->dt;  // 预测时刻
        auto matched = frenet_frame_->get_matched_trj_point(ref_time);
        auto matched_point = matched.first;
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,matched_point.v});
        auto res = l_hessian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        Eigen::MatrixXd l_hes_mat = to_matrix(res[0]);
        total_hessian+= l_hes_mat;
        origin_hess_norm_ += l_hes_mat.norm();

        if(!shut_down_constraints_){
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "barrier_q1", "barrier_q2"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm = to_dm(upper_bound_);
        casadi::DM lower_bound_dm = to_dm(lower_bound_);
        std::vector<casadi::DM> res_must_c;
        if(controller_params_->use_alm){
            Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
            casadi::DM mu_dm = to_dm(mu_vec);
            casadi::DM rho_dm(alm_rho_);
            res_must_c = alm_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, mu_dm, rho_dm});
        }
        else{
            casadi::DM barrier_q1_dm(barrier_q1_);
            casadi::DM barrier_q2_dm(barrier_q2_);
            res_must_c = barrier_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, barrier_q1_dm, barrier_q2_dm});
        }
            Eigen::MatrixXd c_must_hes_mat = to_matrix(res_must_c[0]);
            total_hessian+= c_must_hes_mat;
            constraint_hess_norm_ += c_must_hes_mat.norm();
        }

        return total_hessian;
    }

    void controller_cost::build_functions(){
        build_dynamics_function();
        build_origin_cost_function();
        build_one_step_inequal_cost_function_without_obstacle();
    }

    void controller_cost::build_dynamics_function(){
        casadi::SX z = casadi::SX::sym("z", n_x_+n_u_);
        casadi::SX state= z(casadi::Slice(0, (int)n_x_));
        casadi::SX control= z(casadi::Slice((int)n_x_, (int)(n_x_+n_u_)));
        // 使用运动学模型，控制量：[steering_angle，a],状态量：[x, y,heading,v] 
        casadi::SX x = state(0);
        casadi::SX y = state(1);
        casadi::SX heading = state(2);
        casadi::SX v = state(3); 
        casadi::SX delta = control(0);
        casadi::SX a = control(1);
        casadi::SX dt = get_params()->dt;
        // 以车辆重心为参考点的运动学模型
        // 引入侧偏角 β
        casadi::SX beta = casadi::SX::atan(casadi::SX::tan(delta) / 2);

        // 状态更新（重心模型）
        casadi::SX x_next = x + v * casadi::SX::cos(heading + beta) * dt;
        casadi::SX y_next = y + v * casadi::SX::sin(heading + beta) * dt;
        casadi::SX heading_next = heading + 2 * v * casadi::SX::sin(beta) / (vehicle_params_.lf + vehicle_params_.lr) * dt;
        casadi::SX v_next = v + a * dt;

        casadi::SX next_state = casadi::SX::vertcat({x_next, y_next, heading_next, v_next});
        f_ = casadi::Function("dynamics_function", {z}, {next_state}, {"state+control"}, {"next_state"});
        casadi::SX f_jacobian = casadi::SX::jacobian(next_state, z);
        f_jacobian_ = casadi::Function("dynamics_jacobian", {z}, {f_jacobian}, {"state+control"}, {"dynamics_jacobian"});

        casadi::SXVector hess_vec;
        std::vector<std::string> hess_names;
        for(int i = 0; i < n_x_; ++i){
            casadi::SX grad_i = casadi::SX::gradient(next_state(i), z);
            casadi::SX hess_i = casadi::SX::jacobian(grad_i, z);
            hess_vec.push_back(hess_i);
            hess_names.push_back("hess_" + std::to_string(i));
        }
        f_hessian_ = casadi::Function("dynamics_hessian", {z}, hess_vec, {"state+control"}, hess_names);
    }

    void controller_cost::build_origin_cost_function(){
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v],[steering_angle, a]
        casadi::SX control = z(casadi::Slice(n_x_, n_x_ + n_u_));
        casadi::SX Q = casadi::SX::sym("Q", n_x_, n_x_);
        casadi::SX R = casadi::SX::sym("R", n_u_, n_u_);
        // 注意，参考状态需要通过frenet坐标系转换得到匹配点
        casadi::SX ref = casadi::SX::sym("ref", n_x_);
        casadi::SX state_error = state - ref;
        casadi::SX stage_cost = 0.5 * casadi::SX::mtimes({state_error.T(), Q, state_error}) + 0.5 * casadi::SX::mtimes({control.T(), R, control});
        l_ = casadi::Function("stage_cost_function", {z, Q, R, ref}, {stage_cost}, {"state+control", "Q", "R", "ref"}, {"stage_cost"});

        // 阶段代价梯度
        casadi::SX l_jacobian = casadi::SX::gradient(stage_cost, z);
        l_jacobian_ = casadi::Function("stage_cost_gradient", {z, Q, R, ref}, {l_jacobian}, {"state+control", "Q", "R", "ref"}, {"stage_l_jacobian"});

        // 阶段代价Hessian
        casadi::SX l_hessian = casadi::SX::jacobian(l_jacobian, z);
        l_hessian_ = casadi::Function("stage_cost_hessian", {z, Q, R, ref}, {l_hessian}, {"state+control", "Q", "R", "ref"}, {"stage_l_hessian"});
    }

    void controller_cost::build_one_step_inequal_cost_function_without_obstacle(){
        // 函数输入是速度，加速度，方向盘转角，道路上下界和参考点，通过计算与参考点的距离来判断是否超出道路边界，需要分清楚是上边界还是下边界
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v],[steering_angle, a]
        casadi::SX control = z(casadi::Slice(n_x_, n_x_ + n_u_));
        // 做成三组向量输入，上界，下界，参考点
        casadi::SX upper_bound = casadi::SX::sym("upper_bound",3);// [a_max, v_max, steer_max]
        casadi::SX lower_bound = casadi::SX::sym("lower_bound",3);// [a_min, v_min, steer_min]

        casadi::SX a_max=upper_bound(0);
        casadi::SX a_min=lower_bound(0);
        casadi::SX v_max=upper_bound(1);
        casadi::SX v_min=lower_bound(1);
        casadi::SX steer_max=upper_bound(2);
        casadi::SX steer_min=lower_bound(2);

        // 速度边界代价 (速度是状态变量，不是控制变量)
        casadi::SX v_low_cost = v_min - state(3); //满足约束时小于等于0，v>v_min
        casadi::SX v_high_cost = state(3) - v_max; //v<v_max
        // 加速度边界代价 
        casadi::SX a_low_cost =  a_min - control(1);
        casadi::SX a_high_cost = control(1) - a_max;
        // 方向盘转角边界代价
        casadi::SX steer_low_cost =  steer_min - control(0);
        casadi::SX steer_high_cost = control(0) - steer_max;
        // 现将约束堆叠成向量输出，方便矩阵运算
        casadi::SX ineq_constraints = casadi::SX::vertcat({a_low_cost, a_high_cost,
                                                         v_low_cost, v_high_cost, steer_low_cost, steer_high_cost});

        // 分别构造ALM和Barrier的代价函数
        // alm还需要乘子和惩罚系数作为输入
        casadi::SX mu = casadi::SX::sym("mu", 6); // 6个不等式约束
        casadi::SX rho = casadi::SX::sym("rho");
        // 乘子项：lambda^T * g(x)
        casadi::SX multiplier_term = casadi::SX::dot(mu, ineq_constraints);
        // 不等式约束乘子更新公式 mu_next = pho*max(ineq_constraints + mu/rho, 0),少了个rho，由外部更新时加上
        casadi::SX mu_next=casadi::SX::fmax(ineq_constraints+mu/rho, 0);
        // 惩罚项：(rho/2) * ||max(0, g(x))||^2
        casadi::SX penalty_term = 0.5 * rho * casadi::SX::sum1(casadi::SX::pow(mu_next, 2));
        // 完整ALM成本
        casadi::SX alm_cost = multiplier_term + penalty_term;
        // casadi::SX alm_cost = penalty_term;

        // 函数定义
        alm_cost_ = casadi::Function("alm_cost_function", {z, upper_bound, lower_bound, mu, rho}, {alm_cost},
                                    {"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost"});
        // jacobian
        casadi::SX alm_cost_jacobian = casadi::SX::gradient(alm_cost, z);
        alm_cost_jacobian_ = casadi::Function("alm_cost_jacobian", {z, upper_bound, lower_bound, mu, rho}, {alm_cost_jacobian},
                                             {"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost_jacobian"});
        // hessian
        casadi::SX alm_cost_hessian = casadi::SX::jacobian(alm_cost_jacobian, z);
        alm_cost_hessian_ = casadi::Function("alm_cost_hessian", {z, upper_bound, lower_bound, mu, rho}, {alm_cost_hessian},
                                            {"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_cost_hessian"});
        // 另外，定义一个函数计算乘子导数，用于更新乘子
        // 注意，ALM法乘子更新公式为mu_next=pho_*max(mu/rho + c,0)，
        // 所以直接输出max(mu/rho + c,0),然后外部乘以pho_
        alm_mu_derivative_ = casadi::Function("alm_mu_derivative_function", {z, upper_bound, lower_bound, mu, rho}, {mu_next},
                                             {"state+control", "upper_bound", "lower_bound", "mu", "rho"}, {"alm_mu_derivative"});
        
        // barrier代价函数,不需要乘子,而且系数是常量，公式为 sum(q1*exp(q2*c_i))
        casadi::SX barrier_q1 = casadi::SX::sym("barrier_q1");
        casadi::SX barrier_q2 = casadi::SX::sym("barrier_q2");
        casadi::SX barrier_cost =casadi::SX::sum1((barrier_q1) *casadi::SX::exp(barrier_q2 * ineq_constraints));
        barrier_cost_ = casadi::Function("barrier_cost_function", {z, upper_bound, lower_bound, barrier_q1, barrier_q2}, {barrier_cost},
                                        {"state+control", "upper_bound", "lower_bound", "barrier_q1", "barrier_q2"}, {"barrier_cost"});
        // barrier jacobian
        casadi::SX barrier_cost_jacobian = casadi::SX::gradient(barrier_cost, z);
        barrier_cost_jacobian_ = casadi::Function("barrier_cost_jacobian", {z, upper_bound, lower_bound, barrier_q1, barrier_q2}, {barrier_cost_jacobian},
                                                {"state+control", "upper_bound", "lower_bound", "barrier_q1", "barrier_q2"}, {"barrier_cost_jacobian"});
        // barrier hessian
        casadi::SX barrier_cost_hessian = casadi::SX::jacobian(barrier_cost_jacobian, z);
        barrier_cost_hessian_ = casadi::Function("barrier_cost_hessian", {z, upper_bound, lower_bound, barrier_q1, barrier_q2}, {barrier_cost_hessian},
                                               {"state+control", "upper_bound", "lower_bound", "barrier_q1", "barrier_q2"}, {"barrier_cost_hessian"});
    }

}}