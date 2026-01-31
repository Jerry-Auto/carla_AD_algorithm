#include "cilqr_planner/cost_define.h"
#include <iostream>
namespace AD_algorithm {
namespace planner {

cilqr_problem::cilqr_problem(const std::shared_ptr<CILQRPlannerparams>& params)
    : planner_cost(params, params->ilqr_params->nx, params->ilqr_params->nu) {
    alm_rho_=planner_params_->alm_rho_init;
    alm_obstacle_rho_=planner_params_->obstacle_alm_rho_init;
    barrier_q1_=planner_params_->barrier_q1_init;
    barrier_q2_=planner_params_->barrier_q2_init;
    barrier_obstacle_q1_=planner_params_->obstacle_barrier_q1_init;
    barrier_obstacle_q2_=planner_params_->obstacle_barrier_q2_init;
    if(planner_params_->for_control){
        shut_down_obstacles_=true;   //用于控制时关闭障碍物相关约束
        shut_down_constraints_=false;   //用于控制时可以考虑一些约束，基本不会影响计算速度
    }
    else{
        shut_down_obstacles_=false;   //用于规划时开启障碍物相关约束
        shut_down_constraints_=false;   //用于规划时需要考虑约束
    }
    alm_mu_must_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N+1, num_c_); // 每个时间步的所有不等式约束乘子，注意，终端状态也有约束
    build_functions();
}

void cilqr_problem::set_obstacles(const std::vector<std::vector<general::Obstacle>>& obstacles){
    obs_preds_ = obstacles;
    num_obstacles_ = static_cast<int>(obs_preds_.size());
    if (planner_params_->for_control) {
        shut_down_obstacles_ = true;
    } else {
        shut_down_obstacles_ = obs_preds_.empty();
    }
    if(num_obstacles_==0){
        return;
    }
    alm_mu_obstacles_ = Eigen::MatrixXd::Zero(planner_params_->ilqr_params->N+1, num_obstacles_ * 2); // 每个障碍物2个不等式约束
}


    void cilqr_problem::update(int iter, const Eigen::MatrixXd& x_and_u){
        // 更新乘子和障碍系数等内部状态，在新一轮迭代开始前调用
        // 对于ALM方法，更新乘子和惩罚系数,分别是：
        // alm_mu_must_, alm_mu_obstacles_, alm_rho_,alm_obstacle_rho_
        // 对于障碍函数法，更新障碍系数,分别是：    
        // barrier_q1_, barrier_q2_，barrier_obstacle_q1_, barrier_obstacle_q2_
        iter_=iter;
        if(iter==0){
            // 第一轮迭代不更新乘子，只初始化为0
            alm_mu_must_.setZero();
            alm_mu_obstacles_.setZero();
            alm_rho_=planner_params_->alm_rho_init;
            alm_obstacle_rho_=planner_params_->obstacle_alm_rho_init;
            barrier_q1_=planner_params_->barrier_q1_init;
            barrier_q2_=planner_params_->barrier_q2_init;
            barrier_obstacle_q1_=planner_params_->obstacle_barrier_q1_init;
            barrier_obstacle_q2_=planner_params_->obstacle_barrier_q2_init;
            return;
        }

        if(planner_params_->use_alm){
            // 更新乘子，使用传入的轨迹,终端状态也有乘子，一并更新
            int N = x_and_u.rows() - 1;
            for(int k=0; k<=N; ++k){
                Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
                x_u_vec << x_and_u.row(k).transpose();
                auto mu_grad = compute_mu_gradient_one_step(x_u_vec, k);
                // mu_next = rho * derivative =rho * max(c + mu/rho, 0)=max(rho*c + mu, 0)，使得互补松弛天然满足
                if(!shut_down_constraints_){
                    alm_mu_must_.row(k) = (alm_rho_ * mu_grad.head(num_c_)).cwiseMin(planner_params_->alm_mu_max * Eigen::VectorXd::Ones(num_c_));
                }
                if(!shut_down_obstacles_ && num_obstacles_ > 0){
                    alm_mu_obstacles_.row(k) = (alm_obstacle_rho_ * mu_grad.tail(num_obstacles_ * 2)).cwiseMin(planner_params_->obstacle_alm_mu_max * Eigen::VectorXd::Ones(num_obstacles_ * 2));
                }
            }
        }

        // 更新障碍系数
        if(planner_params_->use_alm){
            alm_rho_ = std::min(alm_rho_ * planner_params_->alm_beta, planner_params_->alm_rho_max);
            alm_obstacle_rho_ = std::min(alm_obstacle_rho_ * planner_params_->obstacle_alm_rho_beta, planner_params_->obstacle_alm_rho_max);
        }else{
            barrier_q1_ = std::min(barrier_q1_ * planner_params_->barrier_beta1, planner_params_->barrier_q1_max);
            barrier_q2_ = std::min(barrier_q2_ * planner_params_->barrier_beta2, planner_params_->barrier_q2_max);
            barrier_obstacle_q1_ = std::min(barrier_obstacle_q1_ * planner_params_->obstacle_barrier_beta1, planner_params_->obstacle_barrier_q1_max);
            barrier_obstacle_q2_ = std::min(barrier_obstacle_q2_ * planner_params_->obstacle_barrier_beta2, planner_params_->obstacle_barrier_q2_max);
        }
    }

    Eigen::VectorXd cilqr_problem::state_transition(const Eigen::VectorXd& state_and_control){
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << state_and_control.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        auto next_state = f_(std::vector<casadi::DM>{z_row_dm});
        casadi::DM next_state_dm = next_state.at(0);
        Eigen::VectorXd next_state_vec = to_vector(next_state_dm);
        return next_state_vec;
    }

    // 计算状态转移函数的雅可比矩阵，输入是z=[x;u]合并后的向量,有N+1个，最后一个不管，输出是雅可比矩阵堆成的张量
    Eigen::Tensor<double, 3,Eigen::ColMajor> cilqr_problem::state_total_jacobian(const Eigen::MatrixXd& x_and_u){
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
    Eigen::Tensor<double, 4,Eigen::ColMajor> cilqr_problem::state_total_hessian(const Eigen::MatrixXd& x_and_u){
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

    double cilqr_problem::compute_total_cost(const Eigen::MatrixXd& x_and_u){
        int N = x_and_u.rows() - 1;
        double total_cost = 0.0;
        origin_cost_=0.0;
        obstacle_cost_=0.0;
        constraint_cost_=0.0;
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_cost += compute_one_step_cost(x_u_vec,Q_,R_,k);
        }
        std::cout<<"origin cost: "<<origin_cost_<<", obstacle cost: "<<obstacle_cost_<<", constraint cost: "<<constraint_cost_<<std::endl;
        Eigen::VectorXd x_N = x_and_u.row(N).head(n_x_).transpose();
        total_cost += compute_terminal_cost(x_N);
        return total_cost;
    }

    double cilqr_problem::compute_terminal_cost(const Eigen::VectorXd& x_N){
        Eigen::VectorXd z_N=Eigen::VectorXd(n_x_+n_u_);
        z_N<<x_N, Eigen::VectorXd::Zero(n_u_);
        return compute_one_step_cost(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), planner_params_->ilqr_params->N); // 终端代价不考虑控制量,设为0，不会影响结果
    }


    double cilqr_problem::compute_one_step_cost(const Eigen::VectorXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
        double total_cost = 0.0;
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        // 计算阶段代价
        casadi::DM Q_dm = to_dm(Q);
        casadi::DM R_dm = to_dm(R);
        auto matched_point = frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,ref_velo_});
        auto res = l_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        total_cost+= static_cast<double>(res[0]);
        origin_cost_+=total_cost;
        if(!shut_down_constraints_){
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_);
            casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
            std::vector<casadi::DM> res_must_c;

            if(planner_params_->use_alm){
                Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
                casadi::DM mu_dm = to_dm(mu_vec);
                casadi::DM rho_dm(alm_rho_);
                res_must_c = alm_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
            }
            else{
                casadi::DM barrier_q1_dm(barrier_q1_);
                casadi::DM barrier_q2_dm(barrier_q2_);
                res_must_c = barrier_cost_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_q1_dm, barrier_q2_dm});
            }
            
            total_cost += static_cast<double>(res_must_c[0]);
            constraint_cost_+=static_cast<double>(res_must_c[0]);
        }

        // 障碍物代价
        double obstacle_cost=0.0;
        if(!shut_down_obstacles_&&num_obstacles_>0){
            for(int i=0;i<num_obstacles_;++i){
                const auto& obs = obs_preds_[i][step];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    Eigen::VectorXd mu_vec = alm_mu_obstacles_.row(step).segment(i*2, 2);
                    casadi::DM mu_dm = to_dm(mu_vec);
                    casadi::DM obs_rho_dm(alm_obstacle_rho_);
                    res_obstacle = alm_obstacle_cost_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, obs_rho_dm});
                }
                else{
                    casadi::DM barrier_obs_q1_dm(barrier_obstacle_q1_);
                    casadi::DM barrier_obs_q2_dm(barrier_obstacle_q2_);
                    res_obstacle = barrier_obstacle_cost_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_obs_q1_dm, barrier_obs_q2_dm});
                }
                obstacle_cost += static_cast<double>(res_obstacle[0]);
            }
        }
        obstacle_cost_+=obstacle_cost;
        total_cost += obstacle_cost;
        return total_cost;
    }

    void cilqr_problem::get_total_jacobian_hessian(const Eigen::MatrixXd& x_and_u,Eigen::MatrixXd&total_jacbian,Eigen::Tensor<double, 3,Eigen::ColMajor>&total_hessian){
        // 传入的是从时刻0到N的所有状态和控制量，每一行是一个时间步的[x,u]，注意，有一个终端状态，但是不计算
        int N = x_and_u.rows() - 1;
        int nz = n_x_ + n_u_;
        origin_grad_norm_=0.0;
        obstacle_grad_norm_=0.0;
        constraint_grad_norm_=0.0;
        total_jacbian.setZero();
        for(int k=0;k<N;++k){
            Eigen::VectorXd x_u_vec = Eigen::VectorXd(n_x_ + n_u_);
            x_u_vec << x_and_u.row(k).transpose();
            total_jacbian.row(k) = compute_gradient_one_step(x_u_vec,Q_,R_,k).transpose();
        }
        Eigen::VectorXd z_N=Eigen::VectorXd(n_x_+n_u_);
        z_N<<x_and_u.row(N).transpose();
        total_jacbian.row(N)=compute_gradient_one_step(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), planner_params_->ilqr_params->N);
        std::cout<<"origin grad norm: "<<origin_grad_norm_<<", obstacle grad norm: "<<obstacle_grad_norm_<<", constraint grad norm: "<<constraint_grad_norm_<<std::endl;
         
        origin_hess_norm_=0.0;
        obstacle_hess_norm_=0.0;
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
        auto terminal_hessian=compute_hessian_one_step(z_N, Qf_,Eigen::MatrixXd::Zero(n_u_, n_u_), planner_params_->ilqr_params->N);
        Eigen::TensorMap<Eigen::Tensor<const double, 2, Eigen::ColMajor>> src_tensor_map(terminal_hessian.data(), nz, nz);
        total_hessian.chip(N,0) = src_tensor_map;
        std::cout<<"origin hess norm: "<<origin_hess_norm_<<", obstacle hess norm: "<<obstacle_hess_norm_<<", constraint hess norm: "<<constraint_hess_norm_<<std::endl;
        
    }

    Eigen::VectorXd cilqr_problem::compute_mu_gradient_one_step(const Eigen::MatrixXd& x_and_u,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(num_c_ + num_obstacles_ * 2);
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm = to_dm(z_row);
        if(!shut_down_constraints_){
            auto matched_point = frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
            // 计算必有的不等式约束代价
            // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_); 
            casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
            Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
            casadi::DM mu_dm = to_dm(mu_vec);
            casadi::DM rho_dm(alm_rho_);
            auto res_must_c = alm_mu_derivative_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
            Eigen::VectorXd c_must_jac_mat = to_vector(res_must_c[0]);
            total_gradient.head(num_c_) += c_must_jac_mat;
        }

        // 障碍物代价
        if(!shut_down_obstacles_&&num_obstacles_>0){
            for(int i=0;i<num_obstacles_;++i){
                const auto& obs = obs_preds_[i][step];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                Eigen::VectorXd mu_vec = alm_mu_obstacles_.row(step).segment(i*2, 2);
                casadi::DM mu_dm_obs = to_dm(mu_vec);
                casadi::DM rho_dm_obs(alm_obstacle_rho_);
                auto res_obstacle = alm_obstacle_mu_derivative_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm_obs, rho_dm_obs});
                Eigen::VectorXd c_obs_jac_mat = to_vector(res_obstacle[0]);
                total_gradient.segment(num_c_ + i * 2, 2) += c_obs_jac_mat;
            }
        }
        return total_gradient;
    }


    Eigen::VectorXd cilqr_problem::compute_gradient_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
        Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(n_x_ + n_u_);
        // 输入输出
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        Eigen::RowVectorXd z_row(n_x_ + n_u_);
        z_row << x_and_u.transpose();
        casadi::DM z_row_dm(std::vector<double>(z_row.data(), z_row.data() + z_row.size()));
        // 计算阶段代价
        casadi::DM Q_dm = to_dm(Q);
        casadi::DM R_dm = to_dm(R);
        auto matched_point = frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,ref_velo_});
        auto res = l_jacobian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        Eigen::VectorXd l_jac_mat = to_vector(res[0]);
        total_gradient+= l_jac_mat;
        origin_grad_norm_ += l_jac_mat.norm();
        if(!shut_down_constraints_){
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_rho"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
            casadi::DM upper_bound_dm = to_dm(upper_bound_);
            casadi::DM lower_bound_dm = to_dm(lower_bound_);
            casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
            std::vector<casadi::DM> res_must_c;
            if(planner_params_->use_alm){
                Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
                casadi::DM mu_dm = to_dm(mu_vec);
                casadi::DM rho_dm(alm_rho_);
                res_must_c = alm_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
            }
            else{
                casadi::DM barrier_q1_dm(barrier_q1_);
                casadi::DM barrier_q2_dm(barrier_q2_);
                res_must_c = barrier_cost_jacobian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_q1_dm, barrier_q2_dm});
            }
            Eigen::VectorXd c_must_jac_mat = to_vector(res_must_c[0]);
            total_gradient+= c_must_jac_mat;
            constraint_grad_norm_ += c_must_jac_mat.norm();
        }

        // 障碍物代价
        if(!shut_down_obstacles_&&num_obstacles_>0){
            for(int i=0;i<num_obstacles_;++i){
                const auto& obs = obs_preds_[i][step];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    Eigen::VectorXd mu_vec = alm_mu_obstacles_.row(step).segment(i*2, 2);
                    casadi::DM mu_dm = to_dm(mu_vec);
                    casadi::DM rho_dm(alm_obstacle_rho_);
                    res_obstacle = alm_obstacle_jacobian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                }
                else{
                    casadi::DM barrier_obs_q1_dm(barrier_obstacle_q1_);
                    casadi::DM barrier_obs_q2_dm(barrier_obstacle_q2_);
                    res_obstacle = barrier_obstacle_jacobian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_obs_q1_dm, barrier_obs_q2_dm});
                }
                Eigen::VectorXd c_obs_jac_mat = to_vector(res_obstacle[0]);
                total_gradient+= c_obs_jac_mat;
                obstacle_grad_norm_ += c_obs_jac_mat.norm();
            }
        }
        return total_gradient;
    }

    Eigen::MatrixXd cilqr_problem::compute_hessian_one_step(const Eigen::MatrixXd& x_and_u,const Eigen::MatrixXd& Q,const Eigen::MatrixXd& R,int step){
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
        auto matched_point = frenet_frame_->get_matched_point(double(x_and_u(0)),double(x_and_u(1)),double(x_and_u(2)));
        casadi::DM ref_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading,ref_velo_});
        auto res = l_hessian_(std::vector<casadi::DM>{z_row_dm, Q_dm, R_dm, ref_dm});
        Eigen::MatrixXd l_hes_mat = to_matrix(res[0]);
        total_hessian+= l_hes_mat;
        origin_hess_norm_ += l_hes_mat.norm();

        if(!shut_down_constraints_){
        // 计算必有的不等式约束代价
        // 输入输出{"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"}
        // {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_q1", "barrier_q2"}, {"barrier_cost"}
        // 上下界由每次迭代外部传入
        casadi::DM upper_bound_dm = to_dm(upper_bound_);
        casadi::DM lower_bound_dm = to_dm(lower_bound_);
        casadi::DM reference_point_dm(std::vector<double>{matched_point.x,matched_point.y,matched_point.heading});
        std::vector<casadi::DM> res_must_c;
        if(planner_params_->use_alm){
            Eigen::VectorXd mu_vec = alm_mu_must_.row(step);
            casadi::DM mu_dm = to_dm(mu_vec);
            casadi::DM rho_dm(alm_rho_);
            res_must_c = alm_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, mu_dm, rho_dm});
        }
        else{
            casadi::DM barrier_q1_dm(barrier_q1_);
            casadi::DM barrier_q2_dm(barrier_q2_);
            res_must_c = barrier_cost_hessian_(std::vector<casadi::DM>{z_row_dm, upper_bound_dm, lower_bound_dm, reference_point_dm, barrier_q1_dm, barrier_q2_dm});
        }
            Eigen::MatrixXd c_must_hes_mat = to_matrix(res_must_c[0]);
            total_hessian+= c_must_hes_mat;
            constraint_hess_norm_ += c_must_hes_mat.norm();
        }

        // 障碍物代价
        if(!shut_down_obstacles_&&num_obstacles_>0){
            for(int i=0;i<num_obstacles_;++i){
                const auto& obs = obs_preds_[i][step];
                casadi::DM obs_params_dm(std::vector<double>{obs.x, obs.y, obs.width, obs.length, obs.heading});
                std::vector<casadi::DM> res_obstacle;
                if(planner_params_->use_alm){
                    Eigen::VectorXd mu_vec = alm_mu_obstacles_.row(step).segment(i*2, 2);
                    casadi::DM mu_dm = to_dm(mu_vec);
                    casadi::DM rho_dm(alm_obstacle_rho_);
                    res_obstacle = alm_obstacle_hessian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, mu_dm, rho_dm});
                }
                else{
                    casadi::DM barrier_obs_q1_dm(barrier_obstacle_q1_);
                    casadi::DM barrier_obs_q2_dm(barrier_obstacle_q2_);
                    res_obstacle = barrier_obstacle_hessian_(std::vector<casadi::DM>{z_row_dm, obs_params_dm, barrier_obs_q1_dm, barrier_obs_q2_dm});
                }
                Eigen::MatrixXd c_obs_hes_mat = to_matrix(res_obstacle[0]);
                total_hessian+= c_obs_hes_mat;
                obstacle_hess_norm_ += c_obs_hes_mat.norm();
            }
        }
        return total_hessian;
    }

    void cilqr_problem::build_functions(){
        build_dynamics_function();
        build_origin_cost_function();
        build_one_step_inequal_cost_function_without_obstacle();
        build_one_step_obstacle_cost_functions();
    }

    void cilqr_problem::build_dynamics_function(){
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


    void cilqr_problem::build_origin_cost_function(){
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

    void cilqr_problem::build_one_step_inequal_cost_function_without_obstacle(){
        // 函数输入是速度，加速度，方向盘转角，道路上下界和参考点，通过计算与参考点的距离来判断是否超出道路边界，需要分清楚是上边界还是下边界
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v],[steering_angle, a]
        casadi::SX control = z(casadi::Slice(n_x_, n_x_ + n_u_));
        // 做成三组向量输入，上界，下界，参考点
        casadi::SX upper_bound = casadi::SX::sym("upper_bound",4);// [road_upper_bound, a_max, v_max, steer_max]
        casadi::SX lower_bound = casadi::SX::sym("lower_bound",4);// [road_lower_bound, a_min, v_min, steer_min]
        casadi::SX reference_point = casadi::SX::sym("reference_point", 3); // [x_ref, y_ref, heading_ref]

        casadi::SX road_upper_bound = upper_bound(0);
        casadi::SX road_lower_bound = lower_bound(0);
        casadi::SX a_max=upper_bound(1);
        casadi::SX a_min=lower_bound(1);
        casadi::SX v_max=upper_bound(2);
        casadi::SX v_min=lower_bound(2);
        casadi::SX steer_max=upper_bound(3);
        casadi::SX steer_min=lower_bound(3);

        // 参考点在外部计算好传入，这里直接计算与参考点的距离，要有方向，判断是上边界还是下边界
        casadi::SX dx = state(0) - reference_point(0);
        casadi::SX dy = state(1) - reference_point(1);
        casadi::SX heading_ref = reference_point(2);
        // 左侧为正，右侧为负
        // 首先计算参考线方向的法向量，然后点乘得到横向误差，相当于投影到法向量上，得到带符号的横向误差
        casadi::SX lateral_error = -dx * casadi::SX::sin(heading_ref) + dy * casadi::SX::cos(heading_ref);
        // road_lower_bound < lateral_error < road_upper_bound，就是两个不等式约束
        // road_lower_bound - lateral_error <= 0  和 lateral_error - road_upper_bound <= 0
        casadi::SX road_lower_cost = road_lower_bound - lateral_error; // 这里只返回一个约束的值，外部根据这个值计算乘子相关的代价
        casadi::SX road_upper_cost = lateral_error - road_upper_bound;

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
        casadi::SX ineq_constraints = casadi::SX::vertcat({road_lower_cost, road_upper_cost, a_low_cost, a_high_cost,
                                                         v_low_cost, v_high_cost, steer_low_cost, steer_high_cost});

        // 分别构造ALM和Barrier的代价函数
        // alm还需要乘子和惩罚系数作为输入
        casadi::SX mu = casadi::SX::sym("mu", 8); // 8个不等式约束
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
        alm_cost_ = casadi::Function("alm_cost_function", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost},
                                    {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost"});
        // jacobian
        casadi::SX alm_cost_jacobian = casadi::SX::gradient(alm_cost, z);
        alm_cost_jacobian_ = casadi::Function("alm_cost_jacobian", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost_jacobian},
                                             {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost_jacobian"});
        // hessian
        casadi::SX alm_cost_hessian = casadi::SX::jacobian(alm_cost_jacobian, z);
        alm_cost_hessian_ = casadi::Function("alm_cost_hessian", {z, upper_bound, lower_bound, reference_point, mu, rho}, {alm_cost_hessian},
                                            {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_cost_hessian"});
        // 另外，定义一个函数计算乘子导数，用于更新乘子
        // 注意，ALM法乘子更新公式为mu_next=pho_*max(mu/rho + c,0)，
        // 所以直接输出max(mu/rho + c,0),然后外部乘以pho_
        alm_mu_derivative_ = casadi::Function("alm_mu_derivative_function", {z, upper_bound, lower_bound, reference_point, mu, rho}, {mu_next},
                                             {"state+control", "upper_bound", "lower_bound", "reference_point", "mu", "rho"}, {"alm_mu_derivative"});
        
        // barrier代价函数,不需要乘子,而且系数是常量，公式为 sum(q1*exp(q2*c_i))
        casadi::SX barrier_q1 = casadi::SX::sym("barrier_q1");
        casadi::SX barrier_q2 = casadi::SX::sym("barrier_q2");
        casadi::SX barrier_cost =casadi::SX::sum1((barrier_q1) *casadi::SX::exp(barrier_q2 * ineq_constraints));
        barrier_cost_ = casadi::Function("barrier_cost_function", {z, upper_bound, lower_bound, reference_point, barrier_q1, barrier_q2}, {barrier_cost},
                                        {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_q1", "barrier_q2"}, {"barrier_cost"});
        // barrier jacobian
        casadi::SX barrier_cost_jacobian = casadi::SX::gradient(barrier_cost, z);
        barrier_cost_jacobian_ = casadi::Function("barrier_cost_jacobian", {z, upper_bound, lower_bound, reference_point, barrier_q1, barrier_q2}, {barrier_cost_jacobian},
                                                {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_q1", "barrier_q2"}, {"barrier_cost_jacobian"});
        // barrier hessian
        casadi::SX barrier_cost_hessian = casadi::SX::jacobian(barrier_cost_jacobian, z);
        barrier_cost_hessian_ = casadi::Function("barrier_cost_hessian", {z, upper_bound, lower_bound, reference_point, barrier_q1, barrier_q2}, {barrier_cost_hessian},
                                               {"state+control", "upper_bound", "lower_bound", "reference_point", "barrier_q1", "barrier_q2"}, {"barrier_cost_hessian"});
    }

    void cilqr_problem::build_one_step_obstacle_cost_functions(){
        // 定义障碍物代价，使用椭圆裕度算法
        // 输入是自车状态，障碍物位置和尺寸，先构建不等式约束函数c(z)，然后基于c(z)构建alm和barrier代价函数
        // 首先是椭圆裕度，假设障碍物是椭圆形的，以障碍物中心为原点，x轴和y轴方向的半轴长度分别为a和b，则椭圆的边界由方程(x/a)^2+(y/b)^2=1描述
        // 然后自车的前后轴中心点分别为(x_f, y_f)和(x_r, y_r），使用旋转矩阵将全局坐标系下的点转换到障碍物椭圆坐标系下，长轴方向为x轴，短轴方向为y轴
        // 转换后的点为(x_f', y_f')和(x_r', y_r')，代价要定义为越小越好，即c<=0表示满足约束，因此定义不等式约束函数为：
        // c_f = 1 - (x_f'/a)^2 + (y_f'/b)^2 <= 0,表示前轴点在椭圆外部
        // c_r = 1 - (x_r'/a)^2 + (y_r'/b)^2  <= 0
        // 其中(x_f', y_f') = R_theta * (x_f - x_obs, y_f - y_obs)，R_theta是旋转矩阵，theta是障碍物的朝向角
        casadi::SX z = casadi::SX::sym("z", n_x_ + n_u_);
        casadi::SX state = z(casadi::Slice(0, n_x_));
        // [x,y,heading,v]
        casadi::SX x = state(0);
        casadi::SX y = state(1);
        casadi::SX heading = state(2);
        casadi::SX v = state(3);
        // 自车前后轴中心点位置
        casadi::SX x_f = x + 0.5 * (vehicle_params_.lf + vehicle_params_.lr) * casadi::SX::cos(heading);  // 重心 + 0.5 * wheelbase
        casadi::SX y_f = y + 0.5 * (vehicle_params_.lf + vehicle_params_.lr) * casadi::SX::sin(heading);
        casadi::SX x_r = x - 0.5 * (vehicle_params_.lf + vehicle_params_.lr) * casadi::SX::cos(heading);
        casadi::SX y_r = y - 0.5 * (vehicle_params_.lf + vehicle_params_.lr) * casadi::SX::sin(heading);
        // 障碍物参数输入
        casadi::SX obs_params = casadi::SX::sym("obs_params", 5); // [x_obs, y_obs, width, length, theta]
        casadi::SX x_obs = obs_params(0);
        casadi::SX y_obs = obs_params(1);
        casadi::SX width = obs_params(2);
        casadi::SX length = obs_params(3);
        casadi::SX theta = obs_params(4);
        // 旋转矩阵
        casadi::SX cos_theta = casadi::SX::cos(theta);
        casadi::SX sin_theta = casadi::SX::sin(theta);
        casadi::SX R_Theta = casadi::SX::vertcat({casadi::SX::horzcat({cos_theta, sin_theta}),
                                                     casadi::SX::horzcat({-sin_theta, cos_theta})});
        // 计算转换后的点
        casadi::SX front_vec = casadi::SX::vertcat({x_f - x_obs, y_f - y_obs});
        casadi::SX rear_vec = casadi::SX::vertcat({x_r - x_obs, y_r - y_obs});
        casadi::SX front_rotated = casadi::SX::mtimes(R_Theta, front_vec);
        casadi::SX rear_rotated = casadi::SX::mtimes(R_Theta, rear_vec);
        casadi::SX x_f_prime = front_rotated(0);
        casadi::SX y_f_prime = front_rotated(1);
        casadi::SX x_r_prime = rear_rotated(0);
        casadi::SX y_r_prime = rear_rotated(1);
        // 计算椭圆长短轴，使用经验公式
        casadi::SX a = length / 2.0+ vehicle_params_.width / 2.0 + vehicle_params_.safety_distance*10;
        casadi::SX b = width / 2.0 + vehicle_params_.width / 2.0 + vehicle_params_.safety_distance*3;
        // 不等式约束函数
        casadi::SX c_f = 1 - casadi::SX::pow(x_f_prime / a, 2) - casadi::SX::pow(y_f_prime / b, 2);
        casadi::SX c_r = 1 - casadi::SX::pow(x_r_prime / a, 2) - casadi::SX::pow(y_r_prime / b, 2);
        casadi::SXVector ineq_constraints_vec = {c_f, c_r};
        casadi::SX ineq_constraints = casadi::SX::vertcat(ineq_constraints_vec);
        // 分别构造ALM和Barrier的代价函数
        // alm还需要乘子和惩罚系数作为输入  
        casadi::SX mu = casadi::SX::sym("obs_mu", 2); // 2个不等式约束
        casadi::SX rho = casadi::SX::sym("obs_rho");
        // 乘子项：lambda^T * g(x)
        casadi::SX multiplier_term = casadi::SX::dot(mu, ineq_constraints);
        // 不等式约束乘子更新公式 mu_next = pho*max(ineq_constraints + mu/rho, 0),少了个rho，由外部更新时加上
        casadi::SX mu_next=casadi::SX::fmax(ineq_constraints+mu/rho, 0);
        // 惩罚项：(rho/2) * ||max(0, g(x))||^2
        casadi::SX penalty_term = 0.5 * rho * casadi::SX::sum1(casadi::SX::pow(mu_next, 2));
        // 完整ALM成本
        casadi::SX alm_cost = multiplier_term + penalty_term;
        // casadi::SX alm_cost =  penalty_term; // 只使用惩罚项效果更好一些
        // 函数定义  alm_obstacle_cost_, alm_obstacle_jacobian_, alm_obstacle_hessian_;barrier_obstacle_cost_, barrier_obstacle_jacobian_, barrier_obstacle_hessian_;alm_obstacle_mu_derivative_
        alm_obstacle_cost_ = casadi::Function("alm_obstacle_cost_function", {z, obs_params, mu, rho}, {alm_cost},
                                    {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost"});
        // jacobian
        casadi::SX alm_cost_jacobian = casadi::SX::gradient(alm_cost, z);
        alm_obstacle_jacobian_ = casadi::Function("alm_obstacle_jacobian", {z, obs_params, mu, rho}, {alm_cost_jacobian},
                                             {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost_jacobian"});
        // hessian
        casadi::SX alm_cost_hessian = casadi::SX::jacobian(alm_cost_jacobian, z);
        alm_obstacle_hessian_ = casadi::Function("alm_obstacle_hessian", {z, obs_params, mu, rho}, {alm_cost_hessian},
                                            {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_cost_hessian"});
        // 另外，定义一个函数计算乘子导数，用于更新乘子
        // 注意，ALM法乘子更新公式为mu_next=pho_*max(mu/rho + c,0)，
        // 所以直接输出max(mu/rho + c,0),然后外部乘以pho_
        alm_obstacle_mu_derivative_ = casadi::Function("alm_obstacle_mu_derivative", {z, obs_params, mu, rho}, {mu_next},
                                             {"state+control", "obs_params", "mu", "rho"}, {"alm_obstacle_mu_derivative"});
        
        // barrier代价函数,不需要乘子,而且系数是常量，公式为 sum(q1*exp(q2*c_i))
        casadi::SX barrier_q1 = casadi::SX::sym("barrier_q1");
        casadi::SX barrier_q2 = casadi::SX::sym("barrier_q2");
        casadi::SX barrier_cost =casadi::SX::sum1((barrier_q1) *casadi::SX::exp(barrier_q2 * ineq_constraints));
        barrier_obstacle_cost_ = casadi::Function("barrier_obstacle_cost", {z, obs_params, barrier_q1, barrier_q2}, {barrier_cost},
                                        {"state+control", "obs_params", "barrier_q1", "barrier_q2"}, {"barrier_obstacle_cost"});
        // barrier jacobian
        casadi::SX barrier_cost_jacobian = casadi::SX::gradient(barrier_cost, z);
        barrier_obstacle_jacobian_ = casadi::Function("barrier_obstacle_jacobian", {z, obs_params, barrier_q1, barrier_q2}, {barrier_cost_jacobian},
                                                {"state+control", "obs_params", "barrier_q1", "barrier_q2"}, {"barrier_obstacle_jacobian"});
        // barrier hessian
        casadi::SX barrier_cost_hessian = casadi::SX::jacobian(barrier_cost_jacobian, z);
        barrier_obstacle_hessian_ = casadi::Function("barrier_obstacle_hessian", {z, obs_params, barrier_q1, barrier_q2}, {barrier_cost_hessian},
                                               {"state+control", "obs_params", "barrier_q1", "barrier_q2"}, {"barrier_obstacle_hessian"});
    }


}
}