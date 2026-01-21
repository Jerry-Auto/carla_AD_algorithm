#include "general_modules/cilqr.h"
#include <casadi/casadi.hpp>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace AD_algorithm {
namespace general {
    ILQR::ILQR(std::shared_ptr<cost_factory> cost_factory) : logger_(std::make_shared<AD_algorithm::general::Logger>("CILQR")) {
        cost_factory_ = cost_factory;
        // 参数由具体问题定义类提供
        params_ = cost_factory_->get_params();
        x_ = Eigen::MatrixXd::Zero(params_->N + 1, params_->nx);
        u_ = Eigen::MatrixXd::Zero(params_->N, params_->nu);
        new_x_ = Eigen::MatrixXd::Zero(params_->N + 1, params_->nx);
        new_u_ = Eigen::MatrixXd::Zero(params_->N, params_->nu);
        d_ = Eigen::MatrixXd::Zero(params_->N, params_->nu);
        K_ = Eigen::MatrixXd::Zero(params_->nu * params_->N, params_->nx);
        V_x_ = Eigen::MatrixXd::Zero(params_->N + 1, params_->nx);
        V_xx_ = Eigen::MatrixXd::Zero(params_->nx, params_->nx);
        last_solve_u_ = Eigen::MatrixXd::Zero(params_->N, params_->nu);
    }

    void ILQR::set_initial_state(const Eigen::VectorXd& x0) { 
        if (x0.size() == params_->nx) {
            if (x_.rows() != params_->N + 1 || x_.cols() != params_->nx) {
                x_ = Eigen::MatrixXd::Zero(params_->N + 1, params_->nx);
            }
            log("INFO","Setting initial state");
            x_.row(0) = x0.transpose();
        } else {
            log("ERROR","Initial state dimension mismatch");
        }
    }

    void ILQR::get_init_traj() {
        // 实现首次运行时的初始轨迹生成逻辑
        u_.setZero();
        // Ensure the first state x_.row(0) has been set by the caller via set_initial_state
        for (int i = 0; i < params_->N; ++i) {
            Eigen::RowVectorXd z_row(params_->nx + params_->nu);
            z_row << x_.row(i), u_.row(i);
            auto next_state = cost_factory_->state_transition(Eigen::VectorXd(z_row.transpose()));
            x_.row(i + 1) = next_state.transpose();
        }
    }

    void ILQR::get_init_traj_increment() {
        // 实现非首次运行时的初始轨迹生成逻辑,逻辑是将上次控制向前平移一位，最后一位保持不变
        // 行代表时间步，列代表控制维度
        Eigen::MatrixXd new_u = Eigen::MatrixXd::Zero(params_->N, params_->nu);
        if (u_.rows() == params_->N && u_.cols() == params_->nu) {
            if (params_->N > 1) {
                // block的用法：block(start_row, start_col, num_rows, num_cols)，两个block赋值，后面两位必须相等
                // 这里表示将旧的控制序列的第1行到第N_-1行复制到新的控制序列的第0行到第N_-2行，列全选
                new_u.block(0, 0, params_->N - 1, params_->nu) = last_solve_u_.block(1, 0, params_->N - 1, params_->nu);
                new_u.row(params_->N - 1) = last_solve_u_.row(params_->N - 1);
            } else {
                new_u.row(0) = last_solve_u_.row(0);
            }
        }
        u_ = new_u;

        // Forward propagate states using f_
        for (int i = 0; i < params_->N; ++i) {
            Eigen::RowVectorXd z_row(params_->nx + params_->nu);
            z_row << x_.row(i), u_.row(i);
            auto next_state = cost_factory_->state_transition(Eigen::VectorXd(z_row.transpose()));
            x_.row(i + 1) = next_state.transpose();
        }
    }
    void ILQR::solve() {
        log("INFO","开始求解ILQR问题");
        solve_status_ = LQRSolveStatus::RUNNING;
        if(first_run_) {
            first_run_ = false;
            get_init_traj();
        } else {
            get_init_traj_increment();
        }
        log("INFO","初始轨迹生成完成");
        // Build x_and_u matrix: N rows of [x_k, u_k], plus last row [x_N, zeros]
        Eigen::MatrixXd x_and_u(params_->N + 1, params_->nx + params_->nu);
        for (int k = 0; k < params_->N; ++k) {
            x_and_u.row(k) << x_.row(k), u_.row(k);
        }
        x_and_u.row(params_->N) << x_.row(params_->N), Eigen::RowVectorXd::Zero(params_->nu);
        prev_cost_ = cost_factory_->compute_total_cost(x_and_u);
        log("INFO","初始总代价: ", prev_cost_);

        reg_ = params_->reg_init;
        start_timer();
        bool is_exceed_max_itr = true;
        bool iter_effective_flag = false;
        for (int itr = 0; itr < params_->max_iter; ++itr) {
            // 如果前向或反向传播失败，增加正则化参数，更新乘子，同时使用上次的轨迹
            cost_factory_->update(itr);
            iter_effective_flag = iter_step();
            if (solve_status_ == LQRSolveStatus::BACKWARD_PASS_FAIL ||
                solve_status_ == LQRSolveStatus::FORWARD_PASS_FAIL) {
                reg_ = std::max(reg_ * params_->reg_factor, params_->reg_max);
                // SPDLOG_DEBUG("iter , increase mu to ", itr, reg_);
            } else if (solve_status_ == LQRSolveStatus::RUNNING) {
                reg_ *= params_->reg_factor;
                // SPDLOG_DEBUG("iter , decrease mu to ", itr, reg_);
            }
            if (iter_effective_flag) {
                x_ = new_x_;
                u_ = new_u_;
            }
            
            if (reg_ > params_->reg_max) {
                double solve_cost_time = stop_timer();
                log("WARN",
                    "正则化参数达到最大值,迭代次数:"+std::to_string(itr)+",最终代价:"+std::to_string(prev_cost_)+", 计算时间"+std::to_string(solve_cost_time)+"ms");
                is_exceed_max_itr = false;
                break;
            } else if (solve_status_ == LQRSolveStatus::CONVERGED) {
                double solve_cost_time = stop_timer();
                log("INFO",
                    "优化已收敛, 最终代价:"+std::to_string(prev_cost_)+", 计算时间 "+std::to_string(solve_cost_time)+" ms");
                is_exceed_max_itr = false;
                break;
            }
        }

        last_solve_u_ = u_;

        if (is_exceed_max_itr) {
            log("WARN","迭代次数达到最大值, 最终代价: "+std::to_string(prev_cost_));
        }
        double solve_cost_time = stop_timer();
        log("INFO","求解完成，耗时 "+std::to_string(solve_cost_time)+" ms");
    }

    bool ILQR::iter_step() {
        log("INFO","Starting backward_pass");
        bool success = false;
        success = backward_pass();
        if (solve_status_ == LQRSolveStatus::BACKWARD_PASS_FAIL) {
            return false;
        }
        log("INFO","Backward pass successful, starting line search");
        success = linear_search(params_->alpha_ls_init);
        return success;
    }

    bool ILQR::backward_pass() {
        bool success = false;
        Eigen::MatrixXd K_mat = Eigen::MatrixXd::Zero(params_->nu * params_->N, params_->nx);
        Eigen::MatrixXd d_mat = Eigen::MatrixXd::Zero(params_->N, params_->nu);
        // Build x_and_u matrix: N rows of [x_k, u_k], plus last row [x_N, zeros]
        Eigen::MatrixXd x_and_u(params_->N + 1, params_->nx + params_->nu);
        for (int k = 0; k < params_->N; ++k) {
            x_and_u.row(k) << x_.row(k), u_.row(k);
        }
        log("INFO","Computing gradients and Hessians");
        x_and_u.row(params_->N) << x_.row(params_->N), Eigen::RowVectorXd::Zero(params_->nu);
        // 一个矩阵，每一行是一个时间步的代价矩阵
        auto l_z = cost_factory_->compute_total_gradient(x_and_u);
        // 一个张量，每一片是一个时间步的代价Hessian矩阵
        auto l_zz = cost_factory_->compute_total_hessian(x_and_u);
        // 一个张量，每一片是一个时间步的状态转移函数雅可比矩阵
        auto f_z = cost_factory_->state_total_jacobian(x_and_u);
        // 一个张量，每一片是一个时间步的状态转移函数Hessian矩阵(三维张量堆成的四维张量)
        Eigen::Tensor<double, 4,Eigen::RowMajor> f_zz;
        if (params_->use_DDP) {
            f_zz = cost_factory_->state_total_hessian(x_and_u);
        }
        // 一个向量，终端状态的代价梯度
        auto V_x= cost_factory_->compute_terminal_gradient(x_.row(params_->N).transpose());
        // 一个矩阵，终端状态的代价Hessian矩阵
        auto V_xx= cost_factory_->compute_terminal_hessian(x_.row(params_->N).transpose());
        auto V_x_k= V_x;
        auto V_xx_k= V_xx;
        for (int i = params_->N - 1; i >= 0; --i) {
            // 首先从张量中提取对应时间步的雅可比和Hessian矩阵，chip(index,dim)表示在dim维度上提取index切片
            Eigen::Tensor<double, 2, Eigen::RowMajor> f_jaco_temp = f_z.chip(i, 0).eval();
            Eigen::Tensor<double, 2, Eigen::RowMajor> l_hess_tensor = l_zz.chip(i,0).eval();
            Eigen::Map<Eigen::MatrixXd> f_jaco(f_jaco_temp.data(), params_->nx, params_->nx + params_->nu);
            Eigen::Map<Eigen::MatrixXd> l_hess_mat(l_hess_tensor.data(), params_->nx + params_->nu, params_->nx + params_->nu);
            
            auto l_jaco= l_z.row(i);
            auto Q_z= l_jaco.transpose() + f_jaco.transpose() * V_x_k;
            Eigen::MatrixXd Q_zz = l_hess_mat + f_jaco.transpose() * V_xx_k * f_jaco;

            if(params_->use_DDP){
                Eigen::Tensor<double, 3, Eigen::RowMajor> f_hess_temp = f_zz.chip(i,0).eval();
                // DDP 二阶项修正，不支持向量乘张量，需要逐维处理
                for(int dim =0; dim < params_->nx; ++dim){
                    Eigen::Tensor<double, 2, Eigen::RowMajor> f_hess_dim_temp = f_hess_temp.chip(dim,0).eval();
                    Eigen::Map<Eigen::MatrixXd> f_hess_mat(f_hess_dim_temp.data(), params_->nx + params_->nu, params_->nx + params_->nu);
                    Q_zz.block(0,0,params_->nx,params_->nx) += V_x_k(dim) * f_hess_mat.block(0,0,params_->nx,params_->nx);
                }
            }
            // 提取Q矩阵的子块
            Eigen::MatrixXd Q_xx = Q_zz.block(0, 0, params_->nx, params_->nx);
            Eigen::MatrixXd Q_ux = Q_zz.block(params_->nx, 0, params_->nu, params_->nx);
            Eigen::MatrixXd Q_uu = Q_zz.block(params_->nx, params_->nx, params_->nu, params_->nu);
            Eigen::VectorXd Q_x = Q_z.segment(0, params_->nx);
            Eigen::VectorXd Q_u = Q_z.segment(params_->nx, params_->nu);
            // 正则化
            Eigen::MatrixXd Q_uu_reg = Q_uu + reg_ * Eigen::MatrixXd::Identity(params_->nu, params_->nu);
            Eigen::LLT<Eigen::MatrixXd> llt_quu(Q_uu_reg);
            if (llt_quu.info() == Eigen::NumericalIssue) {
                log("WARN","反向传播失败，正则化系数： ", reg_);
                solve_status_ = LQRSolveStatus::BACKWARD_PASS_FAIL;
                return false;
            }

            d_mat.row(i) = Q_uu_reg.ldlt().solve(-Q_u);
            K_mat.block(params_->nu * i, 0, params_->nu, params_->nx) = Q_uu_reg.ldlt().solve(-Q_ux);

            // Comput V_x, V_xx
            V_x_k = Q_x + Q_ux.transpose() * d_mat.row(i).transpose();
            V_xx_k = Q_xx + Q_ux.transpose() * K_mat.block(params_->nu * i, 0, params_->nu, params_->nx);
            // 确保V_xx_k是对称的
            V_xx_k = 0.5 * (V_xx_k + V_xx_k.transpose());

            // expected cost reduction
            delta_V_[0] += (0.5 * d_mat.row(i) * Q_uu * d_mat.row(i).transpose())(0, 0);
            delta_V_[1] += (d_mat.row(i) * Q_u)(0, 0);

            success = true;
        }
        d_= d_mat;
        K_= K_mat;
        return success;
    }

    bool ILQR::linear_search(double alpha) {
        curr_cost_= prev_cost_;// 初始化为上次代价
        for(;alpha >= params_->min_alpha; alpha *= params_->beta_ls){
            forward_pass(alpha);
            Eigen::MatrixXd x_and_u(params_->N + 1, params_->nx + params_->nu);
            for (int k = 0; k < params_->N; ++k) {
                x_and_u.row(k) << new_x_.row(k), new_u_.row(k);
            }
            x_and_u.row(params_->N) << new_x_.row(params_->N), Eigen::RowVectorXd::Zero(params_->nu);
            double new_cost = cost_factory_->compute_total_cost(x_and_u);
            double cost_reduction = curr_cost_ - new_cost;
            if(std::fabs(alpha - params_->alpha_ls_init) < params_->EPS && std::fabs(cost_reduction) < params_->cost_tol){
                // 代价下降量过小，认为收敛
                curr_cost_ = new_cost;
                x_ = new_x_;
                u_ = new_u_;
                solve_status_ = LQRSolveStatus::CONVERGED;
                log("INFO","线搜索: alpha {:.4f}, 代价下降量过小 {:.6f}，认为收敛", alpha, cost_reduction);
                return true;
            }
            double approx_cost_decay = -(alpha * alpha * delta_V_[0] + alpha * delta_V_[1]);
            if (cost_reduction > 0.0 &&// 代价确实减少
                (approx_cost_decay < 0.0 ||// 近似衰减为负（预期减少）
                cost_reduction / approx_cost_decay > 0.1)) { // 实际减少与预期减少比值足够大
                if (std::fabs(alpha - params_->alpha_ls_init) > params_->EPS) {
                    solve_status_ = LQRSolveStatus::FORWARD_PASS_SMALL_STEP;
                }
                curr_cost_ = new_cost;
                x_ = new_x_;
                u_ = new_u_;
                return true;
            }
        }
        solve_status_ = LQRSolveStatus::FORWARD_PASS_FAIL;
        return false;
    }

    void ILQR::forward_pass(double alpha) {
        new_x_.setZero();
        new_u_.setZero();
        new_x_.row(0) = x_.row(0);
        max_delta_u_ = 0;
        max_delta_x_ = 0;
        for(int i=0;i<params_->N;++i){
            Eigen::RowVectorXd u_ff =  alpha * d_.row(i);
            Eigen::VectorXd u_fb = K_.block(params_->nu * i, 0, params_->nu, params_->nx) * (new_x_.row(i) - x_.row(i)).transpose();
            Eigen::RowVectorXd du = u_ff + u_fb.transpose();
            new_u_.row(i) = u_.row(i) + du;
            auto dx= new_x_.row(i) - x_.row(i);
            if(dx.norm() > max_delta_x_){
                max_delta_x_ = dx.norm();
            }
            if(du.norm() > max_delta_u_){
                max_delta_u_ = du.norm();
            }
            Eigen::RowVectorXd z_row(params_->nx + params_->nu);
            z_row << new_x_.row(i), new_u_.row(i);
            auto next_state = cost_factory_->state_transition(Eigen::VectorXd(z_row.transpose()));
            new_x_.row(i + 1) = next_state.transpose();
        }
    }

} // namespace general
} // namespace AD_algorithm