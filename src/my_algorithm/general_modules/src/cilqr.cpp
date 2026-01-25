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

    void ILQR::solve() {
        log("INFO","开始求解ILQR问题");
        solve_status_ = LQRSolveStatus::RUNNING;
        // 无论如何，粗解由cost_factory_提供,因为工程可以直接从参考上截取
        cost_factory_->get_initial_trajectory(x_, u_);
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
            // 每次迭代开始时，根据当前的x_和u_构建x_and_u
            for (int k = 0; k < params_->N; ++k) {
                x_and_u.row(k) << x_.row(k), u_.row(k);
            }
            x_and_u.row(params_->N) << x_.row(params_->N), Eigen::RowVectorXd::Zero(params_->nu);

            // Barrier法每次迭代都更新参数；ALM仅在前向传播失败时更新
            if (!cost_factory_->use_alm()) {
                cost_factory_->update(itr, x_and_u);
            }

            // 计算当前轨迹代价作为线搜索基准
            prev_cost_ = cost_factory_->compute_total_cost(x_and_u);
             
            iter_effective_flag = iter_step();
            if (cost_factory_->use_alm() &&
                solve_status_ == LQRSolveStatus::FORWARD_PASS_FAIL) {
                // 前向传播失败时，更新ALM乘子与rho，并同步基准代价
                cost_factory_->update(itr, x_and_u);
                prev_cost_ = cost_factory_->compute_total_cost(x_and_u);
            }
            if (solve_status_ == LQRSolveStatus::BACKWARD_PASS_FAIL ||
                solve_status_ == LQRSolveStatus::FORWARD_PASS_FAIL) {
                reg_ = std::max(reg_ * params_->reg_factor, params_->reg_max);// 失败时放大
                // SPDLOG_DEBUG("iter , increase mu to ", itr, reg_);
            } else if (solve_status_ == LQRSolveStatus::RUNNING) {
                reg_ = std::max(reg_ / params_->reg_factor, params_->reg_min);  // 成功时缩小
                // SPDLOG_DEBUG("iter , decrease mu to ", itr, reg_);
            }

            if (iter_effective_flag && max_delta_u_ < params_->delta_u_tol && max_delta_x_ < params_->delta_x_tol) {
                solve_status_ = LQRSolveStatus::CONVERGED;
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
        // 将结果传回工厂，方便下次作为初始值
        cost_factory_->set_result(u_, x_);
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
        const int nz = params_->nx + params_->nu;
        Eigen::MatrixXd x_and_u(params_->N + 1, nz);
        for (int k = 0; k < params_->N; ++k) {
            x_and_u.row(k) << x_.row(k), u_.row(k);
        }
        log("INFO","Computing gradients and Hessians");
        x_and_u.row(params_->N) << x_.row(params_->N), Eigen::RowVectorXd::Zero(params_->nu);

        Eigen::MatrixXd l_z=Eigen::MatrixXd::Zero(params_->N+1, nz);
        Eigen::Tensor<double, 3,Eigen::ColMajor> l_zz(params_->N+1, nz, nz);
        l_zz.setZero();
        // l_z是一个矩阵，每一行是一个时间步的代价梯度向量,形状为(N+1, nx+nu)
        // l_zz是一个张量，每一片是一个时间步的代价Hessian矩阵，形状为(N+1, nx+nu, nx+nu)
        cost_factory_->get_total_jacobian_hessian(x_and_u,l_z,l_zz);
        // 一个张量，每一片是一个时间步的状态转移函数雅可比矩阵，形状为(N, nx, nx+nu)
        auto f_z = cost_factory_->state_total_jacobian(x_and_u);
        // 一个张量，每一片是一个时间步的状态转移函数Hessian矩阵(三维张量堆成的四维张量)，形状为(N, nx, nx+nu, nx+nu)
        Eigen::Tensor<double, 4,Eigen::ColMajor> f_zz;
        if (params_->use_DDP) {
            f_zz = cost_factory_->state_total_hessian(x_and_u);
        }
        int N_f = f_z.dimension(0); // 应该等于params_->N
        int N_l = l_zz.dimension(0); // 应该等于params_->N+1,加上终端
        assert(N_f == params_->N && N_l == params_->N+1);

        // 一个向量，终端状态的代价梯度，形状为(nx)，从l_z最后一位提取出来
        Eigen::VectorXd V_x= l_z.row(params_->N).head(params_->nx);
        // 一个矩阵，终端状态的代价Hessian矩阵，形状为(nx, nx),从l_zz最后一片提取出来
        Eigen::Tensor<double, 2> V_xx_tensor = l_zz.chip(params_->N, 0);
        Eigen::Map<const Eigen::MatrixXd> V_xx_map(V_xx_tensor.data(), nz, nz);
        Eigen::MatrixXd V_xx = V_xx_map.block(0, 0, params_->nx, params_->nx);
        
        auto V_x_k= V_x;
        auto V_xx_k= V_xx;
        for (int i = params_->N - 1; i >= 0; --i) {
            // Use chip to extract slices without copying data
            Eigen::Tensor<double, 2> f_z_i = f_z.chip(i, 0);
            Eigen::Map<const Eigen::MatrixXd> f_jaco(f_z_i.data(), params_->nx, nz);
            
            Eigen::Tensor<double, 2> l_zz_i = l_zz.chip(i, 0);
            Eigen::Map<const Eigen::MatrixXd> l_hess_mat(l_zz_i.data(), nz, nz);
            // (1, nz)
            auto l_jaco= l_z.row(i);
            // Q_z = l_z + f_z' * V_x_k，形状为(n_x*n_z)^T*nx=(nz)
            auto Q_z= l_jaco.transpose() + f_jaco.transpose() * V_x_k;
            // Q_zz = l_zz + f_z' * V_xx_k * f_z，形状为(n_x*n_z)^T*(nx*nx)*(n_x*n_z)=(nz,nz)
            Eigen::MatrixXd Q_zz = l_hess_mat + f_jaco.transpose() * V_xx_k * f_jaco;

            if(params_->use_DDP){
                Eigen::Tensor<double, 3> f_zz_i = f_zz.chip(i, 0);
                for(int dim =0; dim < params_->nx; ++dim){
                    Eigen::Tensor<double, 2> f_zz_idim = f_zz_i.chip(dim, 0);
                    Eigen::Map<const Eigen::MatrixXd> f_hess_mat(f_zz_idim.data(), nz, nz);
                    Q_zz += V_x_k(dim) * f_hess_mat;
                }
            }
            // Ensure Q_zz is symmetric after DDP
            Q_zz = 0.5 * (Q_zz + Q_zz.transpose());

            // 提取Q矩阵的子块
            Eigen::MatrixXd Q_xx = Q_zz.block(0, 0, params_->nx, params_->nx);
            Eigen::MatrixXd Q_ux = Q_zz.block(params_->nx, 0, params_->nu, params_->nx);
            Eigen::MatrixXd Q_uu = Q_zz.block(params_->nx, params_->nx, params_->nu, params_->nu);
            Eigen::VectorXd Q_x = Q_z.segment(0, params_->nx);
            Eigen::VectorXd Q_u = Q_z.segment(params_->nx, params_->nu);
            // 正则化
            Eigen::MatrixXd Q_uu_reg = Q_uu + reg_ * Eigen::MatrixXd::Identity(params_->nu, params_->nu);
            Eigen::LDLT<Eigen::MatrixXd> ldlt_quu(Q_uu_reg);
            if (ldlt_quu.info() == Eigen::NumericalIssue) {
                log("WARN","反向传播失败，正则化系数： ", reg_);
                solve_status_ = LQRSolveStatus::BACKWARD_PASS_FAIL;
                return false;
            }
            // 公式： d = -Q_uu^{-1} * Q_u, K = -Q_uu^{-1} * Q_ux
            // 等价于求解： Q_uu * d = -Q_u, Q_uu * K = -Q_ux
            Eigen::VectorXd d = ldlt_quu.solve(-Q_u);
            Eigen::MatrixXd K = ldlt_quu.solve(-Q_ux);
            d_mat.row(i) = d.transpose();
            K_mat.block(params_->nu * i, 0, params_->nu, params_->nx) = K;

            // Comput V_x, V_xx
            V_x_k = Q_x + Q_ux.transpose() * d;
            V_xx_k = Q_xx + Q_ux.transpose() * K;
            // 确保V_xx_k是对称的
            V_xx_k = 0.5 * (V_xx_k + V_xx_k.transpose());

            // 期望代价减少量 = 0.5 * d^T Q_uu d - d^T Q_u
            delta_V_[0] += 0.5 * d.transpose() * Q_uu * d;
            delta_V_[1] += d.transpose() * Q_u;

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
                log("INFO","线搜索: alpha ",alpha,", 代价下降量过小", cost_reduction,"，认为收敛" );
                return true;
            }
            double approx_cost_decay = (alpha * alpha * delta_V_[0] + alpha * delta_V_[1]);
            log("DEBUG", "alpha: ", alpha, ", cost_reduction: ", cost_reduction, ", approx_cost_decay: ", approx_cost_decay);
            if (cost_reduction > 0.0 &&// 代价确实减少
                (cost_reduction >= params_->accept_threshold * approx_cost_decay)) { // 实际减少与预期减少比值足够大
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
            auto dx= new_x_.row(i) - x_.row(i);
            auto u_ff =  alpha * d_.row(i);
            auto u_fb = K_.block(params_->nu * i, 0, params_->nu, params_->nx) * dx.transpose();
            auto du = u_ff + u_fb.transpose();
            new_u_.row(i) = u_.row(i) + du;
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