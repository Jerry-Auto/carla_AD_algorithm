#include <iostream>
#include "cilqr_planner/cost_define.h"
#include "cilqr_planner/planner_weight.h"
#include "general_modules/FrenetFrame.h"
using namespace AD_algorithm::planner;
using namespace AD_algorithm::general;

int main() {
    // 测试函数计算是否正确
    auto planner_params = std::make_shared<CILQRPlannerparams>();
    auto cilqr_problem = std::make_shared<planner_problem>(planner_params);
    
    std::vector<PathPoint> reference_line;
    for (double s = 0.0; s <= 100.0; s += 1.0) {
        PathPoint point;
        point.x = s;
        point.y = 0.0;
        point.heading = 0.0;
        point.accumulated_s = s;
        reference_line.push_back(point);
    }
    FrenetFrame  frenet_frame(reference_line);
    cilqr_problem->set_frenet_frame(std::make_shared<FrenetFrame>(frenet_frame));
    std::cout << "FrenetFrame set successfully." << std::endl;
    
        // 构造一组简单的状态和控制序列
        int N = planner_params->ilqr_params->N;
        int nx = planner_params->ilqr_params->nx;
        int nu = planner_params->ilqr_params->nu;
        Eigen::MatrixXd x_and_u = Eigen::MatrixXd::Zero(N + 1, nx + nu);
        // 设置第一个状态为参考线起点
        x_and_u.row(0).head(nx) << reference_line[0].x, reference_line[0].y, reference_line[0].heading, 0.0;
        // 控制量全为0
        for (int k = 0; k < N; ++k) {
            x_and_u.row(k).tail(nu).setZero();
        }
        // 终端状态也为参考线终点
        x_and_u.row(N).head(nx) << reference_line.back().x, reference_line.back().y, reference_line.back().heading, 0.0;

        // 1. planner_problem接口计算总代价
        double cost_api = cilqr_problem->compute_total_cost(x_and_u);
        std::cout << "planner_problem接口总代价: " << cost_api << std::endl;

        // 2. 手动计算（只考虑Q和R，不考虑约束和障碍）
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
        double cost_manual = 0.0;
        for (int k = 0; k < N; ++k) {
            Eigen::VectorXd xk = x_and_u.row(k).head(nx);
            Eigen::VectorXd uk = x_and_u.row(k).tail(nu);
            Eigen::VectorXd ref = Eigen::VectorXd::Zero(nx);
            ref << reference_line[k].x, reference_line[k].y, reference_line[k].heading, 0.0;
            Eigen::VectorXd state_error = xk - ref;
            double stage_cost_x = 0.5 * (state_error.transpose() * Q * state_error)(0,0);
            double stage_cost_u = 0.5 * (uk.transpose() * R * uk)(0,0);
            cost_manual += stage_cost_x + stage_cost_u;
        }
        // 终端代价
        Eigen::VectorXd xN = x_and_u.row(N).head(nx);
        Eigen::VectorXd refN = Eigen::VectorXd::Zero(nx);
        refN << reference_line.back().x, reference_line.back().y, reference_line.back().heading, 0.0;
        Eigen::VectorXd state_error_N = xN - refN;
        cost_manual += 0.5 * (state_error_N.transpose() * Q * state_error_N)(0,0);
        std::cout << "手动计算总代价: " << cost_manual << std::endl;

        // 3. 梯度和Hessian接口
        Eigen::MatrixXd grad_api = cilqr_problem->compute_total_gradient(x_and_u);
        std::cout << "planner_problem接口梯度范数: " << grad_api.norm() << std::endl;
        Eigen::Tensor<double, 3, Eigen::RowMajor> hess_api = cilqr_problem->compute_total_hessian(x_and_u);
        int dim = nx + nu;
        Eigen::MatrixXd h0 = Eigen::MatrixXd::Zero(dim, dim);
        for (int i = 0; i < dim; ++i) {
            for (int j = 0; j < dim; ++j) {
                h0(i, j) = hess_api(0, i, j);
            }
        }
        std::cout << "planner_problem接口Hessian第0步范数: " << h0.norm() << std::endl;

        // 4. 手动计算梯度（只对第一个时间步，状态部分）
        Eigen::VectorXd ref0 = Eigen::VectorXd::Zero(nx);
        ref0 << reference_line[0].x, reference_line[0].y, reference_line[0].heading, 0.0;
        Eigen::VectorXd grad_manual = Q * (x_and_u.row(0).head(nx) - ref0);
        std::cout << "手动计算第0步状态梯度: " << grad_manual.transpose() << std::endl;
        std::cout << "planner_problem接口第0步状态梯度 (API): " << grad_api.row(0).segment(0, nx).transpose() << std::endl;

    return 0;
}
