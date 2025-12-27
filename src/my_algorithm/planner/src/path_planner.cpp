#include "planner/path_planner.h"
#include "planner/DP_solver.h"
#include "general_modules/polynomial_curve.h"
#include <iostream>
#include <algorithm>
#include <chrono>

namespace AD_algorithm {
namespace planner {

using namespace general;

PathPlanner::PathPlanner(const WeightCoefficients& weights,const PathPlannerConfig& config) 
    : weights_(weights), config_(config) {
    qp_solver_ = std::make_shared<OsqpEigen::Solver>();
    qp_solver_->settings()->setWarmStart(true);
    
    // 默认日志回调
    log_callback_ = [](const std::string& msg) {
        std::cout << "[PathPlanner] " << msg << std::endl;
    };
}

void PathPlanner::log(const std::string& message, const std::string& level) {
    if (_enable_log && log_callback_) {
        log_callback_("[" + level + "] " + message);
    }
}

std::vector<TrajectoryPoint> PathPlanner::planPath(
    const std::shared_ptr<FrenetFrame>& frenet_frame,
    const FrenetPoint& planning_start_point,
    const std::vector<FrenetPoint>& static_obstacles
    ) {
    // 注意，传入的planning_start_point的s是相对于全局frenet坐标系的原点来计算的
    // static_obstacles也是相对于全局坐标系的S
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 验证配置
    if (!config_.validate()) {
        log("Invalid configuration", "ERROR");
        return {};
    }

    if (!frenet_frame) {
        log("Frenet frame is null", "ERROR");
        return {};
    }
    
    log("PathPlanner Config: s_step=" + std::to_string(config_.s_sample_distance) + 
        ", s_num=" + std::to_string(config_.s_sample_number));

    // 清除旧的路径
    dp_path_.clear();
    trajectory_.clear();
    
    // log("Starting path planning with " + std::to_string(static_obstacles.size()) + " static obstacles");
    
    // 复制起始点偏移量，并确保起点s坐标为0
    // S偏移了，但是轨迹点和障碍物的全局x,y都是没变的
    FrenetPoint start_point = planning_start_point;
    double offset_s = start_point.s;
    start_point.s = 0.0;
    
    // log("Planning start point: s=" + std::to_string(start_point.s) + 
    //     ", l=" + std::to_string(start_point.l));
    
    // 1. 使用DP进行路径搜索，起点的s坐标必须为0
    log("Running DP path planning...");

    SLState start_state(start_point.s, start_point.l, 
                       start_point.l_prime, start_point.l_prime_prime);
    if(_enable_log){
        std::cout << "[DEBUG] DP Start State: s=" << start_state.s 
                  << ", l=" << start_state.l 
                  << ", l'=" << start_state.l_prime 
                  << ", l''=" << start_state.l_prime_prime << std::endl;
    }

    // 转换障碍物
    std::vector<SLObstacle> sl_obstacles;
    for (const auto& obs : static_obstacles) {
        sl_obstacles.emplace_back(obs.s - offset_s, obs.l, 5.0, 2.0, config_.safety_margin);//这里已经转化到局部坐标系
    }
    
    // 创建策略
    auto cost_function = std::make_shared<PathCostFunction>(weights_, config_, sl_obstacles);
    auto constraint_checker = std::make_shared<PathConstraintChecker>(
        config_.lane_width * 1.5,//道路边界上界，相对于参考线对应的frenet中心线
        -config_.lane_width * 0.5,//道路边界下界，相对于参考线对应的frenet中心线
        sl_obstacles
    );
    auto sampling_strategy = std::make_shared<PathSamplingStrategy>(config_);
    auto backtrack_strategy = std::make_shared<DefaultBacktrackStrategy<SLState>>();
    
    // 配置DP规划器
    DpPlannerConfig dp_config;
    dp_config.max_layers = config_.s_sample_number;
    dp_config.enable_pruning = true;
    dp_config.pruning_keep_num = 5;
    dp_config.debug_mode = true;
    
    // 执行规划
    DpPlanner<SLState> dp_planner(
        cost_function, constraint_checker, 
        sampling_strategy, backtrack_strategy, dp_config
    );

    // 可以使用预设采样网格的接口
    if(!dp_sampling_grid_){
        // 只有第一次规划时生成采样网格，后续复用
        dp_sampling_grid_ = std::make_shared<std::vector<std::vector<SLState>>>(
            sampling_strategy->generateSamplingGrid(start_state, config_.s_sample_number)
        );
    }

    auto result = dp_planner.planWithGrid(*dp_sampling_grid_, start_state);
    // auto result = dp_planner.plan(start_state, config.s_sample_number);//每次都重新生成采样网格，较慢
    
    if (!result.success) {
        log("DP planning failed: " + result.message, "ERROR");
        return {};
    }

    
    // dp_planner.printSearchTree([](const SLState& state) {
    //     std::cout << "s=" << std::setw(6) << state.s 
    //               << ", l=" << std::setw(6) << state.l 
    //               << ", l'=" << std::setw(6) << state.l_prime
    //               << ", l''=" << std::setw(6) << state.l_prime_prime;
    // });

    log("DP planning succeeded, found path with " + std::to_string(result.optimal_path.size()) + " points");
    log("Total cost: " + std::to_string(result.total_cost));
    log("Computation time: " + std::to_string(result.computation_time_ms) + " ms");
    
    // 转换结果
    dp_path_.clear();
    for (const auto& state : result.optimal_path) {
        FrenetPoint point;
        point.s = state.s;
        point.l = state.l;
        point.l_prime = state.l_prime;
        point.l_prime_prime = state.l_prime_prime;
        dp_path_.push_back(point);
    }

    // 增加路径点密度
    IncreasePathDensity(dp_path_,config_.qp_dense_path_interval);
    log("Increased path density to " + std::to_string(dp_path_.size()) + " points");

    // 2. 生成凸空间
    log("Generating convex space...");
    std::vector<double> l_min, l_max;

    std::vector<FrenetPoint> local_static_obstacles;
    // 上面的SL障碍物是给DP用的，这里的还没有转换，需要转换到局部坐标系
    for (const auto& obs : static_obstacles) {
        FrenetPoint local_obs = obs;
        local_obs.s -= offset_s;
        local_static_obstacles.push_back(local_obs);
    }

    generateConvexSpace(local_static_obstacles, l_min, l_max);
    
    // 3. QP路径优化
    log("Running QP path optimization...");
    QP_pathOptimization(l_min, l_max);
    
    if (qp_path_.empty()) {
        log("QP optimization failed: No path generated", "ERROR");
        return {};
    }

    log("QP optimization completed, " + std::to_string(qp_path_.size()) + " points generated");
    
    // 路径加密
    IncreasePathDensity(qp_path_,config_.final_path_interval);

    // 所有的点的S要偏移回来
    for(auto& sl_pnt:qp_path_){
        sl_pnt.s+=offset_s;
    }

    // 4. 转换为笛卡尔坐标系
    log("Converting to Cartesian coordinates...");
    trajectory_ = frenet_frame->frenet_to_cartesian(qp_path_);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    log("Path planning completed in " + std::to_string(duration.count()) + " ms");
    log("Generated " + std::to_string(trajectory_.size()) + " trajectory points");
    return trajectory_;
}

bool PathPlanner::QP_pathOptimization(const std::vector<double>& l_min,const std::vector<double>& l_max) {
    // 0. 初始化与准备
    qp_path_.clear();
    size_t point_num = dp_path_.size();
    if (point_num == 0) return false;

    double l_desire = 0.0;
    double dl_desire = 0.0;
    double ddl_desire = 0.0;

    // ==========================================
    // 1. 建立目标函数 (Cost Function)
    // J = 0.5 * x'Hx + f'x
    // ==========================================

    // 1.1 构建 H 矩阵 (Hessian Matrix)
    // H = w_ref*H_ref + w_dl*H_dl + ...
    Eigen::SparseMatrix<double> H_ref(point_num, 3*point_num);
    Eigen::SparseMatrix<double> H_dl(point_num, 3*point_num);
    Eigen::SparseMatrix<double> H_ddl(point_num, 3*point_num);
    Eigen::SparseMatrix<double> H_mid(point_num, 3*point_num);
    
    // 填充基础代价矩阵
    for (size_t i = 0; i < point_num; i++) {
        H_ref.insert(i, 3*i) = 1;       // 参考线代价
        H_mid.insert(i, 3*i) = 1;       // 居中代价
        H_dl.insert(i, 3*i+1) = 1;      // 一阶导代价
        H_ddl.insert(i, 3*i+2) = 1;     // 二阶导代价
    }

    // H_dddl: 三阶导数 (Jerk) 代价，使用差分近似
    Eigen::SparseMatrix<double> H_dddl(point_num-1, 3*point_num);
    for (size_t i = 0; i < point_num-1; i++) {
        H_dddl.insert(i, 3*i+2) = -1;
        H_dddl.insert(i, 3*(i+1) + 2) = 1;
    }

    // 终点状态代价
    Eigen::SparseMatrix<double> H_l_end(1, 3*point_num);
    Eigen::SparseMatrix<double> H_dl_end(1, 3*point_num);
    Eigen::SparseMatrix<double> H_ddl_end(1, 3*point_num);
    H_l_end.insert(0, 3*(point_num-1)) = 1;
    H_dl_end.insert(0, 3*(point_num-1)+1) = 1;
    H_ddl_end.insert(0, 3*(point_num-1)+2) = 1;

    // 组合总 H 矩阵
    Eigen::SparseMatrix<double> H = 2 * (
        weights_.path_qp_w_ref     * H_ref.transpose()     * H_ref + 
        weights_.path_qp_w_dl      * H_dl.transpose()      * H_dl + 
        weights_.path_qp_w_ddl     * H_ddl.transpose()     * H_ddl +
        weights_.path_qp_w_dddl    * H_dddl.transpose()    * H_dddl +
        weights_.path_qp_w_mid     * H_mid.transpose()     * H_mid +
        weights_.path_qp_w_l_end   * H_l_end.transpose()   * H_l_end +
        weights_.path_qp_w_dl_end  * H_dl_end.transpose()  * H_dl_end + 
        weights_.path_qp_w_ddl_end * H_ddl_end.transpose() * H_ddl_end
    );

    // 1.2 构建 f 向量 (Gradient Vector)
    Eigen::VectorXd f_mid = Eigen::VectorXd::Zero(3*point_num);
    Eigen::VectorXd f_l_end = Eigen::VectorXd::Zero(3*point_num);
    Eigen::VectorXd f_dl_end = Eigen::VectorXd::Zero(3*point_num);
    Eigen::VectorXd f_ddl_end = Eigen::VectorXd::Zero(3*point_num);

    for (size_t i = 0; i < point_num; i++) {
        // 期望轨迹位于 DP 路径附近
        f_mid[3*i] = -2.0 * (l_max[i] + l_min[i]) / 2.0;
    }
    f_l_end[3*(point_num-1)] = -2.0 * l_desire;
    f_dl_end[3*(point_num-1) + 1] = -2.0 * dl_desire;
    f_ddl_end[3*(point_num-1) + 2] = -2.0 * ddl_desire;

    Eigen::VectorXd f = weights_.path_qp_w_mid * f_mid + 
                        weights_.path_qp_w_l_end * f_l_end +
                        weights_.path_qp_w_dl_end * f_dl_end + 
                        weights_.path_qp_w_ddl_end * f_ddl_end;
    

    // ==========================================
    // 2. 建立约束 (Constraints)
    // l <= Ax <= u
    // ==========================================

    // 2.1 连续性约束 (Continuity Constraints)
    // 基于泰勒展开保证相邻点之间的位置和导数连续
    Eigen::SparseMatrix<double> A_continuity(2*(point_num-1), 3*point_num);
    double delta_s = dp_path_[1].s - dp_path_[0].s;
    
    for (size_t i = 0; i < point_num-1; i++) {
        int r = 2*i; // 行索引
        int c = 3*i; // 列索引
        
        // 位置连续: l_{i+1} = l_i + l'_i*ds + ...
        A_continuity.insert(r, c + 0) = 1.0;
        A_continuity.insert(r, c + 1) = delta_s;
        A_continuity.insert(r, c + 2) = delta_s*delta_s/3.0;
        A_continuity.insert(r, c + 3) = -1.0; // -l_{i+1}
        A_continuity.insert(r, c + 5) = delta_s*delta_s/6.0;
        
        // 导数连续: l'_{i+1} = l'_i + ...
        A_continuity.insert(r + 1, c + 1) = 1.0;
        A_continuity.insert(r + 1, c + 2) = delta_s/2.0;
        A_continuity.insert(r + 1, c + 4) = -1.0; // -l'_{i+1}
        A_continuity.insert(r + 1, c + 5) = delta_s/2.0;
    }

    Eigen::VectorXd low_boundary_continuity = Eigen::VectorXd::Zero(2*point_num - 2);
    Eigen::VectorXd up_boundary_continuity = Eigen::VectorXd::Zero(2*point_num - 2);


    // 2.2 避障约束 (Collision Avoidance Constraints)
    // 车辆包围盒需在可行区域 [l_min, l_max] 内
    Eigen::SparseMatrix<double> A_collision(4*point_num, 3*point_num);
    Eigen::VectorXd low_boundary_collision(4*point_num);
    Eigen::VectorXd up_boundary_collision(4*point_num);
    
    double d1 = 3.0; // 前悬长度
    double d2 = 2.0; // 后悬长度
    double car_width = 3.0;

    for (size_t i = 0; i < point_num; i++) {
        // 对每个点添加4个约束，分别对应车身不同部位的横向位置
        // l_corner = l + l' * dist
        A_collision.insert(4*i + 0, 3*i + 0) = 1; A_collision.insert(4*i + 0, 3*i + 1) = d1;
        A_collision.insert(4*i + 1, 3*i + 0) = 1; A_collision.insert(4*i + 1, 3*i + 1) = d1;
        A_collision.insert(4*i + 2, 3*i + 0) = 1; A_collision.insert(4*i + 2, 3*i + 1) = -d2;
        A_collision.insert(4*i + 3, 3*i + 0) = 1; A_collision.insert(4*i + 3, 3*i + 1) = -d2;
        
        // 计算当前点附近的 strict 边界
        int d1_index = std::ceil(d1 / delta_s);
        int d2_index = std::ceil(d2 / delta_s);
        int start_index = (int)std::max((int)(i - d2_index), 0);
        int end_index = std::min((int)(i + d1_index), (int)point_num-1);
        double ubi = std::numeric_limits<double>::max();
        double lbi = std::numeric_limits<double>::lowest();

        for(int j = start_index ; j <= end_index ; j++ ) {
            if (l_max[j] < ubi) ubi = l_max[j];
            if (l_min[j] > lbi) lbi = l_min[j];
        }

        if (lbi + car_width > ubi) {
             std::cout << "[DEBUG] Infeasible constraint at index " << i 
                       << ": lbi=" << lbi << ", ubi=" << ubi 
                       << ", width=" << car_width 
                       << ", start_idx=" << start_index << ", end_idx=" << end_index << std::endl;
        }
    
        // 设置上下界
        low_boundary_collision[4*i + 0] = lbi - car_width/2.0;
        low_boundary_collision[4*i + 1] = lbi + car_width/2.0;
        low_boundary_collision[4*i + 2] = lbi - car_width/2.0;
        low_boundary_collision[4*i + 3] = lbi + car_width/2.0;
        
        up_boundary_collision[4*i + 0] = ubi - car_width/2.0;
        up_boundary_collision[4*i + 1] = ubi + car_width/2.0;
        up_boundary_collision[4*i + 2] = ubi - car_width/2.0;
        up_boundary_collision[4*i + 3] = ubi + car_width/2.0;
    }
    
    // 放宽起点附近的避障约束，允许车辆从非法位置回归
    int relax_num = 5; // 放宽前5个点
    if (relax_num > point_num) relax_num = point_num;
    
    for(int i=0; i<relax_num; ++i) {
        for(int k=0; k<4; ++k) {
            low_boundary_collision[4*i + k] = -1e10;
            up_boundary_collision[4*i + k] = 1e10;
        }
    }

    // 2.3 规划起点约束 (Start Point Constraint)
    // 强制第一点的状态等于规划起点
    Eigen::SparseMatrix<double> A_start(3, 3*point_num);
    A_start.insert(0,0) = 1.0;
    A_start.insert(1,1) = 1.0;
    A_start.insert(2,2) = 1.0;
    
    Eigen::VectorXd b_start(3);
    b_start << dp_path_.front().l, dp_path_.front().l_prime, dp_path_.front().l_prime_prime;
    
    if (std::abs(dp_path_.front().l) > 100.0) {
        std::cout << "[DEBUG] QP Input Suspicious: dp_path_[0].l = " << dp_path_.front().l << std::endl;
    }

    // 2.4 组合所有约束
    // 使用 Triplet 构建 A_total，避免 middleCols 赋值问题
    std::vector<Eigen::Triplet<double>> triplet_list;
    
    // 添加 A_continuity
    for (int k=0; k<A_continuity.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_continuity, k); it; ++it) {
            triplet_list.emplace_back(it.row(), it.col(), it.value());
        }
    }
    
    int row_offset = A_continuity.rows();
    // 添加 A_collision
    for (int k=0; k<A_collision.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_collision, k); it; ++it) {
            triplet_list.emplace_back(row_offset + it.row(), it.col(), it.value());
        }
    }
    
    row_offset += A_collision.rows();
    // 添加 A_start
    for (int k=0; k<A_start.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_start, k); it; ++it) {
            triplet_list.emplace_back(row_offset + it.row(), it.col(), it.value());
        }
    }

    Eigen::SparseMatrix<double> A_total(row_offset + A_start.rows(), 3*point_num);
    A_total.setFromTriplets(triplet_list.begin(), triplet_list.end());

    Eigen::VectorXd low_boundary_total(A_total.rows());
    Eigen::VectorXd up_boundary_total(A_total.rows());
    
    low_boundary_total << low_boundary_continuity, low_boundary_collision, b_start;
    up_boundary_total << up_boundary_continuity, up_boundary_collision, b_start;
    
    // ==========================================
    // 3. 求解 (Solve)
    // ==========================================
    qp_solver_->clearSolver();
    qp_solver_->data()->clearHessianMatrix();
    qp_solver_->data()->clearLinearConstraintsMatrix();
    qp_solver_->settings()->setVerbosity(false);

    qp_solver_->data()->setNumberOfVariables(3*point_num);
    qp_solver_->data()->setNumberOfConstraints(A_total.rows());
    
    if(!qp_solver_->data()->setHessianMatrix(H)) {
        std::cerr << "[ERROR] Failed to set Hessian matrix" << std::endl;
        return false;
    }
    if(!qp_solver_->data()->setLinearConstraintsMatrix(A_total)) {
        std::cerr << "[ERROR] Failed to set Linear Constraints matrix" << std::endl;
        return false;
    }
    if(!qp_solver_->data()->setGradient(f)) return false;
    if(!qp_solver_->data()->setBounds(low_boundary_total, up_boundary_total)) return false;
    
    // Debug prints
    if(_enable_log){
        if (point_num > 0) {
            std::cout << "[DEBUG] QP Setup:" << std::endl;
            std::cout << "  b_start: " << b_start.transpose() << std::endl;
            std::cout << "  relax_num: " << relax_num << std::endl;
            if (!l_min.empty()) {
                std::cout << "  l_min[0]: " << l_min[0] << ", l_max[0]: " << l_max[0] << std::endl;
                if (l_min.size() > 1) std::cout << "  l_min[1]: " << l_min[1] << ", l_max[1]: " << l_max[1] << std::endl;
            }
            std::cout << "  low_boundary_collision[0]: " << low_boundary_collision[0] << std::endl;
            std::cout << "  up_boundary_collision[0]: " << up_boundary_collision[0] << std::endl;
        }
    }

    if(!qp_solver_->initSolver()) return false; 
    if(qp_solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

    // 提取结果
    auto solution = qp_solver_->getSolution();
    if (solution.size() != 3 * point_num) {
        log("QP solution size mismatch!", "ERROR");
        return false;
    }
    
    if (std::abs(solution[0]) > 100.0) {
        std::cout << "[DEBUG] QP Output Suspicious: solution[0] (l) = " << solution[0] << std::endl;
        std::cout << "[DEBUG] QP Output Suspicious: solution[1] (l') = " << solution[1] << std::endl;
        std::cout << "[DEBUG] QP Output Suspicious: solution[2] (l'') = " << solution[2] << std::endl;
        return false;
    }

    for (size_t i = 0; i < point_num; i++)
    {
        FrenetPoint qp_path_point;
        qp_path_point.s = dp_path_[i].s;
        qp_path_point.l = solution[3*i + 0];
        qp_path_point.l_prime = solution[3*i + 1];
        qp_path_point.l_prime_prime = solution[3*i + 2];
        qp_path_.emplace_back(qp_path_point);
    }

    // 清理
    qp_solver_->data()->clearHessianMatrix();
    qp_solver_->data()->clearLinearConstraintsMatrix();
    qp_solver_->clearSolverVariables();
    qp_solver_->clearSolver();
    
    return true;
    }

void PathPlanner::IncreasePathDensity(std::vector<general::FrenetPoint>& DP_or_QP,double interval){
        if (DP_or_QP.size() < 2) return;
        std::vector<general::FrenetPoint> dense_path;
        double sample_s = DP_or_QP[1].s - DP_or_QP[0].s;
        size_t num = std::max((size_t)(sample_s / interval), (size_t)1) + 1;
        // 使用多项式拟合进行路径加密
        for(size_t i=0;i<DP_or_QP.size()-1;++i){
            const auto& p0 = DP_or_QP[i];
            const auto& p1 = DP_or_QP[i+1];
            PolynomialCurve poly;
            poly.curve_fitting(
                p0.s, p0.l, p0.l_prime, p0.l_prime_prime,
                p1.s, p1.l, p1.l_prime, p1.l_prime_prime
            );
            auto linspace=Eigen::VectorXd::LinSpaced(
                num,p0.s, p1.s
            );// 会包含起点和终点，所以应该去重
            for(int j=0;j<linspace.size()-1;++j){
                double s = linspace[j];
                if(s>p1.s) s=p1.s;//防止数值误差超过终点
                general::FrenetPoint fp;
                fp.s = s;
                fp.l = poly.value_evaluation(s,0);
                fp.l_prime = poly.value_evaluation(s,1);
                fp.l_prime_prime = poly.value_evaluation(s,2);
                dense_path.push_back(fp);
            }
        }
        // 最后加上边界点
        dense_path.push_back(DP_or_QP[DP_or_QP.size()-1]);
        // std::cout<<"理论上会有:"<<config_.dense_path_interval*(dp_path_.size()-1)<<"个点"<<std::endl;
        DP_or_QP=dense_path; 
    }


void PathPlanner::generateConvexSpace(
    const std::vector<FrenetPoint>& static_obstacles,
    std::vector<double>& l_min,
    std::vector<double>& l_max) {
    
    double road_up = config_.lane_width * 1.5 - config_.safety_margin;
    double road_low = -config_.lane_width * 0.5 + config_.safety_margin;
    
    l_min.resize(dp_path_.size(), road_low);
    l_max.resize(dp_path_.size(), road_up);

    if (static_obstacles.empty()) {
        return;
    }
    double obs_length = 5.0;
    double obs_width = 2.0;
    
    // 简单的最近点查找函数
    auto findClosestIndex = [&](double target_s) -> int {
        if (dp_path_.empty()) return -1;
        if (target_s < dp_path_.front().s) return 0;
        if (target_s >= dp_path_.back().s) return dp_path_.size() - 1;
        
        for (size_t i = 0; i < dp_path_.size() - 1; i++) {
            if (target_s >= dp_path_[i].s && target_s < dp_path_[i+1].s) {
                return (std::abs(target_s - dp_path_[i].s) < std::abs(target_s - dp_path_[i+1].s)) ? i : i+1;
            }
        }
        return -1;
    };
    
    for (const auto& obs : static_obstacles) {
        int center_index = findClosestIndex(obs.s);
        if (center_index == -1) continue;
        
        std::cout << "[DEBUG] Obs at s=" << obs.s << ", l=" << obs.l 
                  << ", center_idx=" << center_index 
                  << ", dp_l=" << dp_path_[center_index].l << std::endl;

        if (dp_path_[center_index].l > obs.l) { // 左侧绕行
            int start_index = findClosestIndex(obs.s - obs_length/2.0);
            int end_index = findClosestIndex(obs.s + obs_length/2.0);
            
            std::cout << "[DEBUG] Left bypass. start=" << start_index << ", end=" << end_index << std::endl;

            if (start_index == -1 || end_index == -1) continue;
            
            for (int i = start_index; i <= end_index; i++) {
                l_min[i] = std::max(l_min[i], obs.l + obs_width/2.0 + config_.safety_margin);
            }
        } else { // 右侧绕行
            int start_index = findClosestIndex(obs.s - obs_length/2.0);
            int end_index = findClosestIndex(obs.s + obs_length/2.0);
            
            std::cout << "[DEBUG] Right bypass. start=" << start_index << ", end=" << end_index << std::endl;

            if (start_index == -1 || end_index == -1) continue;
            
            for (int i = start_index; i <= end_index; i++) {
                l_max[i] = std::min(l_max[i], obs.l - obs_width/2.0 - config_.safety_margin);
            }
        }
    }
}

} // namespace planner
} // namespace AD_algorithm