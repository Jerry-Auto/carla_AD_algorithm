#include "emplanner/path_planner.h"
#include "emplanner/DP_solver.h"
#include "general_modules/polynomial_curve.h"
#include <algorithm>
#include <chrono>

namespace AD_algorithm {
namespace planner {

using namespace general;

PathPlanner::PathPlanner(const WeightCoefficients& weights, const PathPlannerConfig& config, 
                         std::shared_ptr<general::Logger> logger) 
    : weights_(weights), config_(config), logger_(logger) {
    qp_solver_ = std::make_shared<OsqpEigen::Solver>();
    qp_solver_->settings()->setWarmStart(true);
}

void PathPlanner::set_road_width(const std::shared_ptr<AD_algorithm::general::VehicleState>& vehicle_state,double offset_s) {
    _road_width_left_vec.clear();
    _road_width_right_vec.clear();
    // 假如是空的，或者数量不足，那么设置固定车道宽度
    if(!vehicle_state || vehicle_state->road_width_left_vec.size() < 200 || vehicle_state->road_width_right_vec.size() < 200){
        _road_width_left_vec = std::vector<double>(200,config_.lane_width/2);
        // 注意：该工程约定右侧宽度为负值
        _road_width_right_vec = std::vector<double>(200,-config_.lane_width/2);
        _road_width_resolution = 1.0;
        return;
    }
    size_t start_index = static_cast<size_t>(offset_s / vehicle_state->road_width_resolution);
    // 从车辆状态中提取道路宽度信息，考虑偏移
    for (size_t i = start_index; i < vehicle_state->road_width_left_vec.size(); ++i) {
        double left_width = vehicle_state->road_width_left_vec[i];
        double right_width = vehicle_state->road_width_right_vec[i];
        // 留出车宽和安全边界
        if(left_width<6.0&&right_width>-6.0){
            //两侧同时很小，说明是窄道，直接使用原始值
            _road_width_left_vec.push_back(config_.lane_width/2.0); 
            _road_width_right_vec.push_back(-config_.lane_width/2.0);
            continue;
        }
        if(left_width<6.0){
            _road_width_left_vec.push_back(1.0);
        }else{
            _road_width_left_vec.push_back(left_width - 5.0);
        }
        if(right_width>-6.0){
            _road_width_right_vec.push_back(-1.0);
        }else{
            _road_width_right_vec.push_back(right_width + 5.0);
        }
    }
    _road_width_resolution = vehicle_state->road_width_resolution;
}

std::vector<TrajectoryPoint> PathPlanner::planPath(
    const std::shared_ptr<FrenetFrame>& frenet_frame,
    const FrenetPoint& planning_start_point,
    const std::vector<std::vector<FrenetPoint>>& static_obstacles
    ) {
    // 注意，传入的planning_start_point的s是相对于全局frenet坐标系的原点来计算的
    // static_obstacles也是相对于全局坐标系的S
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 验证配置
    if (!config_.validate()) {
        log("ERROR", "Invalid configuration");
        return {};
    }

    if (!frenet_frame) {
        log("ERROR", "Frenet frame is null");
        return {};
    }
    
    log("INFO", "PathPlanner Config: s_step=" + std::to_string(config_.s_sample_distance) + 
        ", s_num=" + std::to_string(config_.s_sample_number));

    // 清除旧的路径
    dp_path_.clear();
    trajectory_.clear();
    
    // log("INFO", "Starting path planning with " + std::to_string(static_obstacles.size()) + " static obstacles");
    
    // 复制起始点偏移量，并确保起点s坐标为0
    // S偏移了，但是轨迹点和障碍物的全局x,y都是没变的
    FrenetPoint start_point = planning_start_point;
    double offset_s = start_point.s;
    start_point.s = 0.0;
    
    // log("INFO", "Planning start point: s=" + std::to_string(start_point.s) + 
    //     ", l=" + std::to_string(start_point.l));
    
    // 1. 使用DP进行路径搜索，起点的s坐标必须为0
    log("INFO", "Running DP path planning...");

    SLState start_state(start_point.s, start_point.l, 
                       start_point.l_prime, start_point.l_prime_prime);
    
    log("DEBUG", "DP Start State: s=", start_state.s,
        ", l=", start_state.l,
        ", l'=", start_state.l_prime,
        ", l''=", start_state.l_prime_prime);

    // 转换障碍物
    std::vector<std::vector<FrenetPoint>> local_frenet_obstacles;
    for (const auto& obs_corners : static_obstacles) {
        std::vector<FrenetPoint> corners;
        for (const auto& cp : obs_corners) {
            FrenetPoint p = cp;
            p.s -= offset_s;
            corners.push_back(p);
        }
        local_frenet_obstacles.push_back(corners);
    }
    
    std::vector<SLObstacle> sl_obstacles = FrenetFrame::convertToSLObstacles(local_frenet_obstacles, weights_.obs_safety_margin);
    
    // 创建策略
    auto cost_function = std::make_shared<PathCostFunction>(weights_, config_, sl_obstacles);
    auto constraint_checker = std::make_shared<PathConstraintChecker>(
        config_.lane_width * 1.5,//道路边界上界，相对于参考线对应的frenet中心线
        -config_.lane_width * 0.5,//道路边界下界，相对于参考线对应的frenet中心线
        sl_obstacles
    );
    auto sampling_strategy = std::make_shared<PathSamplingStrategy>(config_, _road_width_left_vec, _road_width_right_vec, _road_width_resolution);
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

    // // 可以使用预设采样网格的接口
    // if(!dp_sampling_grid_){
    //     // 只有第一次规划时生成采样网格，后续复用
    //     dp_sampling_grid_ = std::make_shared<std::vector<std::vector<SLState>>>(
    //         sampling_strategy->generateSamplingGrid(start_state, config_.s_sample_number)
    //     );
    // }

    // auto result = dp_planner.planWithGrid(*dp_sampling_grid_, start_state);
    auto result = dp_planner.plan(start_state, config_.s_sample_number);//每次都重新生成采样网格，较慢
    
    if (!result.success) {
        log("ERROR", "DP planning failed: " + result.message);
        return {};
    }

    
    // dp_planner.printSearchTree([](const SLState& state) {
    //     std::cout << "s=" << std::setw(6) << state.s 
    //               << ", l=" << std::setw(6) << state.l 
    //               << ", l'=" << std::setw(6) << state.l_prime
    //               << ", l''=" << std::setw(6) << state.l_prime_prime;
    // });

    log("DP planning succeeded, found path with " + std::to_string(result.optimal_path.size()) + " points");
    log("INFO", "Total cost: " + std::to_string(result.total_cost));
    log("INFO", "Computation time: " + std::to_string(result.computation_time_ms) + " ms");
    
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
    log("INFO", "Increased path density to " + std::to_string(dp_path_.size()) + " points");

    // 2. 生成凸空间
    log("INFO", "Generating convex space...");
    std::vector<double> l_min, l_max;

    std::vector<std::vector<FrenetPoint>> local_static_obstacles;
    // 上面的SL障碍物是给DP用的，这里的还没有转换，需要转换到局部坐标系
    for (const auto& obs_corners : static_obstacles) {
        std::vector<FrenetPoint> local_corners;
        for (const auto& corner : obs_corners) {
            FrenetPoint local_corner = corner;
            local_corner.s -= offset_s;
            local_corners.push_back(local_corner);
        }
        local_static_obstacles.push_back(local_corners);
    }

    // 最近障碍物决策锁定：在调用 generateConvexSpace() 前更新锁定状态
    updateNearestObsDecision(offset_s, static_obstacles, local_static_obstacles);

    generateConvexSpace(local_static_obstacles, l_min, l_max);
    
    // 3. QP路径优化
    log("INFO", "Running QP path optimization...");
    QP_pathOptimization(l_min, l_max);
    
    if (qp_path_.empty()) {
        log("ERROR", "QP optimization failed: No path generated");
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
    log("INFO", "Converting to Cartesian coordinates...");
    trajectory_ = frenet_frame->frenet_to_cartesian(qp_path_);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    log("INFO", "Path planning completed in " + std::to_string(duration.count()) + " ms");
    log("INFO", "Generated " + std::to_string(trajectory_.size()) + " trajectory points");
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
            log("DEBUG", "Infeasible constraint at index", i,
                ": lbi=", lbi, ", ubi=", ubi,
                ", width=", car_width,
                ", start_idx=", start_index, ", end_idx=", end_index);
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
    if (relax_num > static_cast<int>(point_num)) relax_num = static_cast<int>(point_num); // Cast point_num to int
    
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
        log("DEBUG", "QP Input Suspicious: dp_path_[0].l =", dp_path_.front().l);
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
        log("ERROR", "Failed to set Hessian matrix");
        return false;
    }
    if(!qp_solver_->data()->setLinearConstraintsMatrix(A_total)) {
        log("ERROR", "Failed to set Linear Constraints matrix");
        return false;
    }
    if(!qp_solver_->data()->setGradient(f)) return false;
    if(!qp_solver_->data()->setBounds(low_boundary_total, up_boundary_total)) return false;
    
    // Debug prints
    if (point_num > 0) {
        log("DEBUG", "QP Setup:");
        log("DEBUG", "b_start:", b_start.transpose());
        log("DEBUG", "relax_num:", relax_num);
        if (!l_min.empty()) {
            log("DEBUG", "l_min[0]:", l_min[0], ", l_max[0]:", l_max[0]);
            if (l_min.size() > 1) {
                log("DEBUG", "l_min[1]:", l_min[1], ", l_max[1]:", l_max[1]);
            }
        }
        log("DEBUG", "low_boundary_collision[0]:", low_boundary_collision[0]);
        log("DEBUG", "up_boundary_collision[0]:", up_boundary_collision[0]);
    }

    if(!qp_solver_->initSolver()) return false; 
    if(qp_solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

    // 提取结果
    Eigen::VectorXd solution = qp_solver_->getSolution();
    if (static_cast<size_t>(solution.size()) != 3 * point_num) {
        log("ERROR", "QP solution size mismatch!");
        return false;
    }
    
    if (solution.size() >= 3 && std::abs(solution[0]) > 100.0) {
        log("DEBUG", "QP Output Suspicious: solution[0] (l) =", solution[0]);
        log("DEBUG", "QP Output Suspicious: solution[1] (l') =", solution[1]);
        log("DEBUG", "QP Output Suspicious: solution[2] (l'') =", solution[2]);
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
            for(int j=0;j<static_cast<int>(linspace.size())-1;++j){
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
    const std::vector<std::vector<general::FrenetPoint>>& static_obstacles,
    std::vector<double>& l_min,
    std::vector<double>& l_max) {
    // 约定：_road_width_left_vec / _road_width_right_vec 的起点与规划起点对齐，分辨率约为 1m。
    // 障碍物是frenet坐标系下的点集，每个障碍物由多个角点组成，已经转换为局部坐标系，起点是规划起点

    // 道路边界：使用 road width 向量按 s 查找（线性插值）
    // 其中 left/right 存储的是“参考线到左右边界的距离”，左边是正的，右边是负的
    const double resolution = (_road_width_resolution > 1e-6) ? _road_width_resolution : 1.0;
    auto lerp = [](double a, double b, double t) { return a + (b - a) * t; };

    auto widthAtS = [&](const std::vector<double>& vec, double s, double default_value) -> double {
        if (vec.empty()) return default_value;
        if (s <= 0.0) return vec.front();

        const double idx_f = s / resolution;
        const int idx0 = static_cast<int>(std::floor(idx_f));
        const int idx1 = idx0 + 1;
        const double t = idx_f - static_cast<double>(idx0);

        if (idx0 < 0) return vec.front();
        if (idx0 >= static_cast<int>(vec.size()) - 1) return vec.back();
        return lerp(vec[idx0], vec[idx1], t);
    };

    l_min.resize(dp_path_.size());
    l_max.resize(dp_path_.size());

    for (size_t i = 0; i < dp_path_.size(); ++i) {
        const double s = dp_path_[i].s;
        // 注意：该工程约定 left 宽度为正，right 宽度为负
        const double left_w = widthAtS(_road_width_left_vec, s, config_.lane_width * 0.5);
        const double right_w = widthAtS(_road_width_right_vec, s, -config_.lane_width * 0.5);

        // Frenet：l>0 在左侧，l<0 在右侧
        // right_w 本身为负，所以道路下界应直接取 right_w 再加 obs_safety_margin 向内收缩
        const double road_up = left_w - weights_.obs_safety_margin;
        const double road_low = right_w + weights_.obs_safety_margin;

        // 防御：如果道路宽度数据异常导致上下界反转，退化为零宽并避免 NaN
        if (road_low <= road_up) {
            l_min[i] = road_low;
            l_max[i] = road_up;
        } else {
            const double mid = 0.5 * (road_low + road_up);
            l_min[i] = mid;
            l_max[i] = mid;
        }
        // log("DEBUG", "s=", s,", l_min=", l_min[i],", l_max=", l_max[i]);
    }

    if (static_obstacles.empty()) {
        return;
    }

    // 计算“最近障碍物”（只考虑 s>=0 的前向障碍物）
    double nearest_obs_s = std::numeric_limits<double>::infinity();
    for (const auto& obs_corners : static_obstacles) {
        if (obs_corners.empty()) continue;
        double min_s = std::numeric_limits<double>::max();
        for (const auto& p : obs_corners) min_s = std::min(min_s, p.s);
        
        if (min_s >= 0.0 && min_s < nearest_obs_s) {
            nearest_obs_s = min_s;
        }
    }
    const bool has_nearest_obs = std::isfinite(nearest_obs_s);

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
    
    for (const auto& obs_corners : static_obstacles) {
        if (obs_corners.empty()) continue;
        
        double s_min = std::numeric_limits<double>::max();
        double s_max = std::numeric_limits<double>::lowest();
        double l_min_obs = std::numeric_limits<double>::max();
        double l_max_obs = std::numeric_limits<double>::lowest();
        double s_center = 0.0;
        double l_center = 0.0;

        for (const auto& p : obs_corners) {
            s_min = std::min(s_min, p.s);
            s_max = std::max(s_max, p.s);
            l_min_obs = std::min(l_min_obs, p.l);
            l_max_obs = std::max(l_max_obs, p.l);
            s_center += p.s;
            l_center += p.l;
        }
        s_center /= obs_corners.size();
        l_center /= obs_corners.size();

        int center_index = findClosestIndex(s_center);
        if (center_index == -1) continue;
        
        log("DEBUG", "Obs at s_center=", s_center,
            ", l_center=", l_center,
            ", center_idx=", center_index,
            ", dp_l=", dp_path_[center_index].l);

        // 对“最近障碍物”使用锁定决策（近5次多数投票后锁定），其他障碍物仍按当前DP相对位置决定
        NearestObsDecision decision = NearestObsDecision::Unknown;
        if (has_nearest_obs && std::abs(s_min - nearest_obs_s) < 1e-3 && locked_nearest_obs_decision_ != NearestObsDecision::Unknown) {
            decision = locked_nearest_obs_decision_;
        } else {
            decision = (dp_path_[center_index].l > l_center) ? NearestObsDecision::BypassLeft : NearestObsDecision::BypassRight;
        }

        double longitudinal_buffer = 10.0; 
        int start_index = findClosestIndex(s_min - longitudinal_buffer);
        int end_index = findClosestIndex(s_max + longitudinal_buffer);
        
        if (start_index == -1 || end_index == -1) continue;

        if (decision == NearestObsDecision::BypassLeft) { // 左侧绕行
            log("DEBUG", "Left bypass. start=", start_index, ", end=", end_index);
            for (int i = start_index; i <= end_index; i++) {
                l_min[i] = std::max(l_min[i], l_max_obs + weights_.obs_safety_margin);
            }
        } else { // 右侧绕行
            log("DEBUG", "Right bypass. start=", start_index, ", end=", end_index);
            for (int i = start_index; i <= end_index; i++) {
                l_max[i] = std::min(l_max[i], l_min_obs - weights_.obs_safety_margin);
            }
        }
    }
}

void PathPlanner::resetNearestObsDecision() {
    nearest_obs_decision_history_.clear();
    locked_nearest_obs_decision_ = NearestObsDecision::Unknown;
    locked_nearest_obs_finalized_ = false;
    locked_nearest_obs_global_s_ = std::numeric_limits<double>::quiet_NaN();
}

void PathPlanner::updateNearestObsDecision(
    double offset_s,
    const std::vector<std::vector<general::FrenetPoint>>& static_obstacles_global,
    const std::vector<std::vector<general::FrenetPoint>>& static_obstacles_local) {

    if (dp_path_.empty()) return;

    // 找到最近的前向障碍物（local s >= 0）
    int nearest_idx = -1;
    double nearest_local_s = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < static_obstacles_local.size(); ++i) {
        const auto& obs_corners = static_obstacles_local[i];
        if (obs_corners.empty()) continue;
        
        double min_s = std::numeric_limits<double>::max();
        for (const auto& p : obs_corners) min_s = std::min(min_s, p.s);

        if (min_s >= 0.0 && min_s < nearest_local_s) {
            nearest_local_s = min_s;
            nearest_idx = static_cast<int>(i);
        }
    }

    // 没有前向障碍物：清空锁定
    if (nearest_idx < 0 || !std::isfinite(nearest_local_s)) {
        if (!nearest_obs_decision_history_.empty() || locked_nearest_obs_finalized_) {
            resetNearestObsDecision();
        }
        return;
    }

    double nearest_global_s = 0.0;
    if (nearest_idx < static_cast<int>(static_obstacles_global.size())) {
        const auto& obs_corners_global = static_obstacles_global[nearest_idx];
        double min_s_global = std::numeric_limits<double>::max();
        for (const auto& p : obs_corners_global) min_s_global = std::min(min_s_global, p.s);
        nearest_global_s = min_s_global;
    } else {
        nearest_global_s = offset_s + nearest_local_s;
    }

    // 如果已越过当前锁定障碍物：重置并重新计数
    const double pass_clearance_s = 2.0;
    if (std::isfinite(locked_nearest_obs_global_s_)) {
        if (offset_s - locked_nearest_obs_global_s_ > pass_clearance_s) {
            resetNearestObsDecision();
        }
    }

    // 如果当前最近障碍物与锁定对象差异很大（例如场景切换/障碍物列表突变），也重置
    const double switch_threshold_s = 2.0;
    if (std::isfinite(locked_nearest_obs_global_s_) && std::abs(nearest_global_s - locked_nearest_obs_global_s_) > switch_threshold_s) {
        resetNearestObsDecision();
    }

    // 初始化锁定对象
    if (!std::isfinite(locked_nearest_obs_global_s_)) {
        locked_nearest_obs_global_s_ = nearest_global_s;
    }

    // 若已锁定最终决策，则不再更新历史，保持决策不变直到越过该障碍物
    if (locked_nearest_obs_finalized_) {
        return;
    }

    // 计算本次对最近障碍物的“原始决策”（基于DP路径相对位置）
    auto findClosestIndex = [&](double target_s) -> int {
        if (dp_path_.empty()) return -1;
        if (target_s < dp_path_.front().s) return 0;
        if (target_s >= dp_path_.back().s) return static_cast<int>(dp_path_.size()) - 1;

        for (size_t i = 0; i + 1 < dp_path_.size(); i++) {
            if (target_s >= dp_path_[i].s && target_s < dp_path_[i + 1].s) {
                return (std::abs(target_s - dp_path_[i].s) < std::abs(target_s - dp_path_[i + 1].s)) ? static_cast<int>(i)
                                                                                                       : static_cast<int>(i + 1);
            }
        }
        return -1;
    };

    const auto& obs_corners = static_obstacles_local[nearest_idx];
    double s_center = 0.0;
    double l_center = 0.0;
    for (const auto& p : obs_corners) {
        s_center += p.s;
        l_center += p.l;
    }
    s_center /= obs_corners.size();
    l_center /= obs_corners.size();

    const int center_index = findClosestIndex(s_center);
    if (center_index < 0) return;

    const NearestObsDecision raw_decision = (dp_path_[center_index].l > l_center)
                                                ? NearestObsDecision::BypassLeft
                                                : NearestObsDecision::BypassRight;

    nearest_obs_decision_history_.push_back(raw_decision);
    while (nearest_obs_decision_history_.size() > kNearestObsDecisionWindow) {
        nearest_obs_decision_history_.pop_front();
    }

    // 多数投票
    int left_cnt = 0;
    int right_cnt = 0;
    for (const auto& d : nearest_obs_decision_history_) {
        if (d == NearestObsDecision::BypassLeft) left_cnt++;
        if (d == NearestObsDecision::BypassRight) right_cnt++;
    }

    if (left_cnt > right_cnt) {
        locked_nearest_obs_decision_ = NearestObsDecision::BypassLeft;
    } else if (right_cnt > left_cnt) {
        locked_nearest_obs_decision_ = NearestObsDecision::BypassRight;
    } else {
        // 平票：使用本次原始决策
        locked_nearest_obs_decision_ = raw_decision;
    }

    // 历史满5次后锁定
    if (nearest_obs_decision_history_.size() >= kNearestObsDecisionWindow) {
        locked_nearest_obs_finalized_ = true;
        log("DEBUG", "Nearest obstacle decision locked: global_s=", locked_nearest_obs_global_s_,
            ", decision=", (locked_nearest_obs_decision_ == NearestObsDecision::BypassLeft ? "LEFT" : "RIGHT"));
    }
}

} // namespace planner
} // namespace AD_algorithm