#include "general_modules/ReferenceLine.h"

#include <iostream>
#include <cmath>

namespace AD_algorithm {
namespace general {
        
ReferenceLine::ReferenceLine(const std::vector<std::pair<double, double>>& xy_points,bool smooth)
    : _smooth_solver(std::make_shared<OsqpEigen::Solver>()) {
    // 初始化求解器设置
    _smooth_solver->settings()->setWarmStart(true);
    _smooth_solver->settings()->setVerbosity(false);
    // 填充原始点
    _ref_line.reserve(xy_points.size());
    for (const auto& pt : xy_points) {
        _ref_line.emplace_back();
        _ref_line.back().x = pt.first;
        _ref_line.back().y = pt.second;
    }
    if(smooth){
        if(smooth_reference_line()){
            // std::cout<<"参考线平滑成功\n";
        }
        else{
            std::cout<<"参考线平滑失败\n";
        }
    }

    if(ComputePathProfile()){
        _flag=true;
        // std::cout<<"参考线初始化完成\n";
    }
}

ReferenceLine::ReferenceLine(const std::vector<PathPoint>& ref_line,bool smooth)
    : _smooth_solver(std::make_shared<OsqpEigen::Solver>()) {
    // 初始化求解器设置
    _smooth_solver->settings()->setWarmStart(true);
    _smooth_solver->settings()->setVerbosity(false);
    // 直接使用传入的参考线点
    _ref_line = ref_line;

    if(smooth){
        if(smooth_reference_line()){
            // std::cout<<"参考线平滑成功\n";
        }
        else{
            std::cout<<"参考线平滑失败\n";
        }
    }

    // DEBUG: Print smoothed reference line around index 93
    if (_ref_line.size() > 100) {
        std::cout << "[DEBUG] Smoothed Reference Line around index 93:" << std::endl;
        for (size_t i = 90; i <= 95; ++i) {
            std::cout << "i=" << i << " x=" << _ref_line[i].x << " y=" << _ref_line[i].y << std::endl;
        }
    }

    if(ComputePathProfile()){
        _flag=true;
        // std::cout<<"参考线初始化完成\n";
    }
}


bool ReferenceLine::flag_ref() const {
    // 检查参考线是否有效：至少需要2个点才能构成路径
    return _flag;
}

bool ReferenceLine::ComputePathProfile() {
    bool flag=false;
    // 如果点数少于2，无法计算几何属性
    if (_ref_line.size() < 2) {
        std::cerr << "Warning: Not enough points to compute path profile." << std::endl;
        return flag;
    }

    const size_t n = _ref_line.size();
    
    // 1. 计算累积距离（accumulated_s）
    _ref_line[0].accumulated_s = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double dx = _ref_line[i].x - _ref_line[i-1].x;
        double dy = _ref_line[i].y - _ref_line[i-1].y;
        double segment_length = std::hypot(dx, dy); // 使用hypot避免溢出
        _ref_line[i].accumulated_s = _ref_line[i-1].accumulated_s + segment_length;
    }

    // 2. 计算航向角（heading）
    // 使用中心差分法，边界使用前向/后向差分
    for (size_t i = 0; i < n; ++i) {
        double dx, dy;
        
        if (i == 0) {
            // 起点：前向差分
            dx = _ref_line[i+1].x - _ref_line[i].x;
            dy = _ref_line[i+1].y - _ref_line[i].y;
        } else if (i == n - 1) {
            // 终点：后向差分
            dx = _ref_line[i].x - _ref_line[i-1].x;
            dy = _ref_line[i].y - _ref_line[i-1].y;
        } else {
            // 中间点：中心差分（精度更高）
            dx = 0.5 * (_ref_line[i+1].x - _ref_line[i-1].x);
            dy = 0.5 * (_ref_line[i+1].y - _ref_line[i-1].y);
        }
        
        _ref_line[i].heading = std::atan2(dy, dx);
    }

    // 3. 计算一阶导数（dx/ds, dy/ds）
    std::vector<double> dx_ds(n, 0.0);
    std::vector<double> dy_ds(n, 0.0);
    
    for (size_t i = 0; i < n; ++i) {
        if (i == 0) {
            // 起点：前向差分
            dx_ds[i] = (_ref_line[i+1].x - _ref_line[i].x) / 
                      (_ref_line[i+1].accumulated_s - _ref_line[i].accumulated_s);
            dy_ds[i] = (_ref_line[i+1].y - _ref_line[i].y) / 
                      (_ref_line[i+1].accumulated_s - _ref_line[i].accumulated_s);
        } else if (i == n - 1) {
            // 终点：后向差分
            dx_ds[i] = (_ref_line[i].x - _ref_line[i-1].x) / 
                      (_ref_line[i].accumulated_s - _ref_line[i-1].accumulated_s);
            dy_ds[i] = (_ref_line[i].y - _ref_line[i-1].y) / 
                      (_ref_line[i].accumulated_s - _ref_line[i-1].accumulated_s);
        } else {
            // 中间点：中心差分
            dx_ds[i] = (_ref_line[i+1].x - _ref_line[i-1].x) / 
                      (_ref_line[i+1].accumulated_s - _ref_line[i-1].accumulated_s);
            dy_ds[i] = (_ref_line[i+1].y - _ref_line[i-1].y) / 
                      (_ref_line[i+1].accumulated_s - _ref_line[i-1].accumulated_s);
        }
    }

    // 4. 计算二阶导数（d²x/ds², d²y/ds²）
    std::vector<double> d2x_ds2(n, 0.0);
    std::vector<double> d2y_ds2(n, 0.0);
    
    for (size_t i = 0; i < n; ++i) {
        if (i == 0) {
            // 起点：前向差分
            d2x_ds2[i] = (dx_ds[i+1] - dx_ds[i]) / 
                        (_ref_line[i+1].accumulated_s - _ref_line[i].accumulated_s);
            d2y_ds2[i] = (dy_ds[i+1] - dy_ds[i]) / 
                        (_ref_line[i+1].accumulated_s - _ref_line[i].accumulated_s);
        } else if (i == n - 1) {
            // 终点：后向差分
            d2x_ds2[i] = (dx_ds[i] - dx_ds[i-1]) / 
                        (_ref_line[i].accumulated_s - _ref_line[i-1].accumulated_s);
            d2y_ds2[i] = (dy_ds[i] - dy_ds[i-1]) / 
                        (_ref_line[i].accumulated_s - _ref_line[i-1].accumulated_s);
        } else {
            // 中间点：中心差分
            d2x_ds2[i] = (dx_ds[i+1] - dx_ds[i-1]) / 
                        (_ref_line[i+1].accumulated_s - _ref_line[i-1].accumulated_s);
            d2y_ds2[i] = (dy_ds[i+1] - dy_ds[i-1]) / 
                        (_ref_line[i+1].accumulated_s - _ref_line[i-1].accumulated_s);
        }
    }

    // 5. 计算曲率（kappa）
    const double epsilon = 1e-6; // 小量防止除零
    for (size_t i = 0; i < n; ++i) {
        double x_prime = dx_ds[i];
        double y_prime = dy_ds[i];
        double x_double_prime = d2x_ds2[i];
        double y_double_prime = d2y_ds2[i];
        
        // 曲率公式：κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
        double numerator = x_prime * y_double_prime - y_prime * x_double_prime;
        double denominator = std::pow(x_prime * x_prime + y_prime * y_prime, 1.5);
        
        _ref_line[i].kappa = (std::abs(denominator) > epsilon) ? 
                            numerator / denominator : 0.0;
    }

    // 6. 计算曲率变化率（kappa_rate）
    for (size_t i = 0; i < n; ++i) {
        if (i == 0) {
            // 起点：前向差分
            _ref_line[i].kappa_rate = (_ref_line[i+1].kappa - _ref_line[i].kappa) / 
                                     (_ref_line[i+1].accumulated_s - _ref_line[i].accumulated_s);
        } else if (i == n - 1) {
            // 终点：后向差分
            _ref_line[i].kappa_rate = (_ref_line[i].kappa - _ref_line[i-1].kappa) / 
                                     (_ref_line[i].accumulated_s - _ref_line[i-1].accumulated_s);
        } else {
            // 中间点：中心差分
            _ref_line[i].kappa_rate = (_ref_line[i+1].kappa - _ref_line[i-1].kappa) / 
                                     (_ref_line[i+1].accumulated_s - _ref_line[i-1].accumulated_s);
        }
    }
    
    // 可选：对heading进行规范化，使其在[-π, π]范围内
    for (auto& point : _ref_line) {
        // 使用fmod将角度规范化到[-π, π]
        point.heading = fmod(point.heading + M_PI, 2.0 * M_PI) - M_PI;
    }

    return true;
}

 
bool ReferenceLine::smooth_reference_line() {
    const size_t n = _ref_line.size();
    if (n < 3) {
        // 点太少，无法构建平滑项，直接返回成功
        return true;
    }

    // 1. 构建变量：z = [x0, x1, ..., xn-1, y0, y1, ..., yn-1]^T
    const int num_vars = 2 * static_cast<int>(n);

    // 2. 构建 Hessian 矩阵 P = 2*(Q_smooth + Q_compact + Q_geometry)
    Eigen::SparseMatrix<double> H_smooth, H_compact, H_geometry, P;
    std::vector<Eigen::Triplet<double>> triplets;

    // ---- 平滑项: (x_{i-1} - 2x_i + x_{i+1})^2 for i=1..n-2 ----
    for (int i = 1; i < static_cast<int>(n) - 1; ++i) {
        // x 部分
        triplets.push_back({2*i - 2, 2*i - 2, 1.0});   // x_{i-1}
        triplets.push_back({2*i - 2, 2*i,     -2.0});  // x_i
        triplets.push_back({2*i - 2, 2*i + 2, 1.0});   // x_{i+1}
        // y 部分（偏移 n）
        triplets.push_back({2*i - 1, 2*i - 1, 1.0});
        triplets.push_back({2*i - 1, 2*i + 1, -2.0});
        triplets.push_back({2*i - 1, 2*i + 3, 1.0});
    }
    H_smooth.resize(num_vars, num_vars);
    H_smooth.setFromTriplets(triplets.begin(), triplets.end());

    // ---- 紧凑项: (x_{i+1} - x_i)^2 for i=0..n-2 ----
    triplets.clear();
    for (int i = 0; i < static_cast<int>(n) - 1; ++i) {
        // x: -x_i + x_{i+1}
        triplets.push_back({2*i,     2*i,     1.0});
        triplets.push_back({2*i,     2*i + 2, -1.0});
        triplets.push_back({2*i + 2, 2*i,     -1.0});
        triplets.push_back({2*i + 2, 2*i + 2, 1.0});

        // y
        triplets.push_back({2*i + 1,     2*i + 1,     1.0});
        triplets.push_back({2*i + 1,     2*i + 3, -1.0});
        triplets.push_back({2*i + 3, 2*i + 1,     -1.0});
        triplets.push_back({2*i + 3, 2*i + 3, 1.0});
    }
    H_compact.resize(num_vars, num_vars);
    H_compact.setFromTriplets(triplets.begin(), triplets.end());

    // ---- 几何保真项: (x_i - x̄_i)^2 ----
    triplets.clear();
    for (int i = 0; i < num_vars; ++i) {
        triplets.push_back({i, i, 1.0});
    }
    H_geometry.resize(num_vars, num_vars);
    H_geometry.setFromTriplets(triplets.begin(), triplets.end());

    // 组合 Hessian: P = 2 * (w_s * H_s^T H_s + w_c * H_c^T H_c + w_g * H_g^T H_g)
    // 注意：H_smooth 和 H_compact 是“差分矩阵 D”，所以 Q = D^T D
    P = 2.0 * (_cost_smooth * (H_smooth.transpose() * H_smooth) +
               _cost_compact * (H_compact.transpose() * H_compact) +
               _cost_geometry * (H_geometry.transpose() * H_geometry));

    // 3. 构建线性项 q = -2 * w_g * [x̄0, ..., ȳ_{n-1}]^T
    Eigen::VectorXd q(num_vars);
    for (size_t i = 0; i < n; ++i) {
        q[2*i]     = -2.0 * _cost_geometry * _ref_line[i].x;
        q[2*i + 1] = -2.0 * _cost_geometry * _ref_line[i].y;
    }

    // 4. 构建不等式约束: A*z <= u, A*z >= l
    // 这里 A = I, l = p̄ - offset, u = p̄ + offset
    Eigen::SparseMatrix<double> A(num_vars, num_vars);
    Eigen::VectorXd lower_bound(num_vars), upper_bound(num_vars);
    std::vector<Eigen::Triplet<double>> A_triplets;
    for (int i = 0; i < num_vars; ++i) {
        A_triplets.push_back({i, i, 1.0});
        if (i % 2 == 0) { // x
            size_t idx = i / 2;
            lower_bound[i] = _ref_line[idx].x - _max_x_offset;
            upper_bound[i] = _ref_line[idx].x + _max_x_offset;
        } else { // y
            size_t idx = (i - 1) / 2;
            lower_bound[i] = _ref_line[idx].y - _max_y_offset;
            upper_bound[i] = _ref_line[idx].y + _max_y_offset;
        }
    }
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // 5. 设置 OSQP 求解器
    _smooth_solver->data()->setNumberOfVariables(num_vars);
    _smooth_solver->data()->setNumberOfConstraints(num_vars);

    if (!_smooth_solver->data()->setHessianMatrix(P)) {
        std::cerr << "[smooth_reference_line] Failed to set Hessian." << std::endl;
        return false;
    }
    if (!_smooth_solver->data()->setGradient(q)) {
        std::cerr << "[smooth_reference_line] Failed to set gradient." << std::endl;
        return false;
    }
    if (!_smooth_solver->data()->setLinearConstraintsMatrix(A)) {
        std::cerr << "[smooth_reference_line] Failed to set constraint matrix." << std::endl;
        return false;
    }
    if (!_smooth_solver->data()->setLowerBound(lower_bound)) {
        std::cerr << "[smooth_reference_line] Failed to set lower bound." << std::endl;
        return false;
    }
    if (!_smooth_solver->data()->setUpperBound(upper_bound)) {
        std::cerr << "[smooth_reference_line] Failed to set upper bound." << std::endl;
        return false;
    }

    // 6. 求解
    if (!_smooth_solver->initSolver()) {
        std::cerr << "[smooth_reference_line] Failed to initialize solver." << std::endl;
        return false;
    }

    if (!_smooth_solver->solve()) {
        std::cerr << "[smooth_reference_line] OSQP solve failed." << std::endl;
        return false;
    }

    // 7. 提取解并更新 _ref_line
    Eigen::VectorXd solution = _smooth_solver->getSolution();
    for (size_t i = 0; i < n; ++i) {
        _ref_line[i].x = solution[2*i];
        _ref_line[i].y = solution[2*i + 1];
    }

    // 8. 清理（可选，避免内存累积）
    _smooth_solver->clearSolver();

    // 9. 重要：重新计算 heading, kappa 等几何属性！
    return true;
}


} // namespace general
} // namespace AD_algorithm
