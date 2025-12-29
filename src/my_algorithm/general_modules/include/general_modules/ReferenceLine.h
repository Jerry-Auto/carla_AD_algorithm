#ifndef REFERENCE_LINE_H_
#define REFERENCE_LINE_H_
#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Sparse>
#include "general_modules/common_types.h"

namespace AD_algorithm {
namespace general {
class ReferenceLine {
public:

    ReferenceLine(const std::vector<std::pair<double, double>>& xy_points,bool smooth=true);
    ReferenceLine(const std::vector<PathPoint>& ref_line,bool smooth=true);
    ~ReferenceLine() = default;
    bool flag_ref() const;
    const std::vector<PathPoint>& get_path_points() const { return _ref_line; }
    const PathPoint& operator[](size_t index) const {
        // 可选：调试时启用 assert
        // assert(index < path_points_.size());
        return _ref_line[index];
    }
    size_t size() const {
        return _ref_line.size();
    }
    
private:
    bool smooth_reference_line();
    bool ComputePathProfile();
    // ===== 新增：平滑相关参数与求解器 =====
    std::shared_ptr<OsqpEigen::Solver> _smooth_solver;
    double _cost_smooth = 5.0;      // 平滑代价权重
    double _cost_geometry = 10.0;   // 几何保真权重
    double _cost_compact = 50.0;    // 紧凑代价权重
    double _max_x_offset = 0.5;     // x方向最大偏移
    double _max_y_offset = 0.5;     // y方向最大偏移
    
    std::vector<PathPoint> _ref_line;
    bool _flag = false;
};
} 
} 
#endif