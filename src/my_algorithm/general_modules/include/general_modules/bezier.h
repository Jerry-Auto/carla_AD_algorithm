#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "general_modules/spline.h"

namespace AD_algorithm {
namespace general {

/**
 * @brief Bezier曲线工具（默认五次）
 */
template <int N_DEG = 5>
class BezierUtils {
public:
    typedef Eigen::Matrix<double, N_DEG + 1, 1> VecBasis;

    /**
     * @brief 计算Bezier基函数（非缩放）
     */
    static VecBasis GetBezierBasis(int derivative_degree, double t) {
        t = std::max(std::min(1.0, t), 0.0);
        VecBasis basis;
        switch (N_DEG) {
            case 5:
                if (derivative_degree == 0) {
                    basis << -std::pow(t - 1, 5),
                              5 * t * std::pow(t - 1, 4),
                              -10 * std::pow(t, 2) * std::pow(t - 1, 3),
                              10 * std::pow(t, 3) * std::pow(t - 1, 2),
                              -5 * std::pow(t, 4) * (t - 1),
                              std::pow(t, 5);
                } else if (derivative_degree == 1) {
                    basis << -5 * std::pow(t - 1, 4),
                              20 * t * std::pow(t - 1, 3) + 5 * std::pow(t - 1, 4),
                              -20 * t * std::pow(t - 1, 3) - 30 * std::pow(t, 2) * std::pow(t - 1, 2),
                              10 * std::pow(t, 3) * (2 * t - 2) + 30 * std::pow(t, 2) * std::pow(t - 1, 2),
                              -20 * std::pow(t, 3) * (t - 1) - 5 * std::pow(t, 4),
                              5 * std::pow(t, 4);
                } else if (derivative_degree == 2) {
                    basis << -20 * std::pow(t - 1, 3),
                              60 * t * std::pow(t - 1, 2) + 40 * std::pow(t - 1, 3),
                              -120 * t * std::pow(t - 1, 2) - 20 * std::pow(t - 1, 3) -
                                  30 * t * t * (2 * t - 2),
                              60 * t * std::pow(t - 1, 2) + 60 * t * t * (2 * t - 2) +
                                  20 * t * t * t,
                              -60 * t * t * (t - 1) - 40 * t * t * t,
                              20 * t * t * t;
                } else if (derivative_degree == 3) {
                    basis << -60 * std::pow(t - 1, 2),
                              60 * t * (2 * t - 2) + 180 * std::pow(t - 1, 2),
                              -180 * t * (2 * t - 2) - 180 * std::pow(t - 1, 2) - 60 * t * t,
                              180 * t * (2 * t - 2) + 60 * std::pow(t - 1, 2) + 180 * t * t,
                              -120 * t * (t - 1) - 180 * t * t,
                              60 * t * t;
                } else {
                    assert(false);
                }
                break;
            default:
                assert(false);
                break;
        }
        return basis;
    }
};

/**
 * @brief Bezier样条（默认五次），控制点矩阵按行存储
 */
template <int N_DEG = 5, int N_DIM = 2>
class BezierSpline {
public:
    typedef Eigen::Matrix<double, N_DIM, 1> VecND;
    typedef Eigen::Matrix<double, N_DEG + 1, N_DIM> CtrlMat;

    BezierSpline() = default;

    /**
     * @brief 设置参数区间
     */
    void set_vec_domain(const std::vector<double>& vec_domain) {
        assert(vec_domain.size() > 1);
        vec_domain_ = vec_domain;
        ctrl_pts_.resize(vec_domain.size() - 1);
        for (auto& m : ctrl_pts_) {
            m.setZero();
        }
    }

    void set_coeff(int segment_idx, int ctrl_pt_index, const VecND& coeff) {
        assert(segment_idx >= 0 && segment_idx < num_segments());
        assert(ctrl_pt_index >= 0 && ctrl_pt_index <= N_DEG);
        ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff.transpose();
    }

    void set_ctrl_pts(const std::vector<CtrlMat>& pts) { ctrl_pts_ = pts; }

    int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

    std::vector<double> vec_domain() const { return vec_domain_; }

    std::vector<CtrlMat> ctrl_pts() const { return ctrl_pts_; }

    double begin() const {
        if (vec_domain_.empty()) return 0.0;
        return vec_domain_.front();
    }

    double end() const {
        if (vec_domain_.empty()) return 0.0;
        return vec_domain_.back();
    }

    /**
     * @brief 样条评估（超出域时返回非法）
     */
    SplineErrorType evaluate(const double s, const int d, VecND* ret) const {
        if (vec_domain_.size() < 2) return SplineErrorType::kIllegalInput;
        if (!ret) return SplineErrorType::kIllegalInput;

        auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
        int idx = std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0),
                           static_cast<int>(vec_domain_.size()) - 2);
        if (s < vec_domain_.front()) {
            return SplineErrorType::kIllegalInput;
        }

        double h = s - vec_domain_[idx];
        double duration = vec_domain_[idx + 1] - vec_domain_[idx];
        if (duration <= 0.0) return SplineErrorType::kIllegalInput;

        double normalized_s = h / duration;
        auto basis = BezierUtils<N_DEG>::GetBezierBasis(d, normalized_s);
        VecND result = (std::pow(duration, 1 - d) * basis.transpose() * ctrl_pts_[idx]).transpose();
        *ret = result;
        return SplineErrorType::kSuccess;
    }

private:
    std::vector<CtrlMat> ctrl_pts_;
    std::vector<double> vec_domain_;
};

}  // namespace general
}  // namespace AD_algorithm
