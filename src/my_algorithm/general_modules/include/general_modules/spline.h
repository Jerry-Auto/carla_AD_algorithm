#pragma once

#include <algorithm>
#include <cassert>
#include <vector>
#include <Eigen/Dense>

namespace AD_algorithm {
namespace general {

enum class SplineErrorType {
    kSuccess = 0,
    kIllegalInput = 1
};

/**
 * @brief 多项式样条（默认五次），系数按低次到高次存储
 * 模板参数顺序与EPSILON保持一致：<N_DEG, N_DIM>
 */
template <int N_DEG = 5, int N_DIM = 2>
class Spline {
public:
    typedef Eigen::Matrix<double, N_DIM, 1> VecND;
    typedef Eigen::Matrix<double, N_DEG + 1, N_DIM> CoeffMatrix;

    Spline() = default;

    /**
     * @brief 设置样条参数区间
     */
    void set_vec_domain(const std::vector<double>& vec_domain) {
        assert(vec_domain.size() > 1);
        vec_domain_ = vec_domain;
        coeffs_.resize(vec_domain_.size() - 1);
        for (auto& c : coeffs_) {
            c.setZero();
        }
    }

    /**
     * @brief 获取分段数量
     */
    int num_segments() const { return static_cast<int>(coeffs_.size()); }

    /**
     * @brief 获取参数区间
     */
    std::vector<double> vec_domain() const { return vec_domain_; }

    /**
     * @brief 参数起点
     */
    double begin() const {
        if (vec_domain_.empty()) return 0.0;
        return vec_domain_.front();
    }

    /**
     * @brief 参数终点
     */
    double end() const {
        if (vec_domain_.empty()) return 0.0;
        return vec_domain_.back();
    }

    /**
     * @brief 设置某段的系数矩阵（低次->高次）
     */
    void set_coeff_segment(int segment_idx, const CoeffMatrix& coeff) {
        assert(segment_idx >= 0 && segment_idx < num_segments());
        coeffs_[segment_idx] = coeff;
    }

    /**
     * @brief 设置某段某阶系数（向量）
     */
    void set_coeff(int segment_idx, int coeff_index, const VecND& coeff) {
        assert(segment_idx >= 0 && segment_idx < num_segments());
        assert(coeff_index >= 0 && coeff_index <= N_DEG);
        coeffs_[segment_idx].row(coeff_index) = coeff.transpose();
    }

    /**
     * @brief 获取某段系数矩阵
     */
    const CoeffMatrix& coeff_segment(int segment_idx) const {
        assert(segment_idx >= 0 && segment_idx < num_segments());
        return coeffs_[segment_idx];
    }

    /**
     * @brief 样条评估（自动外推到两端段）
     */
    SplineErrorType evaluate(const double s, int d, VecND* ret) const {
        if (vec_domain_.size() < 2) return SplineErrorType::kIllegalInput;
        if (!ret) return SplineErrorType::kIllegalInput;

        auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
        int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
        int last_seg = static_cast<int>(vec_domain_.size()) - 2;
        if (idx > last_seg) idx = last_seg;

        double h = s - vec_domain_[idx];
        if (s < vec_domain_.front()) {
            idx = 0;
            h = s - vec_domain_[0];
        } else if (s > vec_domain_.back()) {
            idx = last_seg;
            h = s - vec_domain_[last_seg];
        }

        evaluate_segment(coeffs_[idx], h, d, ret);
        return SplineErrorType::kSuccess;
    }

    /**
     * @brief 0阶评估
     */
    SplineErrorType evaluate(const double s, VecND* ret) const {
        return evaluate(s, 0, ret);
    }

private:
    static inline double factorial_coeff(int i, int d) {
        double c = 1.0;
        for (int k = 0; k < d; ++k) {
            c *= static_cast<double>(i - k);
        }
        return c;
    }

    static inline double evaluate_poly_dim(const CoeffMatrix& coeff, int dim, double s, int d) {
        if (d < 0 || d > N_DEG) return 0.0;
        double p = 0.0;
        for (int i = N_DEG; i >= d; --i) {
            double c = coeff(i, dim);
            if (d > 0) {
                c *= factorial_coeff(i, d);
            }
            p = p * s + c;
        }
        return p;
    }

    static inline void evaluate_segment(const CoeffMatrix& coeff, double s, int d, VecND* ret) {
        ret->setZero();
        for (int dim = 0; dim < N_DIM; ++dim) {
            (*ret)(dim) = evaluate_poly_dim(coeff, dim, s, d);
        }
    }

    std::vector<CoeffMatrix> coeffs_;
    std::vector<double> vec_domain_;
};

}  // namespace general
}  // namespace AD_algorithm
