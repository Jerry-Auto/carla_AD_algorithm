/* Copyright 2025 <Your Name> */

#include "general_modules/polynomial_curve.h"
#include <iostream>
#include <Eigen/QR>
#include <limits> 

namespace AD_algorithm {
namespace general {

PolynomialCurve::PolynomialCurve() : order_(5), x_offset_(0.0) {
    coefficients_ = Eigen::VectorXd::Zero(order_ + 1);
}
//五次多项式拟合
bool PolynomialCurve::curve_fitting(double x1, double y1, double dy1, double ddy1,
                  double x2, double y2, double dy2, double ddy2) {

    const double dx = x2 - x1;
    // 极小区间：直接退化为常值/线性，保证成功
    if (std::abs(dx) < 1e-8) {
        x_offset_ = x1;
        order_ = 1;
        coefficients_ = Eigen::VectorXd::Zero(order_ + 1);
        coefficients_[0] = y1;
        coefficients_[1] = 0.0;
        return true;
    }

    // 先尝试五次解析解（最快且满足全部边界导数约束）
    if (try_fit_quintic_analytic(x1, y1, dy1, ddy1, x2, y2, dy2, ddy2)) {
        return true;
    }

    // 解析解失败则降阶：4 -> 3 -> 2 -> 1
    for (int degree = 4; degree >= 1; --degree) {
        if (try_fit_degree_constrained(degree, x1, y1, dy1, ddy1, x2, y2, dy2, ddy2)) {
            return true;
        }
    }

    // 最终兜底：线性插值，保证端点通过
    x_offset_ = x1;
    order_ = 1;
    coefficients_ = Eigen::VectorXd::Zero(order_ + 1);
    coefficients_[0] = y1;
    coefficients_[1] = (y2 - y1) / dx;
    return true;
}
// 四次多项式拟合
bool PolynomialCurve::curve_fitting(double x1, double y1, double dy1, double ddy1,
                  double x2, double dy2, double ddy2) {
    const double dx = x2 - x1;
    if (std::abs(dx) < 1e-8) {
        return false;
    }

    x_offset_ = x1;
    order_ = 4;
    coefficients_ = Eigen::VectorXd::Zero(order_ + 1);

    // p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
    // t = x - x1
    // p(0) = y1 => a0 = y1
    // p'(0) = dy1 => a1 = dy1
    // p''(0) = ddy1 => 2*a2 = ddy1 => a2 = ddy1 / 2
    
    double a0 = y1;
    double a1 = dy1;
    double a2 = ddy1 / 2.0;

    coefficients_[0] = a0;
    coefficients_[1] = a1;
    coefficients_[2] = a2;

    // At t = dx:
    // p'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 = dy2
    // p''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 = ddy2
    
    // 3*t^2*a3 + 4*t^3*a4 = dy2 - a1 - 2*a2*t
    // 6*t*a3 + 12*t^2*a4 = ddy2 - 2*a2

    double t = dx;
    double t2 = t * t;
    double t3 = t2 * t;

    Eigen::Matrix2d A;
    A << 3 * t2, 4 * t3,
         6 * t, 12 * t2;
    
    Eigen::Vector2d b;
    b << dy2 - a1 - 2 * a2 * t,
         ddy2 - 2 * a2;
    
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    coefficients_[3] = x(0);
    coefficients_[4] = x(1);

    return true;
}

bool PolynomialCurve::try_fit_quintic_analytic(double x1, double y1, double dy1, double ddy1,
                                               double x2, double y2, double dy2, double ddy2) {
    const double dx = x2 - x1;
    if (std::abs(dx) < 1e-8) {
        return false;
    }

    x_offset_ = x1;
    order_ = 5;
    coefficients_ = Eigen::VectorXd::Zero(order_ + 1);

    const double u2 = dx;
    coefficients_[0] = y1;
    coefficients_[1] = dy1;
    coefficients_[2] = 0.5 * ddy1;

    const double u2_2 = u2 * u2;
    const double u2_3 = u2_2 * u2;
    const double u2_4 = u2_3 * u2;
    const double u2_5 = u2_4 * u2;

    const double h = y2 - (y1 + dy1 * u2 + 0.5 * ddy1 * u2_2);
    const double dh = dy2 - (dy1 + ddy1 * u2);
    const double ddh = ddy2 - ddy1;

    Eigen::Matrix3d A_sub;
    A_sub << u2_3, u2_4, u2_5,
             3 * u2_2, 4 * u2_3, 5 * u2_4,
             6 * u2, 12 * u2_2, 20 * u2_3;

    Eigen::Vector3d b_sub;
    b_sub << h, dh, ddh;

    Eigen::Vector3d x_sub = A_sub.colPivHouseholderQr().solve(b_sub);
    coefficients_[3] = x_sub(0);
    coefficients_[4] = x_sub(1);
    coefficients_[5] = x_sub(2);

    const double sum = coefficients_.sum();
    if (!std::isfinite(sum)) {
        return false;
    }

    // 验证端点（避免数值爆炸）
    const double y1_eval = value_evaluation(x1, 0);
    const double y2_eval = value_evaluation(x2, 0);
    if (!std::isfinite(y1_eval) || !std::isfinite(y2_eval)) {
        return false;
    }
    return true;
}

static Eigen::RowVectorXd poly_basis(double u, int degree) {
    Eigen::RowVectorXd row(degree + 1);
    row.setZero();
    double p = 1.0;
    for (int i = 0; i <= degree; ++i) {
        row(i) = p;
        p *= u;
    }
    return row;
}

static Eigen::RowVectorXd poly_basis_derivative(double u, int degree, int deriv_order) {
    Eigen::RowVectorXd row(degree + 1);
    row.setZero();
    if (deriv_order <= 0) {
        return poly_basis(u, degree);
    }

    for (int i = deriv_order; i <= degree; ++i) {
        // i*(i-1)*...*(i-deriv_order+1) * u^(i-deriv_order)
        double coeff = 1.0;
        for (int k = 0; k < deriv_order; ++k) {
            coeff *= static_cast<double>(i - k);
        }
        row(i) = coeff * std::pow(u, i - deriv_order);
    }
    return row;
}

bool PolynomialCurve::try_fit_degree_constrained(int degree, double x1, double y1, double dy1, double ddy1,
                                                 double x2, double y2, double dy2, double ddy2) {
    const double dx = x2 - x1;
    if (degree < 1) {
        return false;
    }
    if (std::abs(dx) < 1e-8) {
        return false;
    }

    x_offset_ = x1;
    order_ = degree;
    coefficients_ = Eigen::VectorXd::Zero(order_ + 1);

    const double u1 = 0.0;
    const double u2 = dx;

    // 等式约束：严格通过端点
    Eigen::MatrixXd E(2, degree + 1);
    E.row(0) = poly_basis(u1, degree);
    E.row(1) = poly_basis(u2, degree);
    Eigen::VectorXd f(2);
    f << y1, y2;

    // 线性（degree==1）时端点约束即可确定唯一解
    if (degree == 1) {
        Eigen::Vector2d c = E.fullPivLu().solve(f);
        if (!std::isfinite(c.sum())) {
            return false;
        }
        coefficients_ = c;
        return true;
    }

    // 软约束：尽量满足端点一阶/二阶导（能表示的情况下）
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> rhs;
    if (degree >= 1) {
        rows.push_back(poly_basis_derivative(u1, degree, 1));
        rhs.push_back(dy1);
        rows.push_back(poly_basis_derivative(u2, degree, 1));
        rhs.push_back(dy2);
    }
    if (degree >= 2) {
        rows.push_back(poly_basis_derivative(u1, degree, 2));
        rhs.push_back(ddy1);
        rows.push_back(poly_basis_derivative(u2, degree, 2));
        rhs.push_back(ddy2);
    }

    const int m = static_cast<int>(rows.size());
    Eigen::MatrixXd A(m, degree + 1);
    Eigen::VectorXd b(m);
    for (int i = 0; i < m; ++i) {
        A.row(i) = rows[i];
        b(i) = rhs[i];
    }

    // 无软约束时（理论上不会发生），直接解端点约束
    if (m == 0) {
        Eigen::VectorXd c = E.fullPivLu().solve(f);
        if (!std::isfinite(c.sum())) {
            return false;
        }
        coefficients_ = c;
        return true;
    }

    // 约束最小二乘：min ||A c - b||^2 s.t. E c = f
    // KKT 系统：
    // [A^T A  E^T] [c]   [A^T b]
    // [ E     0 ] [λ] = [  f  ]
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::VectorXd Atb = A.transpose() * b;

    Eigen::MatrixXd KKT(degree + 1 + 2, degree + 1 + 2);
    KKT.setZero();
    KKT.topLeftCorner(degree + 1, degree + 1) = AtA;
    KKT.topRightCorner(degree + 1, 2) = E.transpose();
    KKT.bottomLeftCorner(2, degree + 1) = E;

    Eigen::VectorXd rhs_kkt(degree + 1 + 2);
    rhs_kkt.head(degree + 1) = Atb;
    rhs_kkt.tail(2) = f;

    Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs_kkt);
    Eigen::VectorXd c = sol.head(degree + 1);

    if (!std::isfinite(c.sum())) {
        return false;
    }

    // 验证端点是否满足（数值容差）
    const double y1_eval = (E.row(0) * c)(0);
    const double y2_eval = (E.row(1) * c)(0);
    if (!std::isfinite(y1_eval) || !std::isfinite(y2_eval)) {
        return false;
    }
    if (std::abs(y1_eval - y1) > 1e-6 || std::abs(y2_eval - y2) > 1e-6) {
        return false;
    }

    coefficients_ = c;
    return true;
}

double PolynomialCurve::value_evaluation(double x, int derivative_order) const {
    if (derivative_order < 0 || derivative_order > 3 || order_ < derivative_order) {
        return 0.0;
    }
    
    // 使用平移后的
    double u = x - x_offset_;
    
    // 使用Horner's rule for efficiency
    if (derivative_order == 0) {
        double p = coefficients_(order_);
        for (int i = order_ - 1; i >= 0; --i) {
            p = p * u + coefficients_(i);
        }
        return p;
    } else {
        // For derivatives, compute the derivative polynomial coefficients on the fly
        // But for efficiency, use Horner's rule adapted for derivatives
        double p = 0.0;
        for (int i = derivative_order; i <= order_; ++i) {
            double coeff_deriv = coefficients_(i);
            for (int k = 0; k < derivative_order; ++k) {
                coeff_deriv *= (i - k);
            }
            if (i - derivative_order >= 0) {
                double term = coeff_deriv * std::pow(u, i - derivative_order);
                p += term;
            }
        }
        return p;
    }
}

const Eigen::VectorXd& PolynomialCurve::getCoefficients() const { return coefficients_; }

void PolynomialCurve::setCoefficients(const Eigen::VectorXd& coeffs) {
    if (coeffs.size() >= 2) {
        order_ = static_cast<int>(coeffs.size()) - 1;
        coefficients_ = coeffs;
    }
}

int PolynomialCurve::getOrder() const { return order_; }

} // namespace general
} // namespace AD_algorithm
