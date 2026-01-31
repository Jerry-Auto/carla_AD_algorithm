#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace AD_algorithm {
namespace general {

class PolynomialCurve {
public:
    PolynomialCurve();
    
    /**
     * @brief 拟合五次多项式曲线
     * @param x1 起点x坐标
     * @param y1 起点y坐标
     * @param dy1 起点一阶导数
     * @param ddy1 起点二阶导数
     * @param x2 终点x坐标
     * @param y2 终点y坐标
     * @param dy2 终点一阶导数
     * @param ddy2 终点二阶导数
     * @return 是否拟合成功
     */
    bool curve_fitting(double x1, double y1, double dy1, double ddy1,
                      double x2, double y2, double dy2, double ddy2);

    /**
     * @brief 拟合四次多项式（用于巡航，不指定终点位置）
     * @param x1 起点x
     * @param y1 起点y
     * @param dy1 起点一阶导
     * @param ddy1 起点二阶导
     * @param x2 终点x
     * @param dy2 终点一阶导
     * @param ddy2 终点二阶导
     * @return 是否拟合成功
     */
    bool curve_fitting(double x1, double y1, double dy1, double ddy1,
                      double x2, double dy2, double ddy2);
    
    /**
     * @brief 计算多项式在指定x处的值或导数值
     * @param x 自变量x
     * @param derivative_order 导数阶数 (0:原函数, 1:一阶导, 2:二阶导, 3:三阶导)
     * @return 计算结果
     */
    double value_evaluation(double x, int derivative_order = 0) const;
    
    const Eigen::VectorXd& getCoefficients() const;
    void setCoefficients(const Eigen::VectorXd& coeffs);

    int getOrder() const;

private:
    bool try_fit_quintic_analytic(double x1, double y1, double dy1, double ddy1,
                                 double x2, double y2, double dy2, double ddy2);
    bool try_fit_degree_constrained(int degree, double x1, double y1, double dy1, double ddy1,
                                    double x2, double y2, double dy2, double ddy2);

    int order_;
    double x_offset_; // 坐标平移量，用于提高数值稳定性
    Eigen::VectorXd coefficients_;  // 多项式系数，从低次到高次排列
};

/**
 * @brief 高维多项式曲线类，模板化以支持任意维度
 * 系数存储顺序：从低次到高次
 */
template <int N_DEG, int N_DIM>
class PolynomialCurveND {
public:
    typedef Eigen::Matrix<double, N_DEG + 1, N_DIM> CoeffMatrix;
    typedef Eigen::Matrix<double, N_DIM, 1> VecND;

    PolynomialCurveND() { set_zero(); }
    PolynomialCurveND(const CoeffMatrix& coeff) : coeff_(coeff) {}

    /**
     * @brief 设置系数
     */
    void set_coeff(const CoeffMatrix& coeff) { coeff_ = coeff; }

    /**
     * @brief 获取系数
     */
    const CoeffMatrix& coeff() const { return coeff_; }

    /**
     * @brief 设置为零
     */
    void set_zero() { coeff_.setZero(); }

    /**
     * @brief 评估多项式及其导数
     * @param s 参数值
     * @param d 导数阶数
     * @param vec 输出向量
     */
    inline void evaluate(const double& s, int d, VecND* vec) const {
        vec->setZero();
        for (int dim = 0; dim < N_DIM; ++dim) {
            // 使用Horner's rule计算导数
            double p = coeff_(0, dim) / factorial(N_DEG - d);
            for (int i = 1; i <= N_DEG - d; ++i) {
                p = p * s + coeff_(i, dim) / factorial(N_DEG - i - d);
            }
            (*vec)(dim) = p;
        }
    }

    /**
     * @brief 评估多项式（0阶导数）
     * @param s 参数值
     * @param vec 输出向量
     */
    inline void evaluate(const double& s, VecND* vec) const {
        vec->setZero();
        for (int dim = 0; dim < N_DIM; ++dim) {
            double p = coeff_(N_DEG, dim);
            for (int i = 1; i <= N_DEG; ++i) {
                p = p * s + coeff_(N_DEG - i, dim);
            }
            (*vec)(dim) = p;
        }
    }

private:
    static double factorial(int n) {
        static std::vector<double> fac = {1.0};
        while (fac.size() <= n) {
            fac.push_back(fac.back() * fac.size());
        }
        return fac[n];
    }

    CoeffMatrix coeff_;
};

} // namespace general
} // namespace AD_algorithm
