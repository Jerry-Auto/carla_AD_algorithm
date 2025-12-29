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
    Eigen::VectorXd coefficients_;
};

} // namespace general
} // namespace AD_algorithm
