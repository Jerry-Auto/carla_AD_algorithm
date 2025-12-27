#include "general_modules/math_tool.h"

namespace AD_algorithm {
namespace general {

// 角度标准化
double normalize_angle(const double angle)
{
    double a = std::fmod(angle, 2 * M_PI);
    if (a > M_PI)
    {
        return a - 2 * M_PI;
    }
    else if(a < -M_PI)
    {
        return a + 2 * M_PI;
    }
    else
    {
        return a;
    }
}

}}