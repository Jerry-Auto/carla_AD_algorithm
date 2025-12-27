#include "general_modules/ST_graph.h"
#include <cmath>

namespace AD_algorithm {
namespace general {

bool STCoordinate::cartesianToST(const std::vector<general::TrajectoryPoint>& cartesianTraj, 
                                 std::vector<general::STPoint>& stTraj) {
    if (cartesianTraj.empty()) return false;

    stTraj.clear();
    std::vector<double> index2s = calculatePathIndex2s(cartesianTraj);

    for (size_t i = 0; i < cartesianTraj.size(); ++i) {
        general::STPoint st;
        st.t = cartesianTraj[i].time_stamped;
        st.s = index2s[i];
        st.s_dot = cartesianTraj[i].v;
        st.s_dot_dot = cartesianTraj[i].a_tau; // 复用通用类的切向加速度

        stTraj.push_back(st);
    }
    return true;
}

bool STCoordinate::dynamicObsToST(const std::vector<general::FrenetPoint>& obsFrenetSet, 
                                  double deltaL, 
                                  std::vector<std::unordered_map<std::string, double>>& stGraph) {
    stGraph.clear();
    for (const auto& obs : obsFrenetSet) {
        if (std::abs(obs.l_dot) <= 0.2) continue;

        double tIn = 0.0, tOut = 0.0;
        if (std::abs(obs.l) > deltaL) {
            if (obs.l * obs.l_dot > 0) continue;
            tIn = std::abs(obs.l/obs.l_dot) - std::abs(deltaL/obs.l_dot);
            tOut = std::abs(obs.l/obs.l_dot) + std::abs(deltaL/obs.l_dot);
        } else {
            tIn = 0.0;
            tOut = obs.l_dot > 0 ? (deltaL - obs.l)/obs.l_dot : (-deltaL - obs.l)/obs.l_dot;
        }

        if (tOut >= 8 || tOut <= 1) continue;

        stGraph.push_back({{"t_in", tIn}, {"t_out", tOut},
                           {"s_in", obs.s + obs.s_dot * tIn}, {"s_out", obs.s + obs.s_dot * tOut}});
    }
    return true;
}

bool STCoordinate::stToCartesian(const std::vector<general::STPoint>& stTraj,
                                 const std::vector<general::TrajectoryPoint>& pathTraj,
                                 const std::vector<double>& pathIndex2s,
                                 std::vector<general::TrajectoryPoint>& cartesianTraj) {
    cartesianTraj.clear();
    if (stTraj.empty() || pathTraj.empty() || pathIndex2s.empty()) return false;

    for (const auto& st : stTraj) {
        general::TrajectoryPoint tp;
        tp.v = st.s_dot;
        tp.a_tau = st.s_dot_dot;
        tp.time_stamped = st.t;

        // 插值找对应路径点（复用pathIndex2s）
        double curS = st.s;
        int idx = -1;
        for (size_t i = 0; i < pathIndex2s.size()-1; ++i) {
            if (curS >= pathIndex2s[i] && curS < pathIndex2s[i+1]) {
                idx = static_cast<int>(i);
                break;
            }
        }
        if (idx == -1) idx = static_cast<int>(pathIndex2s.size()-1);

        // 线性插值（复用通用类的TrajectoryPoint字段）
        double ratio = (curS - pathIndex2s[idx]) / (pathIndex2s[idx+1] - pathIndex2s[idx]);
        tp.x = pathTraj[idx].x + ratio * (pathTraj[idx+1].x - pathTraj[idx].x);
        tp.y = pathTraj[idx].y + ratio * (pathTraj[idx+1].y - pathTraj[idx].y);
        tp.heading = pathTraj[idx].heading + ratio * (pathTraj[idx+1].heading - pathTraj[idx].heading);
        tp.kappa = pathTraj[idx].kappa + ratio * (pathTraj[idx+1].kappa - pathTraj[idx].kappa);

        cartesianTraj.push_back(tp);
    }
    return true;
}

int STCoordinate::findTIndex(const std::vector<general::STPoint>& stTraj, double t) {
    if (t < stTraj.front().t || t >= stTraj.back().t) return -1;

    for (size_t i = 0; i < stTraj.size()-1; ++i) {
        if (t >= stTraj[i].t && t < stTraj[i+1].t) {
            return (std::abs(t - stTraj[i].t) <= std::abs(t - stTraj[i+1].t)) ? i : i+1;
        }
    }
    return -1;
}

std::vector<double> STCoordinate::calculatePathIndex2s(const std::vector<general::TrajectoryPoint>& pathTraj) {
    std::vector<double> index2s;
    index2s.push_back(0.0);
    for (size_t i = 1; i < pathTraj.size(); ++i) {
        double ds = std::hypot(pathTraj[i].x - pathTraj[i-1].x, pathTraj[i].y - pathTraj[i-1].y);
        index2s.push_back(index2s.back() + ds);
    }
    return index2s;
}

} // namespace planning
} // namespace AD_algorithm
