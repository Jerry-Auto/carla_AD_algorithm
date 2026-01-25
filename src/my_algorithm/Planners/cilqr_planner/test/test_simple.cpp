#include <iostream>
#include "cilqr_planner/cost_define.h"
#include "cilqr_planner/planner_weight.h"
#include "general_modules/FrenetFrame.h"
#include "general_modules/common_types.h"
#include "general_modules/matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace AD_algorithm;
using namespace AD_algorithm::planner;
using namespace AD_algorithm::general;

void visualize(
    const std::vector<general::PathPoint>& reference_path,
    const std::vector<general::Obstacle>& obstacles,
    const std::vector<general::TrajectoryPoint>& trajectory,
    const general::VehicleState& ego)
{
    plt::backend("Qt5Agg");
    plt::figure(1);
    plt::clf();

    // 1. 绘制参考线
    std::vector<double> rx, ry;
    for (const auto& p : reference_path) {
        rx.push_back(p.x);
        ry.push_back(p.y);
    }
    plt::plot(rx, ry, {{"color", "blue"}, {"linewidth", "2"}});
    plt::text(rx.front(), ry.front(), "Reference Start");

    // 2. 绘制障碍物
    for (const auto& obs : obstacles) {
        double half_l = obs.length / 2.0;
        double half_w = obs.width / 2.0;

        std::vector<double> ox = {
            obs.x - half_l, obs.x + half_l,
            obs.x + half_l, obs.x - half_l,
            obs.x - half_l
        };
        std::vector<double> oy = {
            obs.y - half_w, obs.y - half_w,
            obs.y + half_w, obs.y + half_w,
            obs.y - half_w
        };

        plt::plot(ox, oy, "r");
        plt::text(obs.x, obs.y, "Obs " + std::to_string(obs.id));
    }

    // 3. 绘制规划轨迹
    std::vector<double> tx, ty;
    for (const auto& p : trajectory) {
        tx.push_back(p.x);
        ty.push_back(p.y);
    }
    plt::plot(tx, ty, {{"color", "green"}, {"linewidth", "2"}});
    plt::text(tx.front(), ty.front(), "Trajectory Start");

    // 4. 绘制自车位置
    std::vector<double> ex = {ego.x};
    std::vector<double> ey = {ego.y};

    plt::scatter(ex, ey, 80.0, {{"c", "yellow"}, {"edgecolors", "black"}});

    plt::text(ego.x, ego.y, "Ego");

    // 图形设置
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::title("CILQR Planner Visualization");
    plt::axis("equal");
    plt::grid(true);
    plt::xlim(ego.x - 30.0, ego.x + 30.0);
    plt::ylim(ego.y - 20.0, ego.y + 30.0);

    plt::pause(0.001);
}

void test_vehicle_model(){
    // 测试并可视化车辆状态转移是否正确
    auto param_=std::make_shared<AD_algorithm::planner::CILQRPlannerparams>();
    param_->ilqr_params->N=50;
    param_->ilqr_params->dt=0.1;
    std::shared_ptr<cilqr_problem> problem=std::make_shared<cilqr_problem>(param_);
    auto ilqr_solver_=std::make_shared<cilqr_problem>(param_);
    std::vector<AD_algorithm::general::PathPoint> path_points;
    for(int i=0;i<100;i++){
        PathPoint pt;
        pt.x= i*0.5;
        pt.y= 0.0;
        path_points.push_back(pt);
    }
    AD_algorithm::general::ReferenceLine frenet_line(path_points);
    auto frenet_frame_=std::make_shared<AD_algorithm::general::FrenetFrame>(frenet_line);
    problem->set_frenet_frame(frenet_frame_);
    general::VehicleState init_state;
    init_state.x=0.0;
    init_state.y=0.0;
    init_state.heading=0.0;
    init_state.v=10.0;
    problem->set_initial_state(init_state);
    std::vector<general::Obstacle> obstacles;
    general::VehicleState ego_state=init_state;
    for(int i=0;i<100;i++){
        // 每次推进0.1秒
        Eigen::VectorXd state_vec(4);
        state_vec<<ego_state.x,ego_state.y,ego_state.heading,ego_state.v;
        Eigen::VectorXd control_vec(2);
        control_vec<<0.5,0.5; 
        Eigen::VectorXd su=Eigen::VectorXd(state_vec.size()+control_vec.size());
        su<<state_vec,control_vec;
        Eigen::VectorXd next_state_vec=problem->state_transition(su);
        ego_state.x=next_state_vec(0);
        ego_state.y=next_state_vec(1);
        ego_state.heading=next_state_vec(2);
        ego_state.v=next_state_vec(3);

        std::vector<general::TrajectoryPoint> traj_points;
        general::TrajectoryPoint tp;
        tp.x=ego_state.x;
        tp.y=ego_state.y;
        traj_points.push_back(tp);
        visualize(path_points,obstacles,traj_points,ego_state);
    }
}

int main() {
    // test_safe_conversion();
    test_vehicle_model();
    return 0;
}
