#include "general_modules/Trajectory.h"  // 包含 PathPoint 定义
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <random>
#include <cstdlib>
#if __has_include(<Python.h>)
#include "general_modules/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#define HAVE_MATPLOTLIBCPP 1
#else
#define HAVE_MATPLOTLIBCPP 0
#endif

#include "lattice/lattice_planner.h"
#include "general_modules/FrenetFrame.h"

using namespace AD_algorithm;
using namespace AD_algorithm::planner;
using namespace AD_algorithm::general;


static std::vector<PathPoint> generate_path(){
    std::vector<PathPoint> reference_path;
    const int total_length = 700; // 总长700米
    const double ds = 0.3;        // 间隔
    const int num_points = total_length/ds + 1; // s = 0 到 700

    // 转弯参数
    const double R_left = 100.0;   // 左转半径（正）
    const double R_right = 100.0;  // 右转半径（正）
    const double arc_length = 100.0;

    // 各段边界（按s）
    const double s1_end = 150.0;           // 直线1结束
    const double s2_end = s1_end + arc_length;   // 左转结束
    const double s3_end = s2_end + 200.0;        // 直线2结束
    const double s4_end = s3_end + arc_length;   // 右转结束


    for (int i = 0; i <= num_points; ++i) {
        double s = i * ds;
        PathPoint point;
        point.x = 0.0;
        point.y = 0.0;
        point.heading = 0.0;
        point.kappa = 0.0;
        point.kappa_rate = 0.0;
        point.accumulated_s = s;

        if (s <= s1_end) {
            // 第一段：直线
            point.x = s;
            point.y = 0.0;
            point.heading = 0.0;
            point.kappa = 0.0;
        }
        else if (s <= s2_end) {
            // 第二段：左转圆弧（逆时针）
            double delta_s = s - s1_end; // 从左转起点开始的弧长
            double theta = delta_s / R_left; // 转过的角度（弧度）
            // 圆心在 (s1_end, R_left)
            point.x = s1_end + R_left * std::sin(theta);
            point.y = R_left * (1.0 - std::cos(theta));
            point.heading = theta; // 切线方向
            point.kappa = 1.0 / R_left;
        }
        else if (s <= s3_end) {
            // 第三段：直线（沿左转结束方向）
            double delta_s = s - s2_end;
            // 左转结束时的状态
            double theta_total = arc_length / R_left; // 总偏航角
            double x_start = s1_end + R_left * std::sin(theta_total);
            double y_start = R_left * (1.0 - std::cos(theta_total));
            point.x = x_start + delta_s * std::cos(theta_total);
            point.y = y_start + delta_s * std::sin(theta_total);
            point.heading = theta_total;
            point.kappa = 0.0;
        }
        else if (s <= s4_end) {
            // 第四段：右转圆弧（顺时针）
            double delta_s = s - s3_end;
            double theta = delta_s / R_right; // 转过的角度（正值，但向右）
            double theta_start = arc_length / R_left;
            double x_start = s1_end + R_left * std::sin(theta_start) + 200.0 * std::cos(theta_start);
            double y_start = R_left * (1.0 - std::cos(theta_start)) + 200.0 * std::sin(theta_start);
            point.x = x_start + R_right * (std::sin(theta_start) - std::sin(theta_start - theta));
            point.y = y_start + R_right * (std::cos(theta_start - theta) - std::cos(theta_start));
            point.heading = theta_start - theta; // heading 减小
            point.kappa = -1.0 / R_right; // 右转曲率为负
        }
        else {
            // 第五段：最后直线
            double delta_s = s - s4_end;
            double theta_after_right = arc_length / R_left - arc_length / R_right; // 最终 heading
            double theta_start = arc_length / R_left;
            double x_after_straight = s1_end + R_left * std::sin(theta_start) + 200.0 * std::cos(theta_start);
            double y_after_straight = R_left * (1.0 - std::cos(theta_start)) + 200.0 * std::sin(theta_start);
            double x_turn_end = x_after_straight + R_right * (std::sin(theta_start) - std::sin(theta_start - arc_length / R_right));
            double y_turn_end = y_after_straight + R_right * (std::cos(theta_start - arc_length / R_right) - std::cos(theta_start));
            point.x = x_turn_end + delta_s * std::cos(theta_after_right);
            point.y = y_turn_end + delta_s * std::sin(theta_after_right);
            point.heading = theta_after_right;
            point.kappa = 0.0;
        }

        reference_path.push_back(point);
    }
    return reference_path;
}

// 可视化：显示参考线、障碍物、自车和规划轨迹
static void visualize(
    const std::vector<PathPoint>& reference_path,
    const std::vector<Obstacle>& obstacles,
    const std::vector<TrajectoryPoint>& trajectory,
    const VehicleState& ego)
{
#if !HAVE_MATPLOTLIBCPP
    // matplotlibcpp / Python 开发头不可用时为 no-op
    (void)reference_path; (void)obstacles; (void)trajectory; (void)ego;
    return;
#else
    static bool vis_ok = true;
    if (!vis_ok) return;
    try {
        plt::figure(1);
        plt::clf();

        // 1. 绘制参考线
        std::vector<double> rx, ry;
        for (const auto& p : reference_path) {
            rx.push_back(p.x);
            ry.push_back(p.y);
        }
        plt::plot(rx, ry, {{"color", "blue"}, {"linewidth", "2"}});
        if (!rx.empty()) plt::text(rx.front(), ry.front(), "Reference Start");

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
        if (!tx.empty()) {
            plt::plot(tx, ty, {{"color", "green"}, {"linewidth", "2"}});
            plt::text(tx.front(), ty.front(), "Trajectory Start");
        }

        // 4. 绘制自车位置
        std::vector<double> ex = {ego.x};
        std::vector<double> ey = {ego.y};
        plt::scatter(ex, ey, 80.0, {{"c", "yellow"}, {"edgecolors", "black"}});
        plt::text(ego.x, ego.y, "Ego");

        // 图形设置
        plt::xlabel("X (m)");
        plt::ylabel("Y (m)");
        plt::title("Lattice Planner Visualization");
        plt::axis("equal");
        plt::grid(true);
        plt::xlim(ego.x - 30.0, ego.x + 30.0);
        plt::ylim(ego.y - 30.0, ego.y + 30.0);

        plt::pause(0.001);
    } catch (const std::exception &e) {
        std::cerr << "[visualize] plotting error: " << e.what() << "\n";
        vis_ok = false; // disable further attempts
    } catch (...) {
        std::cerr << "[visualize] plotting error: unknown exception\n";
        vis_ok = false;
    }
#endif
}

int main() {
    std::cout << "Lattice Planner test copied from EMPlanner scenario" << std::endl;

    // 1. 创建规划器
    std::cout << "Creating lattice planner..." << std::endl;
    latticePlanner planner;

    PlannerParams params;
    params.sampling.sample_max_time = 8.0;
    params.sampling.sample_min_time = 3.0;
    params.sampling.sample_time_step = 0.5;
    params.sampling.sample_lat_width = 2.0;
    params.sampling.sample_width_length = 0.5;
    params.sampling.sample_space_resolution = 0.3;
    params.cruise_speed = 4.0;
    params.weights.weight_st_object = 1.0;
    params.weights.weight_st_jerk = 1.0;
    params.weights.weight_lt_offset = 1.0;
    params.limits.max_speed = 25.0;
    params.limits.max_acc = 5.0;
    params.limits.max_jerk = 50.0;
    params.limits.max_curvature = 1000.0;

    planner.setPlannerParams(params);

    // 2. 参考线
    std::cout << "Generating curved reference line..." << std::endl;
    auto reference_path = generate_path();
    general::FrenetFrame ref_line(reference_path);
    std::cout << "Reference points: " << reference_path.size() << std::endl;

    if (!planner.setGlobalReferenceLine(reference_path)) {
        std::cerr << "Failed to set reference line" << std::endl;
        return 1;
    }

    auto ego = std::make_shared<general::VehicleState>();
    ego->x = reference_path[0].x + 1.0;   // 向右偏1米
    ego->y = reference_path[0].y - 0.5;   // 向下偏0.5米
    ego->heading = reference_path[0].heading - 0.1; // 航向略偏左
    ego->v = 2.0;      
    ego->ax = 0.0;
    ego->ay = 0.0;
    ego->id = 1;

    // 3. 障碍物
    std::vector<general::Obstacle> obstacles;
    {
        general::Obstacle obs; obs.id = 101; obs.x = 40.0; obs.y = -1.5; obs.vx = 0.0; obs.vy = 0.0; obs.length = 4.5; obs.width = 2.0; obstacles.push_back(obs);
        general::Obstacle obs_mid; obs_mid.id = 103; obs_mid.x = 80.0; obs_mid.y = 0.0; obs_mid.vx = 0.0; obs_mid.vy = 0.0; obs_mid.length = 2.0; obs_mid.width = 2.0; obstacles.push_back(obs_mid);
        general::Obstacle obs_left; obs_left.id = 104; obs_left.x = 120.0; obs_left.y = 2.5; obs_left.vx = 0.0; obs_left.vy = 0.0; obs_left.length = 4.5; obs_left.width = 2.0; obstacles.push_back(obs_left);
        general::Obstacle obs_right; obs_right.id = 105; obs_right.x = 150.0; obs_right.y = -2.0; obs_right.vx = 0.0; obs_right.vy = 0.0; obs_right.length = 4.5; obs_right.width = 2.0; obstacles.push_back(obs_right);
    }

    double reference_speed = 4.0;
    double current_time = 0.0;

    // 4. 首次规划
    std::cout << "Executing initial planning..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto trajectory = planner.plan(ego, obstacles, reference_speed, current_time);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if (!trajectory.empty()) {
        std::cout << "\n✓ Initial planning succeeded!" << std::endl;
        std::cout << "  Planning time: " << duration.count() << " ms" << std::endl;
        std::cout << "  Trajectory points: " << trajectory.size() << std::endl;
#if HAVE_MATPLOTLIBCPP
        if (std::getenv("PLOT")) {
            visualize(reference_path, obstacles, trajectory, *ego);
        } else {
            std::cout << "[visualize] plotting disabled (set PLOT=1 to enable)" << std::endl;
        }
#endif
    } else {
        std::cout << "Initial planning failed." << std::endl;
        return 1;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> lat_noise(0.0, 0.10);
    std::normal_distribution<double> lon_noise(0.0, 0.2);
    std::normal_distribution<double> speed_noise(0.0, 0.2);
    std::normal_distribution<double> heading_noise(0.0, 0.03);

    const int total_cycles = 250;
    for (int cycle = 1; cycle <= total_cycles; ++cycle) {
        double dt_cycle = 0.2;
        double target_time = current_time + dt_cycle;
        general::FrenetFrame ref_trj(trajectory);
        auto trj_msg=ref_trj.get_matched_trj_point(target_time);
        auto cur_msg=ref_trj.get_matched_trj_point(current_time);
        auto planned = trj_msg.first;

        double dx_lon = lon_noise(gen);
        double dy_lat = lat_noise(gen);
        double dv = speed_noise(gen);
        double d_heading = heading_noise(gen);

        double ego_x_new = planned.x + dx_lon * std::cos(planned.heading) - dy_lat * std::sin(planned.heading);
        double ego_y_new = planned.y + dx_lon * std::sin(planned.heading) + dy_lat * std::cos(planned.heading);

        ego->x = ego_x_new;
        ego->y = ego_y_new;
        ego->v = std::max(0.0, planned.v + dv);
        ego->heading = planned.heading + d_heading;
        ego->ax = planned.ax;
        ego->ay = planned.ay;
        current_time = target_time;

        for (auto& obs : obstacles) {
            if (obs.id == 102) { // 切入车
                obs.x += obs.vx * dt_cycle;
                obs.y += obs.vy * dt_cycle;
            }
        }

        auto new_traj = planner.plan(ego, obstacles, reference_speed, current_time);
        if (!new_traj.empty()) {
            std::cout << "  ✓ Replanning succeeded. New traj points: " << new_traj.size() << std::endl;
            trajectory = new_traj;
        } else {
            std::cout << "  ✗ Replanning failed at cycle " << cycle << std::endl;
            break;
        }

        if (cycle % 10 == 0) {
            std::cout << "Cycle " << cycle << ": trajectory size=" << trajectory.size() << std::endl;
            // Visualization is intentionally only run once after initial planning to avoid
            // runtime-python import errors on some systems. Enable manually if needed.
        }
    }

    std::cout << "Advanced test completed." << std::endl;
    return 0;
}