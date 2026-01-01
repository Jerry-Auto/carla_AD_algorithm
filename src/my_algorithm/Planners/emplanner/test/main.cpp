#include "emplanner/emplanner.h"
#include "emplanner/DP_solver.h"  
#include "general_modules/Trajectory.h"  // 包含 PathPoint 定义
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>
#include <random>
#include "general_modules/matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace AD_algorithm;

void test_planner();
int main() {
    // planner::demo();
    test_planner();
    return 0;
}


void visualize(
    const std::vector<general::PathPoint>& reference_path,
    const std::vector<general::Obstacle>& obstacles,
    const std::vector<general::TrajectoryPoint>& trajectory,
    const general::VehicleState& ego)
{
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
    plt::title("EMPlanner Visualization");
    plt::axis("equal");
    plt::grid(true);
    plt::xlim(ego.x - 30.0, ego.x + 30.0);
    plt::ylim(ego.y - 30.0, ego.y + 30.0);

    plt::pause(0.001);
    plt::plot();
}
std::vector<general::PathPoint> generate_path(){
    std::vector<general::PathPoint> reference_path;
    const int total_length = 700; // 总长700米
    const double ds = 0.3;        // 1米间隔
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
    // const double s5_end = 700.0;                 // 最后直线结束


    for (int i = 0; i <= num_points; ++i) {
        double s = i * ds;
        general::PathPoint point;
        point.x = 0.0;
        point.y = 0.0;
        point.heading = 0.0;
        point.kappa = 0.0;

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
            // 右转起始 heading 是 theta_total（来自左转结束）
            double theta_start = arc_length / R_left;
            // 右转圆心：从直线终点向“左”偏移 R（因为右转，圆心在轨迹左侧？注意方向！）
            // 实际上，右转时曲率为负，圆心在右侧
            // 更简单：以起始点为基准，顺时针绕圆
            double x_start = s1_end + R_left * std::sin(theta_start) + 200.0 * std::cos(theta_start);
            double y_start = R_left * (1.0 - std::cos(theta_start)) + 200.0 * std::sin(theta_start);
            // 右转：heading 减小，圆心在 (x_start + R*sin(theta_start), y_start - R*cos(theta_start))?
            // 更稳健：用局部坐标系
            point.x = x_start + R_right * (std::sin(theta_start) - std::sin(theta_start - theta));
            point.y = y_start + R_right * (std::cos(theta_start - theta) - std::cos(theta_start));
            point.heading = theta_start - theta; // heading 减小
            point.kappa = -1.0 / R_right; // 右转曲率为负
        }
        else {
            // 第五段：最后直线
            double delta_s = s - s4_end;
            double theta_after_right = arc_length / R_left - arc_length / R_right; // 最终 heading
            // 计算右转结束点坐标
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

void test_planner() {
    plt::backend("Qt5Agg");
    std::cout << "EMPlanner Advanced Test Program" << std::endl;
    std::cout << "=================================" << std::endl;

    // 1. 创建规划器
    std::cout << "Creating EMPlanner..." << std::endl;
    auto emplanner = std::make_shared<planner::EMPlanner>();

    // [FIX] 配置规划器参数，减小S采样步长以应对近距离障碍物
    planner::PathPlannerConfig config;
    config.s_sample_distance = 1.0;
    config.s_sample_number = 100;
    emplanner->setPathPlannerConfig(config);

    // 2. 创建弯曲参考线（模拟弯道）
    std::cout << "Creating curved reference line..." << std::endl;
    std::vector<general::PathPoint> reference_path=generate_path();
    general::FrenetFrame ref_l(reference_path);
    std::cout << "Created " << reference_path.size() << " curved reference points" << std::endl;
    std::cout << "Start: (" << reference_path.front().x << ", " << reference_path.front().y << ")" << std::endl;
    std::cout << "End:   (" << reference_path.back().x << ", " << reference_path.back().y << ")" << std::endl;
    std::cout << "总长S:   (" << ref_l[ref_l.size()-1].accumulated_s << std::endl;


    bool ref_line_set = emplanner->setGlobalReferenceLine(reference_path);
    if (!ref_line_set) {
        std::cout << "Error: Failed to set reference line!" << std::endl;
        return;
    }

    // 3. 自车初始状态：略微偏离参考线，带一定航向偏差
    std::cout << "\nCreating ego vehicle state (offset from reference)..." << std::endl;
    auto ego_state = std::make_shared<general::VehicleState>();
    ego_state->x = reference_path[0].x + 1.0;   // 向右偏1米
    ego_state->y = reference_path[0].y - 0.5;   // 向下偏0.5米
    ego_state->heading = reference_path[0].heading - 0.1; // 航向略偏左
    ego_state->v = 2.0;      // 初始速度6 m/s
    ego_state->ax = 0.0;
    ego_state->ay = 0.0;
    ego_state->id = 1;

    // 4. 创建复杂障碍物场景
    std::cout << "Creating complex obstacle scenario..." << std::endl;
    std::vector<general::Obstacle> obstacles;

    // 场景1：静态障碍物（路边违停车辆）
    {
        general::Obstacle obs;
        obs.id = 101;
        obs.x = 40.0;
        obs.y = -1.5; // 占用部分右侧车道
        obs.vx = 0.0;
        obs.vy = 0.0;
        obs.length = 4.5;
        obs.width = 2.0;
        obs.height = 1.6;
        obs.heading = 0.0;
        obstacles.push_back(obs);
    }

    // // 场景2：切入车辆（从右侧汇入）
    {
        general::Obstacle cut_in_car;
        cut_in_car.id = 102;
        cut_in_car.x = 55.0; // 在自车前方
        cut_in_car.y = -4.0; // 在右侧车道/路肩
        cut_in_car.vx = 5.0; // 纵向速度
        cut_in_car.vy = 1.0; // 向左切入
        cut_in_car.length = 4.5;
        cut_in_car.width = 1.8;
        cut_in_car.height = 1.4;
        obstacles.push_back(cut_in_car);
    }

    // 场景3：路中间静态障碍物（强制变道）
    {
        general::Obstacle obs_mid;
        obs_mid.id = 103;
        obs_mid.x = 80.0;
        obs_mid.y = 0.0; // 路中间
        obs_mid.vx = 0.0;
        obs_mid.vy = 0.0;
        obs_mid.length = 2.0; 
        obs_mid.width = 2.0;
        obs_mid.height = 1.0;
        obstacles.push_back(obs_mid);
    }

    // 场景4：远处左侧静态障碍物
    {
        general::Obstacle obs_left;
        obs_left.id = 104;
        obs_left.x = 120.0;
        obs_left.y = 2.5; // 左侧
        obs_left.vx = 0.0;
        obs_left.vy = 0.0;
        obs_left.length = 4.5;
        obs_left.width = 2.0;
        obs_left.height = 1.6;
        obstacles.push_back(obs_left);
    }

    // 场景5：远处右侧静态障碍物
    {
        general::Obstacle obs_right;
        obs_right.id = 105;
        obs_right.x = 150.0;
        obs_right.y = -2.0; // 右侧
        obs_right.vx = 0.0;
        obs_right.vy = 0.0;
        obs_right.length = 4.5;
        obs_right.width = 2.0;
        obs_right.height = 1.6;
        obstacles.push_back(obs_right);
    }
    // 随机数生成器（用于模拟控制误差）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> lat_noise(0.0, 0.10);   // 横向噪声 σ=0.10m
    std::normal_distribution<double> lon_noise(0.0, 0.2);    // 纵向噪声 σ=0.2m
    std::normal_distribution<double> speed_noise(0.0, 0.2);  // 速度噪声 σ=0.2m/s
    std::normal_distribution<double> heading_noise(0.0, 0.03); // 航向噪声 σ=0.03rad
    std::cout << "Created " << obstacles.size() << " obstacles including dynamic and static." << std::endl;

    // 5. 设置参考速度（可随时间/位置变化，这里简化为常量）
    double reference_speed = 4.0; // 目标速度10 m/s (~36 km/h)
    double current_time = 0.0;

    // 6. 执行首次规划
    std::cout << "\nExecuting initial planning..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto trajectory = emplanner->plan(ego_state, obstacles, reference_speed, current_time);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    visualize(reference_path, obstacles, trajectory, *ego_state);


    if (!trajectory.empty()) {
        std::cout << "\n✓ Initial planning succeeded!" << std::endl;
        std::cout << "  Planning time: " << duration.count() << " ms" << std::endl;
        std::cout << "  Trajectory points: " << trajectory.size() << std::endl;
        // 输出前10个点
        std::cout << "\nFirst 10 trajectory points:" << std::endl;
        std::cout << "Idx | Time(s) | X(m)     | Y(m)     | V(m/s) | Heading(rad)" << std::endl;
        std::cout << "-------------------------------------------------------------" << std::endl;
        for (int i = 0; i < std::min(10, static_cast<int>(trajectory.size())); ++i) {
            const auto& p = trajectory[i];
            std::cout << std::setw(3) << i << " | "
                      << std::fixed << std::setprecision(3) << p.time_stamped << " | "
                      << std::setw(8) << std::setprecision(2) << p.x << " | "
                      << std::setw(8) << std::setprecision(2) << p.y << " | "
                      << std::setw(6) << std::setprecision(2) << p.v << " | "
                      << std::setw(12) << std::setprecision(3) << p.heading << std::endl;
        }

        // 模拟连续规划（10个周期）
        std::cout << "\nSimulating multi-cycle replanning with moving obstacles..." << std::endl;
        const int total_cycles = 250;
        for (int cycle = 1; cycle <= total_cycles; ++cycle) {
            std::cout << "\n--- Cycle " << cycle << " ---" << std::endl;
            general::FrenetFrame ref_trj(trajectory);
            // int count=0;
            // for(auto& t:trajectory){
            //     if(count<100){
            //         std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
            //         <<"，位置(x,y)："<<t.x<<","<<t.y
            //         <<"，S:"<<ref_trj[count].accumulated_s
            //         <<std::endl;
            //     }else{
            //         if(count%10==0){
            //             std::cout<<"时间："<<t.time_stamped<<"，速度："<<t.v
            //             <<"，位置(x,y)："<<t.x<<","<<t.y
            //             <<"，S:"<<ref_trj[count].accumulated_s
            //             <<std::endl;
            //         }
            //     }
            //     count++;
            // }
            
            // 1. 找到目标时间点对应的轨迹索引
            double dt_cycle = 0.2;
            double target_time = current_time + dt_cycle;
            auto trj_msg=ref_trj.get_matched_trj_point(target_time);
            auto cur_msg=ref_trj.get_matched_trj_point(current_time);   
            std::cout << "当前时间: " << current_time 
                    << ", Closest traj point time: " << cur_msg.first.time_stamped 
                    << "\n, 当前S: " << cur_msg.second
                    << "\n, 当前 (x,y): (" <<  cur_msg.first.x << ", " <<  cur_msg.first.y << ")" << std::endl; 

            const auto& planned = trj_msg.first;
            std::cout << "传送点 time: " << target_time 
                    << ", Closest traj point time: " << planned.time_stamped 
                    << "\n, 传送点S: " << trj_msg.second
                    << "\n, 传送点 (x,y): (" << planned.x << ", " << planned.y << ")" << std::endl;

            // 2. 更新自车状态（带控制噪声）
            double dx_lon = lon_noise(gen);
            double dy_lat = lat_noise(gen);
            double dv = speed_noise(gen);
            double d_heading = heading_noise(gen);

            // 坐标系旋转：沿车辆朝向分解噪声
            double ego_x_new = planned.x + dx_lon * std::cos(planned.heading) - dy_lat * std::sin(planned.heading);
            double ego_y_new = planned.y + dx_lon * std::sin(planned.heading) + dy_lat * std::cos(planned.heading);

            std::cout << " Noise: lon=" << dx_lon << ", lat=" << dy_lat 
            <<"\n当前时间："<<current_time<<"目标时间:"<<target_time
                    << " \nΔx=" << (ego_x_new - planned.x) 
                    << ", Δy=" << (ego_y_new - planned.y)
                    <<"\n当前速度："<<ego_state->v<<"目标速度："<<planned.v
                    << std::endl;

            ego_state->x = ego_x_new;
            ego_state->y = ego_y_new;
            ego_state->v = std::max(0.0, planned.v + dv);
            ego_state->heading = planned.heading + d_heading;
            ego_state->ax = planned.ax;
            ego_state->ay = planned.ay;
            current_time = target_time;

            std::cout << "Updated ego: x=" << std::fixed << std::setprecision(4) << ego_state->x
                    << ", y=" << ego_state->y
                    << ", v=" << ego_state->v
                    << ", heading=" << ego_state->heading
                    << ", t=" << current_time << "s" << std::endl;

            // 3. 更新动态障碍物
            for (auto& obs : obstacles) {
                double dt_obs = dt_cycle; // 保持与自车更新步长一致
                if (obs.id == 102) { // 切入车
                    obs.x += obs.vx * dt_obs;
                    obs.y += obs.vy * dt_obs;
                }
            }

            // 4. 重新规划
            auto new_traj = emplanner->plan(ego_state, obstacles, reference_speed, current_time);
            if (!new_traj.empty()) {
                std::cout << "  ✓ Replanning succeeded. New traj points: " << new_traj.size() << std::endl;
                trajectory = new_traj;

                // 可选：检查新轨迹起点是否接近当前 ego
                if (!trajectory.empty()) {
                    const auto& first_point = trajectory.front();
                    double dist_to_ego = std::hypot(first_point.x - ego_state->x, first_point.y - ego_state->y);
                    std::cout << "  [DEBUG] Distance from ego to new traj start: " << dist_to_ego << " m" << std::endl;
                }
            } else {
                std::cout << "  ✗ Replanning failed at cycle " << cycle << std::endl;
                break;
            }

            if (cycle % 5== 0) {
                if (!trajectory.empty()) {
                    // std::cout << "  [VISUALIZE] Showing plot at cycle " << cycle << std::endl;
                    visualize(reference_path, obstacles, trajectory, *ego_state);
                }
            }
        }
    }

    std::cout << "\nAdvanced test completed." << std::endl;
    // plt::show();
}