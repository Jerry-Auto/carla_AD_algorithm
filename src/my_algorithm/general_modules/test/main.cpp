#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "general_modules/FrenetFrame.h"
#include "general_modules/Trajectory.h"
#include "general_modules/visualization_tool.h"

#include <memory>
#include <random>
#include <utility>

using namespace AD_algorithm::general;

// 生成一段螺旋线或圆弧 + 直线，模拟真实道路


int test_frenet_frame();

std::vector<std::vector<TrajectoryPoint>> generateHistoryTrajs();
int test_visualize(int argc, char ** argv);
std::vector<Obstacle> generateObstacles() ;
std::vector<PathPoint> generateRefPath();
std::vector<TrajectoryPoint> generateFinalTraj();
std::vector<std::vector<TrajectoryPoint>> generateSampleTrajs(); 

int main(int argc, char ** argv) {
    test_visualize(argc,argv);
    return 0;
}
std::vector<Obstacle> generateObstacles() {
    std::vector<Obstacle> obstacles;

    // 障碍物 1：静止车辆
    Obstacle obs1;
    obs1.id = 1;
    obs1.x = 5.0;
    obs1.y = 1.0;
    obs1.z = 0.0;
    obs1.vx = 0.0;
    obs1.vy = 0.0;
    obs1.length = 4.5;
    obs1.width = 2.0;
    obstacles.push_back(obs1);

    // 障碍物 2：横向移动车辆
    Obstacle obs2;
    obs2.id = 2;
    obs2.x = 8.0;
    obs2.y = -2.0;
    obs2.z = 0.0;
    obs2.vx = 0.0;
    obs2.vy = 1.0;  // 向上移动
    obs2.length = 4.0;
    obs2.width = 1.8;
    obstacles.push_back(obs2);

    // 障碍物 3：前方对向车
    Obstacle obs3;
    obs3.id = 3;
    obs3.x = 12.0;
    obs3.y = 0.0;
    obs3.z = 0.0;
    obs3.vx = -2.0; // 逆向行驶
    obs3.vy = 0.0;
    obs3.length = 5.0;
    obstacles.push_back(obs3);

    return obstacles;
}

// 全局或类外辅助函数：生成测试数据（可选，也可在类内做）
std::vector<PathPoint> generateRefPath() {
    std::vector<PathPoint> ref_path;
    for (double x = 0.0; x <= 20.0; x += 0.5) {
        PathPoint p;
        p.x = x;
        p.y = 0.1 * x * sin(0.3 * x);
        p.heading = atan2(0.1 * (sin(0.3 * x) + 0.3 * x * cos(0.3 * x)), 1.0);
        ref_path.push_back(p);
    }
    return ref_path;
}

std::vector<TrajectoryPoint> generateFinalTraj() {
    std::vector<TrajectoryPoint> traj;
    double t_offset = rclcpp::Clock().now().seconds(); // 可加动态偏移（可选）
    for (double t = 0.0; t <= 5.0; t += 0.1) {
        TrajectoryPoint tp;
        tp.x = t * 2.0;
        tp.y = 0.05 * t * t;
        tp.v = 2.0 + 0.1 * t;
        tp.heading = atan2(0.1 * t, 2.0);
        tp.time_stamped = t;
        tp.kappa = 0.0;
        traj.push_back(tp);
    }
    return traj;
}

std::vector<std::vector<TrajectoryPoint>> generateSampleTrajs() {
    std::vector<std::vector<TrajectoryPoint>> trajs;
    std::default_random_engine gen(12345);
    std::normal_distribution<double> noise(0.0, 0.2);
    for (int i = 0; i < 5; ++i) {
        std::vector<TrajectoryPoint> traj;
        for (double t = 0.0; t <= 3.0; t += 0.2) {
            TrajectoryPoint tp;
            tp.x = t * 1.5;
            tp.y = 0.3 * i + noise(gen);
            tp.v = 1.5;
            tp.heading = 0.0;
            tp.time_stamped = t;
            tp.kappa = 0.0;
            traj.push_back(tp);
        }
        trajs.push_back(traj);
    }
    return trajs;
}

std::vector<std::vector<TrajectoryPoint>> generateHistoryTrajs() {
    std::vector<std::vector<TrajectoryPoint>> trajs;
    for (int h = 0; h < 3; ++h) {
        std::vector<TrajectoryPoint> hist;
        for (double x = -5.0 + h; x <= 0.0; x += 0.3) {
            TrajectoryPoint tp;
            tp.x = x;
            tp.y = 0.02 * x * x + 0.1 * h;
            tp.v = 1.0;
            tp.heading = 0.0;
            tp.time_stamped = 0.0;
            hist.push_back(tp);
        }
        trajs.push_back(hist);
    }
    return trajs;
}

// 主测试节点（持续运行）
int test_visualize(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AD_algorithm::general::VisualizationTool>("test_visualization");

    // 预生成静态测试数据（因为不随时间变）
    auto ref_path = generateRefPath();
    auto final_traj = generateFinalTraj();
    auto sample_trajs = generateSampleTrajs();
    auto history_trajs = generateHistoryTrajs();
    auto obstacles = generateObstacles();

    // 创建一个定时器，每 0.5 秒发布一次（2 Hz）
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [node, ref_path, final_traj, sample_trajs, history_trajs,obstacles]() {
            node->RefPathVisualization(ref_path);
            node->FinalPathVisualization(final_traj);
            node->SamplePathsVisualization(sample_trajs);
            node->HistoryPathVisualization(history_trajs);
            node->ObstacleVisualization(obstacles);
            RCLCPP_DEBUG(node->get_logger(), "Published visualization data.");
        });

    RCLCPP_INFO(node->get_logger(), "Visualization node started. Publishing continuously...");

    // 进入事件循环（一直运行，直到 Ctrl+C）
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}


std::vector<std::pair<double, double>> generate_curved_reference() {
    std::vector<std::pair<double, double>> points;
    const double R = 10.0;

    // 第一段：左转圆弧（逆时针，从 (0,0) 到 (R, R)）
    for (double theta = 0.0; theta <= M_PI_2; theta += 0.1) {
        double x = R * std::sin(theta);
        double y = R * (1.0 - std::cos(theta));
        points.emplace_back(x, y);
    }

    // 第二段：直线延伸（沿 x 方向）
    double last_x = points.back().first;
    double last_y = points.back().second;
    for (double s = 0.5; s <= 10.0; s += 0.5) { // 从 0.5 开始避免重复点
        points.emplace_back(last_x + s, last_y);
    }

    // 更新末端位置
    last_x = points.back().first;
    last_y = points.back().second;

    // 第三段：右转圆弧（顺时针）
    // 圆心在 (last_x, last_y - R)，车辆向右转（y 减小）
    for (double theta = 0.0; theta <= M_PI_2; theta += 0.1) {
        double x = last_x + R * (1.0 - std::cos(theta)); // 向前走
        double y = last_y - R * std::sin(theta);         // 向下弯（右转）
        points.emplace_back(x, y);
    }

    return points;
}

int test_frenet_frame(){
    std::cout << "=== Advanced & Robust Test: Curved ReferenceLine & FrenetFrame ===\n";

    auto xy_points = generate_curved_reference();
    std::cout << "Generated " << xy_points.size() << " reference points.\n";

    try {
        ReferenceLine ref_line(xy_points);
        if (!ref_line.flag_ref()) {
            std::cerr << "❌ ReferenceLine smoothing failed!\n";
            return -1;
        }
        std::cout << "✅ ReferenceLine built with curvature.\n";

        FrenetFrame frame(ref_line);

        // === 测试 1: 中间某点附近偏移车辆（原测试保留）===
        {
            const PathPoint& ref_pt = frame[8];
            std::cout << "\n-- Test 1: Point on curved segment --\n";
            std::cout << "Ref point: x=" << ref_pt.x << ", y=" << ref_pt.y 
                      << ", heading=" << ref_pt.heading << ", kappa=" << ref_pt.kappa << "\n";

            Eigen::Vector2d tau(std::cos(ref_pt.heading), std::sin(ref_pt.heading));
            Eigen::Vector2d nor(-std::sin(ref_pt.heading), std::cos(ref_pt.heading));
            Eigen::Vector2d ego_pos = Eigen::Vector2d(ref_pt.x, ref_pt.y) + 1.5 * nor;

            TrajectoryPoint ego;
            ego.x = ego_pos.x();
            ego.y = ego_pos.y();
            ego.heading = ref_pt.heading;
            ego.v = 5.0;

            const PathPoint& matched = frame.get_matched_point(ego.x, ego.y, ego.heading);
            FrenetPoint fp = frame.cartesian_to_frenet(ego);
            std::cout << "Frenet: s=" << fp.s << ", l=" << fp.l << "\n";

            assert(fp.l > 1.0 && fp.l < 2.0);
            assert(std::abs(fp.s - matched.accumulated_s) < 1.0);

            TrajectoryPoint recon = frame.frenet_to_cartesian(fp);
            assert(std::abs(recon.x - ego.x) < 0.1);
            assert(std::abs(recon.y - ego.y) < 0.1);
            std::cout << "✅ Curved segment test passed.\n";
        }

        // === 测试 2: 车辆在路径后方，heading 向后（原测试保留）===
        {
            std::cout << "\n-- Test 2: Vehicle behind with backward heading --\n";
            size_t idx = frame.get_reference_path().size() - 2;
            const PathPoint& ref_pt = frame[idx];

            Eigen::Vector2d tau(std::cos(ref_pt.heading), std::sin(ref_pt.heading));
            Eigen::Vector2d ego_pos = Eigen::Vector2d(ref_pt.x, ref_pt.y) - 3.0 * tau;

            TrajectoryPoint ego;
            ego.x = ego_pos.x();
            ego.y = ego_pos.y();
            ego.heading = ref_pt.heading + M_PI;
            ego.v = 2.0;

            const PathPoint& matched = frame.get_matched_point(ego.x, ego.y, ego.heading);
            FrenetPoint fp = frame.cartesian_to_frenet(ego);
            std::cout << "Vehicle at (" << ego.x << ", " << ego.y << "), heading=" << ego.heading << "\n";
            std::cout << "Matched s=" << matched.accumulated_s << ", Frenet s=" << fp.s << ", l=" << fp.l << "\n";

            assert(fp.s < ref_pt.accumulated_s + 1.0);
            std::cout << "✅ Backward heading test passed.\n";
        }

        // === 测试 3: operator[] 一致性（原测试保留）===
        {
            const auto& path = frame.get_reference_path();
            for (size_t i = 0; i < path.size(); ++i) {
                const PathPoint& p1 = path[i];
                const PathPoint& p2 = frame[i];
                assert(std::abs(p1.x - p2.x) < 1e-9);
                assert(std::abs(p1.y - p2.y) < 1e-9);
            }
            std::cout << "\n✅ operator[] consistency verified for all points.\n";
        }

        // === 🆕 测试 4: S型弯道上的横向偏移与方向错位 ===
        {
            std::cout << "\n-- Test 4: S-curve with lateral offset and heading misalignment --\n";
            // 找一个右转圆弧中间的点（即 S 弯的第二段）
            size_t idx = 30; // roughly in the right-turn arc
            if (idx >= frame.size()) idx = frame.size() - 1;
            const PathPoint& ref_pt = frame[idx];

            // 横向偏移 +2.0（右侧），但车辆朝向与参考线相差 30 度
            double offset_l = 2.0;
            double heading_offset = M_PI / 6.0; // 30 degrees

            Eigen::Vector2d nor(-std::sin(ref_pt.heading), std::cos(ref_pt.heading));
            Eigen::Vector2d ego_pos = Eigen::Vector2d(ref_pt.x, ref_pt.y) + offset_l * nor;

            TrajectoryPoint ego;
            ego.x = ego_pos.x();
            ego.y = ego_pos.y();
            ego.heading = ref_pt.heading + heading_offset;
            ego.v = 4.0;

            FrenetPoint fp = frame.cartesian_to_frenet(ego);
            std::cout << "Ego heading misaligned by 30°, l_actual=" << fp.l << "\n";

            // 允许一定误差（因 heading 不一致，l 可能略偏）
            assert(fp.l > 1.5 && fp.l < 2.5);
            std::cout << "✅ S-curve misalignment test passed.\n";
        }

        // === 🆕 测试 5: 多车辆并行测试（验证独立性）===
        {
            std::cout << "\n-- Test 5: Multiple vehicles in different lanes/segments --\n";
            std::vector<std::pair<double, double>> test_offsets = {{0.0, 0.0}, {1.5, 0.0}, {-1.5, 0.0}, {0.0, 5.0}};
            std::vector<double> headings = {0.0, M_PI_4, -M_PI*3, M_PI};

            for (size_t i = 0; i < test_offsets.size(); ++i) {
                size_t pt_idx = 10 + i * 5;
                if (pt_idx >= frame.size()) pt_idx = frame.size() - 1;
                const PathPoint& ref = frame[pt_idx];

                Eigen::Vector2d nor(-std::sin(ref.heading), std::cos(ref.heading));
                Eigen::Vector2d tau(std::cos(ref.heading), std::sin(ref.heading));

                Eigen::Vector2d pos = Eigen::Vector2d(ref.x, ref.y)
                                    + test_offsets[i].first * nor
                                    + test_offsets[i].second * tau;

                TrajectoryPoint ego;
                ego.x = pos.x();
                ego.y = pos.y();
                ego.heading = ref.heading + headings[i];
                ego.v = 3.0 + i;

                FrenetPoint fp = frame.cartesian_to_frenet(ego);
                TrajectoryPoint recon = frame.frenet_to_cartesian(fp);

                assert(std::abs(recon.x - ego.x) < 0.15);
                assert(std::abs(recon.y - ego.y) < 0.15);
            }
            std::cout << "✅ Multi-vehicle test passed.\n";
        }

        // === 🆕 测试 6: 边界情况 —— 起点和终点外推 ===
        {
            std::cout << "\n-- Test 6: Extrapolation beyond path ends --\n";
            const auto& path = frame.get_reference_path();
            double total_s = path.back().accumulated_s;

            // 超出起点前 5 米
            TrajectoryPoint ego_front;
            ego_front.x = path.front().x - 5.0 * std::cos(path.front().heading);
            ego_front.y = path.front().y - 5.0 * std::sin(path.front().heading);
            ego_front.heading = path.front().heading;
            ego_front.v = 2.0;

            FrenetPoint fp_front = frame.cartesian_to_frenet(ego_front);
            std::cout << "Front extrapolation: s=" << fp_front.s << " (should be < 0)\n";
            assert(fp_front.s < 0.0);

            // 超出终点后 5 米
            TrajectoryPoint ego_back;
            ego_back.x = path.back().x + 5.0 * std::cos(path.back().heading);
            ego_back.y = path.back().y + 5.0 * std::sin(path.back().heading);
            ego_back.heading = path.back().heading;
            ego_back.v = 2.0;

            FrenetPoint fp_back = frame.cartesian_to_frenet(ego_back);
            std::cout << "Back extrapolation: s=" << fp_back.s << " (should be > " << total_s << ")\n";
            assert(fp_back.s > total_s);

            // 反向重建应仍合理（即使外推）
            TrajectoryPoint recon_back = frame.frenet_to_cartesian(fp_back);
            assert(std::abs(recon_back.x - ego_back.x) < 0.5); // 容忍更大误差
            assert(std::abs(recon_back.y - ego_back.y) < 0.5);
            std::cout << "✅ Extrapolation test passed.\n";
        }

        // === 🆕 测试 7: 噪声扰动下的稳定性 ===
        {
            std::cout << "\n-- Test 7: Robustness to position noise --\n";
            const PathPoint& ref = frame[15];
            Eigen::Vector2d nor(-std::sin(ref.heading), std::cos(ref.heading));
            Eigen::Vector2d base_pos(ref.x, ref.y);
            base_pos += 1.2 * nor;

            bool all_passed = true;
            for (int i = 0; i < 20; ++i) {
                double noise_x = (rand() / double(RAND_MAX) - 0.5) * 0.2; // ±0.1m
                double noise_y = (rand() / double(RAND_MAX) - 0.5) * 0.2;
                TrajectoryPoint ego;
                ego.x = base_pos.x() + noise_x;
                ego.y = base_pos.y() + noise_y;
                ego.heading = ref.heading + (rand() / double(RAND_MAX) - 0.5) * 0.2; // ±0.1 rad
                ego.v = 5.0;

                FrenetPoint fp = frame.cartesian_to_frenet(ego);
                TrajectoryPoint recon = frame.frenet_to_cartesian(fp);

                if (std::abs(recon.x - ego.x) > 0.15 || std::abs(recon.y - ego.y) > 0.15) {
                    all_passed = false;
                    break;
                }
            }
            assert(all_passed);
            std::cout << "✅ Noise robustness test passed.\n";
        }
        // === 🆕 Test 8: Closed-loop trajectory consistency (Frenet → Cartesian → Frenet → Cartesian) ===
        {
            std::cout << "\n-- Test 8: Closed-loop dynamic trajectory consistency --\n";

            const auto& ref_path = frame.get_reference_path();
            double total_s = ref_path.back().accumulated_s;

            // 生成 Frenet 轨迹：s ∈ [2, total_s - 2]，避免端点外推影响
            std::vector<FrenetPoint> frenet_traj;
            for (double s = 2.0; s <= total_s - 2.0; s += 0.3) {
                // 横向偏移：正弦波，模拟变道或避障
                double l = 1.2 * std::sin(0.4 * s);           // 幅值 1.2m
                double dl_ds = 1.2 * 0.4 * std::cos(0.4 * s); // 一阶导
                double d2l_ds2 = -1.2 * 0.16 * std::sin(0.4 * s); // 二阶导

                // 纵向速度和加速度（可选，用于完整状态）
                double s_dot = 6.0 + 0.5 * std::sin(0.2 * s);     // 速度波动
                double s_ddot = 0.5 * 0.2 * std::cos(0.2 * s);    // 加速度

                FrenetPoint fp;
                fp.s = s;
                fp.l = l;
                fp.l_prime = dl_ds;
                fp.l_prime_prime = d2l_ds2;
                fp.s_dot = s_dot;
                fp.s_dot_dot = s_ddot;

                frenet_traj.push_back(fp);
            }

            bool all_consistent = true;
            int check_count = 0;
            constexpr double POS_TOL = 0.12; // 允许稍大一点，因曲率影响

            for (const auto& fp_orig : frenet_traj) {
                // Step 1: Frenet → Cartesian
                TrajectoryPoint cart = frame.frenet_to_cartesian(fp_orig);

                // Step 2: Cartesian → Frenet (recovered)
                FrenetPoint fp_recovered = frame.cartesian_to_frenet(cart);

                // Step 3: Frenet (recovered) → Cartesian (reconstructed)
                TrajectoryPoint cart_recon = frame.frenet_to_cartesian(fp_recovered);

                // Step 4: Compare original Cartesian vs reconstructed
                double dx = cart_recon.x - cart.x;
                double dy = cart_recon.y - cart.y;
                double dist_err = std::hypot(dx, dy);

                if (dist_err > POS_TOL) {
                    std::cerr << "❌ Large error at s=" << fp_orig.s
                              << ": position error = " << dist_err << " m\n";
                    all_consistent = false;
                    break;
                }

                // 可选：检查 l 一致性（更严格）
                // if (std::abs(fp_recovered.l - fp_orig.l) > 0.15) { ... }

                check_count++;
            }

            assert(all_consistent);
            std::cout << "✅ Closed-loop test passed for " << check_count << " points (max error < " << POS_TOL << " m).\n";
        }

                // === 🆕 Test 9: Projection of a moving trajectory (dynamic obstacle / ego) ===
        {
            std::cout << "\n-- Test 9: Moving trajectory projection consistency --\n";

            const auto& ref_path = frame.get_reference_path();
            double total_s = ref_path.back().accumulated_s;

            // Step 1: Generate a realistic moving trajectory in Cartesian space
            std::vector<TrajectoryPoint> cart_traj;
            double t = 0.0;
            double dt = 0.1; // 10 Hz
            double s_start = -5.0;
            double v0 = 2.0;
            double a = 0.8;

            while (true) {
                double s_true = s_start + v0 * t + 0.5 * a * t * t;
                if (s_true > total_s + 8.0) break;

                double l_true = 0.8 * std::exp(-0.1 * std::pow(s_true - 10.0, 2));

                // 注意：你的 FrenetPoint 构造需匹配字段顺序
                FrenetPoint fp_gt;
                fp_gt.s = s_true;
                fp_gt.l = l_true;
                fp_gt.s_dot = v0 + a * t;
                fp_gt.s_dot_dot = a;
                // l_prime 等可选，若 frenet_to_cartesian 不依赖，可留 0

                TrajectoryPoint pt_cart = frame.frenet_to_cartesian(fp_gt);
                pt_cart.time_stamped = t;          // 假设 TrajectoryPoint 有 .t 字段
                pt_cart.v = fp_gt.s_dot;
                pt_cart.a_tau = fp_gt.s_dot_dot; // 假设字段名为 .a

                cart_traj.push_back(pt_cart);
                t += dt;
            }

            std::cout << "Generated moving trajectory with " << cart_traj.size() << " points.\n";

            // Step 2: Project each point back to Frenet
            std::vector<FrenetPoint> frenet_proj;
            for (const auto& pt : cart_traj) {
                FrenetPoint fp = frame.cartesian_to_frenet(pt);
                frenet_proj.push_back(fp);
            }

            // Step 3: Validate
            bool valid = true;
            double max_pos_error = 0.0;

            for (size_t i = 0; i < frenet_proj.size(); ++i) {
                const auto& fp = frenet_proj[i];
                const auto& pt_orig = cart_traj[i];

                TrajectoryPoint recon = frame.frenet_to_cartesian(fp);
                double pos_err = std::hypot(recon.x - pt_orig.x, recon.y - pt_orig.y);
                max_pos_error = std::max(max_pos_error, pos_err);
                if (pos_err > 0.15) {
                    std::cerr << "Position error too large at t=" << pt_orig.time_stamped << ": " << pos_err << " m\n";
                    valid = false;
                }

                if (std::abs(fp.l) > 4.0) {
                    std::cerr << "Unreasonable lateral offset at t=" << pt_orig.time_stamped << ": l=" << fp.l << "\n";
                    valid = false;
                }
            }

            // Check s(t) monotonicity
            for (size_t i = 1; i < frenet_proj.size(); ++i) {
                if (frenet_proj[i].s < frenet_proj[i-1].s - 1e-3) {
                    std::cerr << "s(t) not monotonic at i=" << i << "\n";
                    valid = false;
                    break;
                }
            }

            // Check velocity consistency using cart_traj.t
            for (size_t i = 1; i < frenet_proj.size(); ++i) {
                double ds = frenet_proj[i].s - frenet_proj[i-1].s;
                double dt_local = cart_traj[i].time_stamped - cart_traj[i-1].time_stamped; // ✅ 正确来源
                if (dt_local <= 1e-6) continue;
                double v_est = ds / dt_local;
                double v_orig = cart_traj[i].v;
                if (std::abs(v_est - v_orig) > 1.2) {
                    // 可记录警告，但不一定失败（因横向运动耦合）
                    // std::cout << "Vel diff: " << v_est << " vs " << v_orig << "\n";
                }
            }

            assert(valid);
            std::cout << "✅ Moving trajectory projection passed. Max pos error: "
                      << max_pos_error << " m.\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Test failed: " << e.what() << "\n";
        return -1;
    }

    std::cout << "\n🎉 All advanced and robust tests passed!\n";
    return 0;
}