# Lattice Planner (refactor) - README

🚗 简介

本目录包含对原有 `lattice` 算法的重构实现，目标是把算法拆分为职责清晰、可复用的模块：

- Trajectory1DGenerator：生成纵向/横向 1D 候选（多项式）
- TrajectoryEvaluator：对 1D 候选进行打分与排序
- TrajectoryCombiner：把 lon/lat pair 组合成离散 Cartesian 轨迹（复用 `general::FrenetFrame`）
- 约束检查：复用 `general::TrajectoryManager` 提供的 `isTrajectoryValid`（速度、加速度、曲率、jerk）

- latticePlanner：继承自 `PlannerBase` 的 orchestrator，负责管道执行（生成→评估→组合→检验→返回）

📦 构建与测试

在仓库根目录下执行：

```bash
colcon build --packages-select lattice
```

可运行的测试二进制（安装后在 `install/lattice/lib/lattice` 下）：
- `test_trajectory1d_generator` 生成候选测试
- `test_trajectory_evaluator` 候选评估测试
- `test_trajectory_combiner` 组合测试
- `test_constraint_collision` 约束/碰撞测试
- `test_lattice_planner` 简单的 planner 流程测试
- `test_following_scenario` 跟车场景测试
- `test_collision_edge` 碰撞边界测试

例如运行：

```bash
/home/<user>/.../install/lattice/lib/lattice/test_lattice_planner
```

⚙️ 参数说明（常用）

在创建 `latticePlanner` 或 `Trajectory1DGenerator` 时通过参数 map 传入：
- `sample_min_time`, `sample_max_time`, `sample_time_step`：纵向时间采样范围与步长
- `sample_lat_width`, `sample_width_length`：横向偏移采样范围与步长
- `weight_st_object`, `weight_st_jerk`, `weight_lt_offset`, `weight_lt_acc`：代价权重
- `weight_st_acc`：纵向加速度惩罚权重
- `max_speed`, `max_acc`, `max_curvature`, `max_jerk`：约束限值

🧭 使用建议

- 首先通过 `setGlobalReferenceLine` 设置参考线（`std::vector<general::PathPoint>`）。
- 调用 `plan(ego_state, obstacles, reference_speed, current_time)` 返回 `std::vector<general::TrajectoryPoint>`。
- 可以通过 `isTrajectoryValid` 再次检查轨迹合法性。

🔧 扩展与改进点

- 横向优化器（OSQP 或 piecewise 曲线）替代当前简单采样
- 更复杂的 leader 识别和 path-time 障碍处理
- 将测试集成进 CI（例如在 GitHub Actions 中运行 colcon build + 执行二进制）

📄 代码组织（主要文件）

- include/lattice/*.h
- src/*.cpp
- test/*.cpp

感谢使用，如果你希望我把更详细的 PR 描述与变更分块成多个 commit，我可以把当前改动打包并生成 PR 草案文本。