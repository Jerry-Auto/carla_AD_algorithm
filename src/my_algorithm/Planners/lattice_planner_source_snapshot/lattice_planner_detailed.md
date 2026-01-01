# LatticePlanner 详解（采样 → 评估 → 组合 → 筛选） ✅

本文档对 Apollo `LatticePlanner` 的实现做了详尽说明，内容涵盖：轨迹采样（纵/横）、1D 预评估、轨迹组合（Frenet→Cartesian 的数学推导）、组合后联合评估（动力学约束与碰撞检测）、以及最终选轨策略。文中包含源码位置引用与关键代码片段，便于对照阅读。📚

---

## 目录
1. 概览（Pipeline 总览）
2. 轨迹采样（longitudinal s(t) / lateral d(s)）
3. 1D 轨迹预评估与排序（`TrajectoryEvaluator`）
4. 轨迹组合（`TrajectoryCombiner::Combine`）与 Frenet→Cartesian 推导
5. 组合后联合评估（`ConstraintChecker` 与 碰撞检测（`latticeCollisionDetection::InCollision`））
6. 轨迹选择逻辑（最终采纳）
7. 关键源码文件清单（便于快速定位）
8. 附：重点数学公式与代码片段

---

## 1. 概览（Pipeline 总览） 🔍
- **输入**：参考线（`ReferenceLine`）、车辆当前状态（`TrajectoryPoint`）、障碍物与预测（`PredictionQuerier` / `PathTimeGraph`）。
- **输出**：最终采纳的离散化轨迹（`DiscretizedTrajectory`），写入 `ReferenceLineInfo`。
- **主要阶段**：
  1. 参考线离散化（`ToDiscretizedReferenceLine`）
  2. 生成纵向 & 横向 1D 候选（`Trajectory1dGenerator`）
  3. 1D 候选的预评估与按代价排序（`TrajectoryEvaluator`）
  4. 逐对取出最优 1D 配对并 **组合成 2D 轨迹**（`TrajectoryCombiner::Combine`）
  5. 对组合后轨迹做联合约束检查（`ConstraintChecker::ValidTrajectory`）与碰撞检测（`latticeCollisionDetection::InCollision`）
  6. 选取首个通过所有检查的轨迹；若均不通过则使用备份轨迹（`BackupTrajectoryGenerator`）或报错。

---

## 2. 轨迹采样（详细） 🎯

### 2.1 纵向（Longitudinal）s(t)
- **自变量**：时间 t。
- **目标**：生成 s(t), ṡ(t), s̈(t)，处理速度目标、停车点、path-time 障碍物等时间依赖约束。
- **生成类**：`Trajectory1dGenerator`（函数 `GenerateSpeedProfilesForCruising/Stopping/PathTimeObstacles`）。
- **多项式**：
  - 巡航使用 Quartic（4阶） → `GenerateTrajectory1DBundle<4>`（无需终点位置）。
  - 停车或与 path-time 点相关的情形使用 Quintic（5阶） → `GenerateTrajectory1DBundle<5>`（终点位置/速度/加速度均可指定）。
- **终点采样**：由 `EndConditionSampler` 提供（如 `SampleLonEndConditionsForCruising`、`SampleLonEndConditionsForStopping`、`SampleLonEndConditionsForPathTimePoints`）。

### 2.2 横向（Lateral）d(s)
- **自变量**：纵向弧长 s（注意：横向曲线是 `d(s)`，不是 `d(t)`）。
- **目标**：生成 d(s), d'(s), d''(s)（分别为 d, dd/ds, d^2d/ds^2）。
- **生成类**：`Trajectory1dGenerator::GenerateLateralTrajectoryBundle()`。
- **默认采样（非优化）**：`EndConditionSampler::SampleLatEndConditions()` 返回组合：
  - `end_s_candidates = {10.0, 20.0, 40.0, 80.0}`（相对 s）
  - `end_d_candidates = {0.0, -0.5, 0.5}`（横偏）
  - 对每对 (s,d) 用 `QuinticPolynomialCurve1d` 生成 d(s)；`param = s`。
- **优化模式**（`FLAGS_lateral_optimization=true`）：用 `LateralOSQPOptimizer` 在 `[s_min, s_max]` 上求解 Piecewise 曲线（`PiecewiseJerkTrajectory1d`）。

> 注：横向 `param` 表示曲线定义到的 s 长度（相对当前 s0）。组合时以纵向的 `relative_s = s(t)-s0` 作为横向自变量。

---

## 3. 1D 预评估与排序（TrajectoryEvaluator） 🧾

实现：`modules/planning/planners/lattice/trajectory_generation/trajectory_evaluator.cc`

### 3.1 构造器时的直接剪枝（早期丢弃）
- **停点过滤**：若 planning_target 有 stop_point，丢弃会越过 stop_point 的纵向候选（带 buffer）。
- **纵向 1D 约束检查**：`ConstraintChecker1d::IsValidLongitudinalTrajectory`，在时间采样点上检查 v/a/jerk 是否在上下界范围内，若不满足则丢弃。
- **（横向 1D 验证 `IsValidLateralTrajectory` 在源码中被注释掉，通常横向约束在组合后检查。）**

### 3.2 代价组成（Evaluate）→ 用于排序
- **LonObjectiveCost**：目标速度偏差 + 距离（是否达目标）
- **LonComfortCost**：纵向 jerk 舒适性
- **LonCollisionCost**：基于 `PathTimeGraph` 的 path-time 间隔对纵向的接近程度做代价
- **CentripetalAccelerationCost**：v^2 * κ 对曲线的离心代价
- **LatOffsetCost**：序列 s 上横偏的平方/绝对代价
- **LatComfortCost**：max |l'' * ṡ^2 + l' * s̈|（横向舒适）

这些代价按权重合成最终 `cost`，并将 `(lon, lat)` 对放入优先队列 `cost_queue_`（按 cost 从小到大）。

---

## 4. 轨迹组合（Combine）与 Frenet→Cartesian（数学推导） 📐

实现：`TrajectoryCombiner::Combine`（`trajectory_combiner.cc`）调用 `CartesianFrenetConverter::frenet_to_cartesian`（`cartesian_frenet_conversion.cc`）。

### 4.1 组合流程（伪代码）
```cpp
s0 = lon.Evaluate(0, 0)
for t in [0, T] step dt:
  s = lon.Evaluate(0, t)
  s_dot = lon.Evaluate(1, t)
  s_ddot = lon.Evaluate(2, t)
  relative_s = s - s0
  d = lat.Evaluate(0, relative_s)
  d' = lat.Evaluate(1, relative_s)
  d''= lat.Evaluate(2, relative_s)
  matched_ref_point = PathMatcher::MatchToPath(reference_line, s)
  CartesianFrenetConverter::frenet_to_cartesian(...)
  append TrajectoryPoint(x,y,theta,kappa,v,a,t)
```

### 4.2 关键公式（来源：`frenet_to_cartesian`）
- **位置**：
  x = x_r − d * sin(θ_r)
  y = y_r + d * cos(θ_r)
- **航向角**：
  Δθ = atan2(d', 1 − κ_r d)
  θ = θ_r + Δθ
- **曲率 κ**（代码表达，来源微分几何推导）：
  kappa_r_d_prime = rdkappa * d + rkappa * d'  
  κ = (((d'' + kappa_r_d_prime * tanΔθ) * cos^2Δθ) / (1 − κ_r d) + κ_r) * cosΔθ / (1 − κ_r d)
- **速度**：
  ḋ = d' * ṡ
  v = sqrt( (1 − κ_r d)^2 * ṡ^2 + ḋ^2 )
- **加速度（合成）**：
  见源码表达式，包含 s̈ 投影项与 ṡ^2 的耦合项。

### 4.3 超出横向 param 的处理（外推）
- 若 `relative_s` > `lat.ParamLength()`，`LatticeTrajectory1d::Evaluate` 会做**常加速度外推**：
  p(t) = p_last + v_last * Δt + 0.5 * a_last * Δt^2 等。
- 这避免崩溃但在大外推情况下精度/物理合理性会下降。

---

## 5. 组合后联合评估（ConstraintChecker、latticeCollisionDetection::InCollision）🔧

### 5.1 联合约束（`ConstraintChecker::ValidTrajectory`）
- **检查点**（对每个 time point，t ≤ FLAGS_trajectory_time_length）：
  - lon v ∈ [speed_lower_bound, speed_upper_bound]
  - lon a ∈ [long_acc_lower_bound, long_acc_upper_bound]
  - curvature κ ∈ [−FLAGS_kappa_bound, FLAGS_kappa_bound]
- **相邻点检查**：
  - longitudinal jerk = Δa / Δt ∈ [jerk_lower_bound, jerk_upper_bound]
  - lateral acceleration ~ v^2 * κ ∈ [−FLAGS_lateral_acc_bound, FLAGS_lateral_acc_bound]
- 违反则返回对应枚举（例如 LON_VELOCITY_OUT_OF_BOUND）。

### 5.2 碰撞检测（`latticeCollisionDetection::InCollision`）
- 预构建每个时间步的障碍 bounding box（拓展 buffer）数组 `predicted_bounding_rectangles_`。
- 对每个轨迹时刻构建 ego box 并做几何重叠检测；若重叠则判为碰撞。

---

## 6. 最终轨迹选择逻辑 🏁
- 在 `PlanOnReferenceLine` 中，按代价循环：每次取 `trajectory_evaluator.next_top_trajectory_pair()` → 组合为 2D → 运行 `ConstraintChecker` 与 `latticeCollisionDetection::InCollision`。
- **首个通过两项检查的轨迹即被选中**（写入 `ReferenceLineInfo` 并返回成功），不用遍历所有组合（早停）。
- 若没有可行轨迹且启用 `FLAGS_enable_backup_trajectory`，使用 `BackupTrajectoryGenerator` 生成保底轨迹。

---

## 7. 关键源码文件清单（便于定位） 🗂️
- `modules/planning/planners/lattice/lattice_planner.cc, .h`
- `modules/planning/planners/lattice/trajectory_generation/trajectory1d_generator.{h,cc}`
- `modules/planning/planners/lattice/trajectory_generation/end_condition_sampler.{h,cc}`
- `modules/planning/planners/lattice/trajectory_generation/trajectory_evaluator.{h,cc}`
- `modules/planning/planners/lattice/trajectory_generation/trajectory_combiner.{h,cc}`
- `modules/planning/planners/lattice/trajectory_generation/lattice_trajectory1d.{cc}`
- `modules/planning/planning_base/math/constraint_checker/constraint_checker.{cc,h}`
- `modules/planning/planning_base/math/constraint_checker/constraint_checker1d.{cc,h}`
- `modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.{cc,h}`
- `modules/common/math/cartesian_frenet_conversion.{cc,h}`
- `modules/planning/planners/lattice/behavior/collision_checker.{cc,h}`

> 我已经在仓库里生成了源码快照：`docs/lattice_planner_source_snapshot/` （包含上面列出的关键文件），便于审阅或打包。

---

## 8. 附：代码片段 & 常用查找点
- 在 `lattice_planner.cc` 中，你会看到从生成 1D bundle 到 `TrajectoryEvaluator` 再到 `TrajectoryCombiner::Combine` 的调用链。关键循环在 `PlanOnReferenceLine()` 的 `while (trajectory_evaluator.has_more_trajectory_pairs())`。
- `CartesianFrenetConverter::frenet_to_cartesian` 是坐标/速度/加速度合成的数学核心（已在本文第 4 节给出公式并贴入源码实现）。

---

## 结语 ✅
如果你希望，我可以：
- 把这份文档提交到仓库（我已创建 `docs/lattice_planner_detailed.md`）；
- 或根据你关心的场景（如变道、跟车）做针对性示例和数值验证；
- 或将 snapshot 打包成 `tar.gz` 供离线传阅。

需要我为你执行哪个操作？（提交 PR / 打包 / 做示例），回复即可，我继续执行。🔧✨
