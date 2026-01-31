# 动态规划（DP）路径规划器模板使用说明

## 目录
- [1. 概述](#1-概述)
- [2. 快速开始](#2-快速开始)
- [3. 详细使用步骤](#3-详细使用步骤)
  - [3.1 定义问题特定的状态类型](#31-定义问题特定的状态类型)
  - [3.2 实现四个核心策略类](#32-实现四个核心策略类)
  - [3.3 配置规划器参数](#33-配置规划器参数)
- [4. 规划模式详解](#4-规划模式详解)
- [5. 完整示例：2D机器人路径规划](#5-完整示例2d机器人路径规划)
- [6. 高级主题](#6-高级主题)
- [7. 常见问题与解决方案](#7-常见问题与解决方案)
- [8. 附录](#8-附录)

## 1. 概述

这是一个通用的动态规划（DP）路径规划器模板，采用策略模式设计，可以灵活地应用于：
- 机器人路径规划（SL平面）
- 车辆速度规划（ST平面）
- 机械臂轨迹规划
- 无人机路径规划
- 以及其他需要序列决策的问题

**核心特点**：
- 🎯 模板化设计，支持任意状态类型
- 🧩 策略模式，易于扩展和替换
- ⚡ 支持剪枝和启发式，平衡最优性与效率
- 📊 模块化约束检查，确保结果可行性
- 🔧 可配置参数，适应不同计算资源
- 🚀 两种规划模式：动态采样和预设网格

**最新优化**：
- `plan` 方法现使用 `generateSamplingGrid` 一次性生成采样网格，提高效率
- 简化代码结构，减少重复逻辑

## 1. 概述

这是一个通用的动态规划（DP）路径规划器模板，采用策略模式设计，可以灵活地应用于：
- 机器人路径规划（SL平面）
- 车辆速度规划（ST平面）
- 机械臂轨迹规划
- 无人机路径规划
- 以及其他需要序列决策的问题

**核心特点**：
- 🎯 模板化设计，支持任意状态类型
- 🧩 策略模式，易于扩展和替换
- ⚡ 支持剪枝和启发式，平衡最优性与效率
- 📊 模块化约束检查，确保结果可行性
- 🔧 可配置参数，适应不同计算资源

## 2. 快速开始

### 2.1 安装依赖
```bash
# 必需依赖
sudo apt-get install libeigen3-dev
# ROS2依赖（如果用于ROS2项目）
sudo apt-get install ros-${ROS_DISTRO}-rclcpp
```

### 2.2 基本使用流程
```cpp
// 1. 定义状态类型
struct MyState { /* 状态变量 */ };

// 2. 创建策略实例
auto cost_func = std::make_shared<MyCostFunction>();
auto constraints = std::make_shared<MyConstraintChecker>();
auto sampling = std::make_shared<MySamplingStrategy>();
auto backtrack = std::make_shared<MyBacktrackStrategy>();

// 3. 配置规划器
planning::DpPlannerConfig config;
config.max_layers = 10;
config.enable_pruning = true;

// 4. 创建并运行规划器
planning::DpPlanner<MyState> planner(
    cost_func, constraints, sampling, backtrack, config
);

// 5. 执行规划（两种模式可选）
auto result = planner.plan(start_state, 10, goal_state);  // 动态采样模式
// 或使用预设网格
auto grid = sampling->generateSamplingGrid(start_state, 10);
auto result = planner.planWithGrid(grid, start_state, goal_state);
```

## 3. 详细使用步骤

### 3.1 定义问题特定的状态类型

#### **必须自定义的部分**：

**1. 状态变量成员（核心必选项）**
```cpp
struct YourState {
    // 根据具体问题定义状态变量
    double x;          // 必须：位置坐标X
    double y;          // 必须：位置坐标Y
    double theta;      // 可选但推荐：朝向角度
    
    // 动力学相关（可选）
    double velocity;   // 速度
    double curvature;  // 曲率
    
    // 时间相关（可选）
    double time;       // 时间戳
    
    // 构造函数（推荐）
    YourState(double x_ = 0, double y_ = 0, double theta_ = 0)
        : x(x_), y(y_), theta(theta_) {}
};
```

**2. 比较操作符（条件必选）**
如果需要在容器中查找或去重：
```cpp
bool operator==(const YourState& other) const {
    return std::abs(x - other.x) < 1e-6 &&
           std::abs(y - other.y) < 1e-6;
}
```

**3. 哈希函数（条件必选）**
如果状态要用于 `unordered_map`：
```cpp
struct Hash {
    size_t operator()(const YourState& s) const {
        size_t h1 = std::hash<double>{}(s.x);
        size_t h2 = std::hash<double>{}(s.y);
        return h1 ^ (h2 << 1);
    }
};
```

### 3.2 实现四个核心策略类

#### **策略1：代价函数策略（`CostFunctionStrategy`）**

**必须重写**：
```cpp
class MyCostFunction : public planning::CostFunctionStrategy<YourState> {
public:
    // 1. 转移代价计算（纯虚函数，必须实现）
    double calculateTransitionCost(
        const YourState& from, 
        const YourState& to
    ) const override {
        // 实现具体的代价计算逻辑
        double distance_cost = /* 距离代价 */;
        double smooth_cost = /* 平滑性代价 */;
        return distance_cost + smooth_cost;
    }
    
    // 2. 启发式代价（可选重写）
    double calculateHeuristicCost(
        const YourState& state,
        const YourState& goal = YourState{}
    ) const override {
        // 默认返回0，启用启发式时需要重写
        if (启用启发式) {
            return (state.x - goal.x) * (state.x - goal.x) +
                   (state.y - goal.y) * (state.y - goal.y);
        }
        return 0.0;
    }
};
```

**必须自定义的参数**：
```cpp
private:
    // 代价权重（根据问题调整）
    double weight_distance = 1.0;
    double weight_smoothness = 0.5;
    double weight_obstacle = 10.0;
    
    // 参考信息（如需要）
    std::vector<Point> reference_path_;
```

#### **策略2：约束检查策略（`ConstraintCheckerStrategy`）**

**必须重写**：
```cpp
class MyConstraintChecker : public planning::ConstraintCheckerStrategy<YourState> {
public:
    // 1. 状态约束检查（纯虚函数，必须实现）
    bool checkState(const YourState& state) const override {
        // 检查边界、障碍物等
        if (state.x < 0 || state.x > map_width) return false;
        if (碰撞检测(state)) return false;
        return true;
    }
    
    // 2. 转移约束检查（纯虚函数，必须实现）
    bool checkTransition(const YourState& from, const YourState& to) const override {
        // 检查运动学约束
        if (曲率过大(from, to)) return false;
        if (速度过大(from, to)) return false;
        return true;
    }
};
```

**必须自定义的参数**：
```cpp
private:
    // 环境约束
    double map_width_, map_height_;
    std::vector<Obstacle> obstacles_;
    
    // 运动学约束
    double max_velocity_, max_acceleration_;
    double robot_radius_;
```

#### **策略3：采样策略（`SamplingStrategy`）**

**必须重写**：
```cpp
class MySamplingStrategy : public planning::SamplingStrategy<YourState> {
public:
    // 生成下一层采样（纯虚函数，必须实现）
    std::vector<YourState> generateNextLayer(
        const std::vector<YourState>& current_layer,
        int layer_index
    ) const override {
        std::vector<YourState> next_states;
        
        for (const auto& current : current_layer) {
            // 生成多个可能的下一状态
            for (int i = 0; i < num_samples; ++i) {
                YourState next = /* 根据当前状态生成下一状态 */;
                next_states.push_back(next);
            }
        }
        
        return next_states;
    }
};
```

**必须自定义的参数**：
```cpp
private:
    // 采样参数
    double step_size_ = 2.0;      // 步长
    int angle_samples_ = 8;       // 角度采样数
    bool adaptive_sampling_ = true; // 是否自适应
```

#### **策略4：回溯策略（`BacktrackStrategy`）**

**必须重写**：
```cpp
class MyBacktrackStrategy : public planning::BacktrackStrategy<YourState> {
public:
    // 1. 回溯路径（纯虚函数，必须实现）
    std::vector<YourState> backtrack(
        const planning::DpNode<YourState>& optimal_node
    ) const override {
        // 标准回溯实现
        std::vector<YourState> path;
        const auto* current = &optimal_node;
        
        while (current != nullptr) {
            path.insert(path.begin(), current->state);
            current = current->parent.get();
        }
        
        return path;
    }
    
    // 2. 终止状态判断（纯虚函数，必须实现）
    bool isTerminalState(
        const YourState& state,
        int layer_index
    ) const override {
        // 判断是否为终止状态
        return (layer_index >= max_layers_) ||
               (到达目标区域(state));
    }
};
```

### 3.3 配置规划器参数

#### **`DpPlannerConfig` 结构体参数说明**

| 参数 | 必须调整 | 说明 | 示例值 |
|------|----------|------|--------|
| `max_layers` | **是** | 最大规划层数 | 10-20 |
| `enable_pruning` | **是** | 是否启用剪枝 | true |
| `pruning_keep_num` | **是** | 每层保留节点数 | 5-15 |
| `enable_heuristic` | 可选 | 是否启用启发式 | true |
| `time_limit_ms` | 建议 | 计算时间限制 | 50.0 |
| `max_nodes_per_layer` | 建议 | 每层最大节点数 | 100 |
| `debug_mode` | 可选 | 调试模式 | false |

#### **配置示例**

```cpp
planning::DpPlannerConfig config;

// 简单问题配置
config.max_layers = 10;
config.enable_pruning = true;
config.pruning_keep_num = 5;

// 复杂问题配置
config.max_layers = 20;
config.enable_pruning = true;
config.pruning_keep_num = 15;
config.enable_heuristic = true;
config.time_limit_ms = 100.0;

// 实时规划配置
config.max_layers = 8;
config.pruning_keep_num = 3;
config.time_limit_ms = 20.0;
```

## 4. 规划模式详解

DP 规划器支持两种规划模式，适用于不同场景：

### 4.1 动态采样模式（`plan` 方法）
- **适用场景**：采样策略复杂、需要动态调整采样
- **工作流程**：
  1. 调用 `generateSamplingGrid` 一次性生成完整采样网格
  2. 使用 `forwardSearch` 执行规划
- **优势**：灵活，采样可自适应
- **示例**：
  ```cpp
  auto result = planner.plan(start_state, num_layers, goal_state);
  ```

### 4.2 预设网格模式（`planWithGrid` 方法）
- **适用场景**：采样网格可预计算、离线规划
- **工作流程**：
  1. 外部提供采样网格
  2. 直接使用网格进行规划
- **优势**：高效，网格可重用
- **示例**：
  ```cpp
  auto grid = sampling->generateSamplingGrid(start_state, num_layers);
  auto result = planner.planWithGrid(grid, start_state, goal_state);
  ```

### 4.3 模式选择建议
- **选择动态采样模式**：当采样依赖当前状态或需要实时调整时
- **选择预设网格模式**：当网格固定、可预计算时（如测试、离线规划）
- **性能对比**：预设网格模式通常更快，因为避免重复采样

## 5. 完整示例：2D机器人路径规划

```cpp
// ==================== 1. 定义状态类型 ====================
struct RobotState {
    double x, y, theta;  // 位置和朝向
    double v;            // 速度
    
    RobotState(double x_=0, double y_=0, double t_=0, double v_=0)
        : x(x_), y(y_), theta(t_), v(v_) {}
};

// ==================== 2. 实现策略类 ====================
class RobotCostFunction : public planning::CostFunctionStrategy<RobotState> {
    double calculateTransitionCost(const RobotState& from, const RobotState& to) const override {
        double dist = sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
        double angle_diff = /* 角度差 */;
        return dist + 0.5 * angle_diff;
    }
};

class RobotConstraintChecker : public planning::ConstraintCheckerStrategy<RobotState> {
    bool checkState(const RobotState& state) const override {
        return state.x >= 0 && state.x <= 100 &&  // 地图边界
               state.y >= 0 && state.y <= 100 &&
               !checkCollision(state);            // 障碍物检查
    }
    
    bool checkTransition(const RobotState& from, const RobotState& to) const override {
        double dist = sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2));
        return dist <= 3.0;  // 最大步长限制
    }
};

// ==================== 3. 创建规划器 ====================
int main() {
    // 创建策略实例
    auto cost_func = std::make_shared<RobotCostFunction>();
    auto constraints = std::make_shared<RobotConstraintChecker>();
    auto sampling = std::make_shared<RobotSamplingStrategy>();
    auto backtrack = std::make_shared<planning::DefaultBacktrackStrategy<RobotState>>();
    
    // 配置规划器
    planning::DpPlannerConfig config;
    config.max_layers = 15;
    config.enable_pruning = true;
    config.pruning_keep_num = 8;
    
    // 创建规划器
    planning::DpPlanner<RobotState> planner(
        cost_func, constraints, sampling, backtrack, config
    );
    
    // 执行规划
    RobotState start(0, 0, 0, 0);
    RobotState goal(50, 50, 0, 0);
    auto result = planner.plan(start, 15, goal);
    
    // 处理结果
    if (result.success) {
        std::cout << "规划成功！路径点数: " << result.optimal_path.size() << std::endl;
    }
    
    return 0;
}
```

## 6. 高级主题

### 5.1 自适应采样策略
```cpp
class AdaptiveSamplingStrategy : public planning::SamplingStrategy<YourState> {
    std::vector<YourState> generateNextLayer(...) const override {
        // 根据当前层数调整采样密度
        int num_samples = 5 + layer_index / 2;  // 随层数增加
        
        // 在障碍物附近增加采样密度
        if (nearObstacle(current_state)) {
            num_samples *= 2;
        }
        
        // 生成采样
        // ...
    }
};
```

### 5.2 多目标优化
```cpp
class MultiObjectiveCost : public planning::CostFunctionStrategy<YourState> {
    double calculateTransitionCost(...) const override {
        // 计算多个目标
        double safety_cost = calculateSafety(from, to);
        double comfort_cost = calculateComfort(from, to);
        double efficiency_cost = calculateEfficiency(from, to);
        
        // 加权求和
        return w1 * safety_cost + w2 * comfort_cost + w3 * efficiency_cost;
    }
};
```

### 5.3 增量式规划
```cpp
class IncrementalPlanner {
    planning::DpPlanner<YourState> planner_;
    std::vector<std::vector<YourState>> previous_grid_;
    
    PlanningResult replan(const YourState& new_start) {
        // 重用之前的采样网格
        auto new_grid = updateGrid(previous_grid_, new_start);
        return planner_.planWithGrid(new_grid, new_start);
    }
};
```

## 7. 常见问题与解决方案

### Q1: 规划器找不到可行路径
**解决方案**：
1. 检查约束条件是否过严：放宽 `checkState` 和 `checkTransition` 中的限制
2. 增加采样点：在 `SamplingStrategy` 中增加采样数量和范围
3. 增加规划层数：调整 `config.max_layers`
4. 检查代价函数：确保没有不合理的惩罚

### Q2: 计算时间过长
**解决方案**：
1. 启用剪枝：设置 `config.enable_pruning = true`
2. 减少保留节点：减小 `config.pruning_keep_num`
3. 简化采样：减少 `SamplingStrategy` 中的采样点数
4. 启用启发式：设置 `config.enable_heuristic = true`

### Q3: 路径不平滑或有突变
**解决方案**：
1. 在状态中添加导数信息：如速度、加速度
2. 在代价函数中增加平滑性惩罚
3. 在约束中限制最大曲率或转向角
4. 使用后处理平滑路径

### Q4: 状态空间爆炸
**解决方案**：
1. 使用强剪枝：设置小的 `pruning_keep_num`
2. 分层规划：先粗规划，再局部精细规划
3. 降维：减少状态维度，如忽略不重要的变量
4. 使用启发式引导搜索

### Q6: 如何处理多目标优化？
**解决方案**：
1. 在代价函数中定义多个目标：安全、舒适、效率
2. 使用加权求和：`total_cost = w1*safety + w2*comfort + w3*efficiency`
3. 根据应用场景调整权重
4. 考虑 Pareto 最优解（高级）

### Q7: 规划器如何扩展到 3D 空间？
**解决方案**：
1. 在状态中添加 Z 坐标：`double z;`
2. 更新代价函数：包含 3D 距离计算
3. 更新约束：3D 障碍物检测
4. 调整采样策略：3D 空间采样

---

## 8. 附录

### 8.1 API 参考

#### `DpPlanner` 类主要方法
- `plan(start, layers, goal)`: 动态采样规划
- `planWithGrid(grid, start, goal)`: 预设网格规划
- `getConfig()`: 获取当前配置
- `updateConfig(config)`: 更新配置
- `reset()`: 重置规划器状态

#### `PlanningResult` 结构体
- `optimal_path`: 最优路径（状态序列）
- `total_cost`: 总代价
- `success`: 是否成功
- `message`: 结果信息
- `num_nodes_expanded`: 扩展节点数
- `computation_time_ms`: 计算时间
- `search_tree`: 搜索树（调试用）

### 8.2 性能调优指南

#### 实时规划优化
```cpp
// 低延迟配置
config.max_layers = 8;
config.pruning_keep_num = 3;
config.time_limit_ms = 20.0;
config.enable_heuristic = true;
```

#### 高精度规划优化
```cpp
// 高质量配置
config.max_layers = 25;
config.pruning_keep_num = 20;
config.enable_pruning = true;
config.enable_heuristic = true;
```

#### 内存优化
- 减少 `max_nodes_per_layer`
- 禁用 `store_full_tree`
- 使用智能指针管理内存

### 8.3 调试技巧

#### 启用调试输出
```cpp
config.debug_mode = true;
config.store_full_tree = true;

// 在策略类中添加日志
RCLCPP_INFO(logger, "扩展节点数: %d", nodes.size());
```

#### 可视化搜索过程
```cpp
// 保存搜索树用于可视化
if (config.store_full_tree) {
    visualizeSearchTree(result.search_tree);
}
```

---

## 技术支持与贡献

如果遇到问题或有改进建议：
1. 检查文档和示例代码
2. 查看代码中的注释说明
3. 调试各个策略类的实现
4. 调整配置参数进行实验

欢迎贡献代码和改进建议！