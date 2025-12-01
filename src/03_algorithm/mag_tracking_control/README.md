# mag_tracking_control

传感器阵列控制策略包，用于实现传感器位姿的自适应控制以优化磁铁跟踪效果。

## 功能特性

- **策略模式设计**：支持多种控制算法，可通过配置轻松切换
- **当前支持的策略**：
  - `fixed_offset`：固定偏移策略（最简单）
  - `adaptive_distance`：自适应距离策略（基于磁场强度）
  - 预留接口，方便后续添加 Fisher 信息矩阵等高级算法
- **连续轨迹模式**：支持平滑连续的机械臂运动，避免分段运动的不连续性
  - 使用 MoveIt 的笛卡尔路径规划 (`computeCartesianPath`)
  - 轨迹缓冲机制，累积多个目标点后执行连续轨迹
  - 异步轨迹执行，不阻塞控制循环

## 架构设计

### 策略模式

采用策略模式设计，便于扩展和切换算法：

```
TrackingControlStrategyBase (基类)
├── FixedOffsetStrategy (固定偏移)
├── AdaptiveDistanceStrategy (自适应距离)
└── FisherOptimalStrategy (预留，未来实现)
```

### 文件结构

```
include/mag_tracking_control/
├── tracking_control_strategy_base.h    # 策略基类接口
├── strategy_fixed_offset.h             # 固定偏移策略
├── strategy_adaptive_distance.h        # 自适应距离策略
└── tracking_control_node.h             # ROS 节点主类

src/
├── strategy_fixed_offset.cpp
├── strategy_adaptive_distance.cpp
├── tracking_control_node.cpp
└── tracking_control_config_loader.cpp

config/
└── tracking_control.yaml               # 配置文件模板

launch/
└── tracking_control.launch             # 启动文件
```

## 使用说明

### 基本使用

```bash
# 使用默认配置（固定偏移策略，仅规划不执行）
roslaunch mag_tracking_control tracking_control.launch

# 启用实际执行
roslaunch mag_tracking_control tracking_control.launch enable_execution:=true

# 使用自定义配置文件
roslaunch mag_tracking_control tracking_control.launch config_file:=/path/to/config.yaml
```

### 切换策略

在配置文件中修改 `strategy/type` 字段：

```yaml
config:
  strategy:
    type: fixed_offset  # 或 "adaptive_distance"
```

### 配置说明

参见 `config/tracking_control.yaml` 中的详细注释。

主要配置项：
- `strategy/type`：策略类型
- `strategy/fixed_offset/*`：固定偏移策略参数
- `strategy/adaptive_distance/*`：自适应距离策略参数
- `control/enable_execution`：是否实际执行运动
- `control/update_rate`：控制循环频率
- `control/use_continuous_trajectory`：是否启用连续轨迹模式（推荐用于平滑跟踪）
- `control/trajectory_buffer_size`：轨迹缓冲大小（累积多少个点后执行）
- `control/cartesian_path_step_size`：笛卡尔路径步长（越小越平滑）

## 接口说明

### 订阅话题

- `/magnetic/pose_estimated` (`mag_core_msgs/MagnetPose`)：磁铁位姿估计结果

### 发布话题

- `tracking_control/target_pose` (`geometry_msgs/PoseStamped`)：目标传感器位姿（用于可视化）

### 服务调用

- `/mag_device_arm/set_end_effector_pose` (`mag_device_arm/SetEndEffectorPose`)：调用机械臂服务执行运动

### TF 使用

- 查询 `world -> sensor_array` 获取当前传感器位姿
- 查询 `world -> magnet` 获取磁铁位姿（备用）

## 扩展新策略

要添加新的控制策略，需要：

1. **继承策略基类**：
```cpp
class MyNewStrategy : public TrackingControlStrategyBase {
    bool initialize() override { ... }
    bool computeTargetPose(const TrackingControlInput &input,
                          TrackingControlOutput &output) override { ... }
    std::string name() const override { return "my_new_strategy"; }
};
```

2. **在节点中注册策略**：
在 `TrackingControlNode::createStrategy()` 中添加新的策略创建逻辑。

3. **更新配置文件**：
在配置文件中添加新策略的参数。

## 连续轨迹模式

### 为什么需要连续轨迹模式？

传统的控制方式每次都会独立规划并执行一条路径，导致机械臂运动不连续、分段明显。连续轨迹模式通过以下方式实现平滑运动：

1. **轨迹缓冲**：控制循环将目标位姿添加到缓冲队列，而不是立即执行
2. **批量规划**：当缓冲中累积足够多的点（默认5个）时，使用 MoveIt 的 `computeCartesianPath` 将这些点组合成一条连续的笛卡尔路径
3. **异步执行**：轨迹执行在独立线程中进行，不阻塞控制循环

### 启用连续轨迹模式

在配置文件中设置：

```yaml
config:
  control:
    use_continuous_trajectory: true  # 启用连续轨迹模式
    trajectory_buffer_size: 5         # 累积5个点后执行
    cartesian_path_step_size: 0.01   # 笛卡尔路径步长（米）
    move_group_name: fr5v6_arm       # MoveIt 规划组名称
    end_effector_link: bracket_tcp_link  # 末端执行器链接
    reference_frame: world           # 参考坐标系
```

### 工作原理

1. **控制循环**（20Hz）：计算目标位姿并添加到缓冲队列
2. **轨迹执行线程**（10Hz）：检查缓冲队列，当有足够点时：
   - 取出缓冲中的点
   - 使用 `computeCartesianPath` 规划连续路径
   - 执行轨迹
3. **平滑效果**：多个点被组合成一条连续轨迹，机械臂运动更加平滑

### 参数调优

- **`trajectory_buffer_size`**：较大的值会产生更长的连续轨迹，但会增加延迟
- **`cartesian_path_step_size`**：较小的值会产生更平滑的轨迹，但计算时间更长
- **`update_rate`**：控制循环频率，影响目标位姿的更新速度

## 依赖

- `mag_core_msgs`：消息定义
- `mag_device_arm`：机械臂控制接口（用于非连续模式）
- `moveit_core`, `moveit_ros_planning_interface`：MoveIt 规划接口（用于连续轨迹模式）
- `tf2`, `tf2_ros`：坐标变换
- `Eigen3`：矩阵运算

## 未来扩展

计划添加的策略：
- Fisher 信息矩阵最优观测位置策略
- 基于可观测性的策略
- 预测性跟踪策略
- 多目标优化策略（FIM + 运动性）
