# mag_single_tracking_app

单臂磁传感器阵列跟踪的仿真应用编排包。传感器阵列刚性连接到机械臂末端工具（`bracket_tcp_link`），通过静态 TF 发布传感器阵列位姿。本包将现有功能包组合到一个可直接运行的仿真流程中，用于单臂跟踪场景的算法开发和测试。

## 功能概述

与可移动式传感器阵列（`mag_sensor_movable_app`）不同，本包针对传感器阵列固定在机械臂末端的场景。主要特点：

- **传感器阵列刚性连接**：传感器阵列通过静态 TF 固定在机械臂末端工具（`bracket_tcp_link`）
- **支持位置偏移配置**：传感器阵列相对于工具末端的位置和姿态可通过配置文件设置
- **完整仿真链路**：集成机械臂控制、磁体运动、传感器仿真、姿态估计和可视化
- **仅支持仿真**：本包仅用于仿真场景，不包含硬件驱动

### 仿真场景

完整的仿真链路，用于算法开发和测试：

- **mag_device_arm**: 负责机械臂控制和 TF 变换
- **mag_device_sensor** (仿真模式): 根据 TF 生成 `mag_core_msgs/MagSensorData` 消息，并发布传感器阵列的静态 TF（从 `bracket_tcp_link` 到 `sensor_array`）
- **mag_device_magnet**: 负责磁铁运动与 TF 变换（独立运动）
- **mag_pose_estimator**: 估计磁铁姿态
- **mag_tracking_control**: 根据磁体位姿估计结果自动控制机械臂跟踪（可选）
- **mag_viz**: 展示真值与估计结果
- **mag_core_description**: 提供传感器阵列结构描述

## 快速启动

### 构建并加载工作空间

```bash
cd ~/workshop/magnet_pose_estimation
catkin_make
source devel/setup.bash
```

### 仿真链路

启动完整的仿真环境：

```bash
roslaunch mag_single_tracking_app simulation.launch
```

**自定义配置示例**：

```bash
# 使用自定义传感器阵列配置
roslaunch mag_single_tracking_app simulation.launch sensor_array_config:=$(find mag_single_tracking_app)/config/sensor_array.yaml

# 禁用 RViz
roslaunch mag_single_tracking_app simulation.launch use_rviz:=false

# 指定磁体运动轨迹
roslaunch mag_single_tracking_app simulation.launch magnet_config:=$(find mag_single_tracking_app)/config/magnet_config.yaml

# 启用跟踪控制（自动跟踪磁铁，仅规划不执行）
roslaunch mag_single_tracking_app simulation.launch enable_tracking_control:=true enable_execution:=false

# 启用跟踪控制并实际执行运动
roslaunch mag_single_tracking_app simulation.launch enable_tracking_control:=true enable_execution:=true

# 禁用跟踪控制
roslaunch mag_single_tracking_app simulation.launch enable_tracking_control:=false
```

## 目录结构

```
mag_single_tracking_app/
├── launch/              # Launch 文件
│   └── simulation.launch    # 仿真场景启动文件
├── config/              # 配置文件
│   ├── sensor_array.yaml       # 传感器阵列几何配置（包含 TF 偏移量配置）
│   ├── arm_config.yaml         # 机械臂配置
│   ├── magnet_config.yaml      # 磁体运动配置
│   ├── sensor_sim_config.yaml  # 传感器仿真配置
│   ├── estimator_config.yaml  # 姿态估计器配置
│   ├── tracking_control_config.yaml  # 跟踪控制配置（机械臂自动跟踪）
│   ├── magnet_viz_config.yaml # 磁体可视化配置
│   ├── sensor_array_viz_config.yaml  # 传感器阵列可视化配置
│   └── sensor_batcher_config.yaml    # 传感器批量打包配置
├── rviz/                # RViz 配置文件
├── src/                 # 源代码（本包不包含自定义节点，使用 mag_device_sensor 的功能）
├── include/             # 头文件（本包不包含自定义节点）
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 关键特性

### 静态传感器阵列位姿

传感器阵列的位姿通过静态 TF 发布，相对于机械臂末端工具（`bracket_tcp_link`）：

1. **刚性连接**：传感器阵列与工具末端支架之间刚性连接，完全跟随机械臂运动
2. **可配置偏移**：通过 `sensor_array.yaml` 中的 `frames.parent` 和 `pose` 配置传感器阵列相对于工具末端的位置和姿态偏移
3. **TF 发布**：由 `mag_device_sensor` 的 `sensor_sim_node` 负责发布静态 TF（通过 `SensorArrayTfPublisher`）

### TF 树结构

```
world
  └── arm1_base_link (静态，由 mag_device_arm 发布，如果配置了 base_tf)
      └── ... (机械臂关节链，由 MoveIt 发布)
          └── arm1_flange_link (机械臂末端法兰)
              └── arm1_bracket_link (工具基座，静态)
                  └── arm1_bracket_tcp_link (工具末端，静态)
                      └── sensor_array (静态，由 sensor_array_tf_node 发布)
                          ├── sensor_1
                          ├── sensor_2
                          └── ...
```

## 配置文件说明

### 传感器阵列几何配置 (`config/sensor_array.yaml`)

定义传感器阵列中各个传感器的位置和姿态（相对于 `sensor_array` 坐标系），以及传感器阵列相对于工具末端的偏移量：

```yaml
config:
  frames:
    parent: bracket_tcp_link  # 父坐标系：机械臂末端工具
    array: sensor_array       # 传感器阵列坐标系
    sensor_frame_prefix: sensor_
    pose:
      xyz: [0.0, 0.0, 0.0]  # 位置偏移 [x, y, z]（米）
      rpy: [0.0, 0.0, 0.0]  # 姿态偏移 [roll, pitch, yaw]（弧度）
  sensors:
    # ... 传感器配置 ...
```

注意：`mag_device_sensor` 的 `sensor_sim_node` 会根据此配置发布从 `bracket_tcp_link` 到 `sensor_array` 的静态 TF。

### 机械臂配置 (`config/arm_config.yaml`)

定义机械臂控制参数，包括 MoveIt 规划组、末端执行器链接等。

### 磁体运动配置 (`config/magnet_config.yaml`)

定义磁体在空间中的运动轨迹和姿态变化（独立于机械臂运动）。

### 传感器仿真配置 (`config/sensor_sim_config.yaml`)

定义传感器数据生成的参数，包括更新频率、磁体偶极子参数、噪声模型等。

### 跟踪控制配置 (`config/tracking_control_config.yaml`)

定义机械臂自动跟踪磁铁的控制策略和参数：

```yaml
config:
  strategy:
    type: fixed_offset  # 策略类型：fixed_offset, adaptive_distance
    fixed_offset:
      offset:
        x: 0.0
        y: 0.0
        z: 0.05  # 在磁铁上方5cm
      max_movement_per_step: 0.02  # 每步最大移动距离
  control:
    update_rate: 20.0           # 控制循环频率 (Hz)
    enable_execution: false     # 是否实际执行运动
    velocity_scaling: 0.2       # 速度缩放因子
    acceleration_scaling: 0.2   # 加速度缩放因子
```

**跟踪控制功能**：
- 订阅 `/magnetic/pose_estimated` 话题获取磁体位姿估计结果
- 根据配置的策略计算目标传感器位姿
- 调用 `/mag_device_arm/set_end_effector_pose` 服务控制机械臂
- 发布 `/tracking_control/target_pose` 话题用于可视化

**支持的策略**：
- `fixed_offset`: 固定偏移策略，保持传感器相对于磁铁的固定偏移
- `adaptive_distance`: 自适应距离策略，根据磁场强度自动调整距离

### 其他配置

- `estimator_config.yaml`: 姿态估计器配置
- `magnet_viz_config.yaml`: 磁体可视化配置
- `sensor_array_viz_config.yaml`: 传感器阵列可视化配置
- `sensor_batcher_config.yaml`: 传感器批量打包配置

## 依赖包

- `mag_core_description`: 传感器阵列描述
- `mag_core_msgs`: 消息定义
- `mag_core_utils`: 核心工具库
- `mag_device_sensor`: 传感器驱动（仿真模式）
- `mag_device_magnet`: 磁体运动仿真
- `mag_device_arm`: 机械臂控制
- `mag_pose_estimator`: 姿态估计算法
- `mag_tracking_control`: 跟踪控制算法（机械臂自动跟踪）
- `mag_viz`: 可视化工具
- `rosparam_shortcuts`: ROS 参数快捷工具
- `zlab_arm_single_moveit_config`: MoveIt 配置（Gazebo 仿真）
- `zlab_arm_bringup`: 硬件接口和 MoveIt 启动配置

## 与 mag_sensor_movable_app 的区别

| 特性 | mag_sensor_movable_app | mag_single_tracking_app |
|------|----------------------|------------------------|
| 传感器阵列位置 | 可移动（独立运动） | 固定在机械臂末端 |
| TF 发布方式 | 动态 TF（轨迹生成） | 静态 TF（刚性连接） |
| 父坐标系 | world | bracket_tcp_link |
| 适用场景 | 可移动阵列（仅仿真） | 单臂跟踪（仅仿真） |
| 运动控制 | 支持多种运动轨迹 | 跟随机械臂运动 |
| 硬件支持 | 否（仅仿真） | 否（仅仿真） |

## 使用示例

### 基本使用

启动完整仿真链路：

```bash
roslaunch mag_single_tracking_app simulation.launch
```

### 控制机械臂

通过 `mag_device_arm` 的服务接口控制机械臂：

```bash
# 设置末端执行器位姿
rosservice call /mag_device_arm/set_end_effector_pose \
  "arm: 'sensor'
   target:
     position: {x: 0.1, y: 0.0, z: 0.1}
     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   velocity_scaling: 0.1
   acceleration_scaling: 0.1
   execute: true"
```

### 调整传感器阵列偏移

编辑 `config/sensor_array.yaml` 中的 `frames.pose` 字段，修改传感器阵列相对于工具末端的位置和姿态。

### 使用跟踪控制

跟踪控制功能可以根据磁体位姿估计结果自动控制机械臂跟踪磁铁：

```bash
# 启用跟踪控制（仅规划，不执行）
roslaunch mag_single_tracking_app simulation.launch enable_tracking_control:=true enable_execution:=false

# 启用跟踪控制并实际执行运动
roslaunch mag_single_tracking_app simulation.launch enable_tracking_control:=true enable_execution:=true
```

跟踪控制节点会：
1. 订阅 `/magnetic/pose_estimated` 获取磁体位姿估计
2. 根据配置的策略计算目标传感器位姿
3. 调用机械臂服务执行运动（如果 `enable_execution:=true`）
4. 发布 `/tracking_control/target_pose` 用于可视化

可以通过修改 `config/tracking_control_config.yaml` 来调整跟踪策略和参数。

