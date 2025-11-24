# mag_sensor_movable_app

可移动式磁传感器阵列的仿真应用编排包。它不会重新实现感知、仿真或可视化逻辑，而是通过 launch/配置将现有功能包组合到一个可直接运行的仿真流程中，用于可移动传感器阵列场景的算法开发和测试。

## 功能概述

与固定式传感器阵列（`mag_sensor_fixture_app`）不同，本包针对传感器阵列可以在空间中移动的场景。主要区别：

- **传感器阵列位置动态变化**：传感器阵列的位姿可以通过 TF 动态更新
- **支持多种运动模式**：通过配置文件定义传感器阵列的运动轨迹（圆形、矩形、直线等）
- **动态 TF 发布**：传感器阵列的 TF 变换会动态更新，而不是静态固定
- **仅支持仿真**：本包仅用于仿真场景，不包含硬件驱动

### 仿真场景

完整的仿真链路，用于算法开发和测试：

- **mag_device_magnet**: 负责磁铁运动与 TF 变换
- **array_motion_node**: 负责传感器阵列运动与 TF 变换（本包提供）
- **mag_device_sensor** (仿真模式): 根据 TF 生成 `mag_core_msgs/MagSensorData` 消息
- **mag_pose_estimator**: 估计磁铁姿态
- **mag_viz**: 展示真值与估计结果
- **mag_core_description**: 提供传感器阵列结构描述（支持动态 TF）

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
roslaunch mag_sensor_movable_app simulation.launch
```

**自定义配置示例**：

```bash
# 使用圆形轨迹
roslaunch mag_sensor_movable_app simulation.launch magnet_config:=$(find mag_device_magnet)/config/circular.yaml

# 禁用 RViz
roslaunch mag_sensor_movable_app simulation.launch use_rviz:=false

# 指定传感器阵列运动轨迹
roslaunch mag_sensor_movable_app simulation.launch array_motion_config:=$(find mag_sensor_movable_app)/config/array_motion.yaml
```

## 目录结构

```
mag_sensor_movable_app/
├── launch/              # Launch 文件
│   └── simulation.launch    # 仿真场景启动文件
├── config/              # 配置文件
│   ├── array_motion.yaml    # 传感器阵列运动配置
│   ├── sensor_array.yaml    # 传感器阵列几何配置
│   ├── magnet_config.yaml    # 磁体运动配置
│   ├── sensor_sim_config.yaml    # 传感器仿真配置
│   ├── estimator_config.yaml    # 姿态估计器配置
│   ├── magnet_viz_config.yaml    # 磁体可视化配置
│   ├── sensor_array_viz_config.yaml    # 传感器阵列可视化配置
│   └── sensor_batcher_config.yaml    # 传感器批量打包配置
├── rviz/                # RViz 配置文件
├── src/                 # 源代码
│   ├── array_motion_node.cpp    # 传感器阵列运动控制节点
│   ├── array_motion_config_loader.cpp    # 配置加载器
│   └── array_motion_main.cpp    # 主程序
├── include/             # 头文件
│   └── mag_sensor_movable_app/
│       ├── array_motion_node.hpp
│       └── array_motion_config_loader.hpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 关键特性

### 动态传感器阵列位姿

传感器阵列的位姿通过配置文件定义的运动轨迹提供：

1. **静态模式**：传感器阵列固定在初始位置
2. **圆形轨迹**：传感器阵列在二维平面上做圆周运动
3. **矩形轨迹**：传感器阵列沿矩形边界运动
4. **直线运动**：传感器阵列在两点间直线运动（支持往返）

### TF 树结构

```
world
  └── sensor_array (动态更新)
      ├── sensor_1
      ├── sensor_2
      └── ...
```

## 配置文件说明

### 传感器阵列运动配置 (`config/array_motion.yaml`)
定义传感器阵列的运动轨迹，支持静态、圆形、矩形、直线等运动模式。

### 磁体运动配置 (`config/magnet_config.yaml`)
定义磁体在空间中的运动轨迹和姿态变化。

### 传感器仿真配置 (`config/sensor_sim_config.yaml`)
定义传感器数据生成的参数，包括更新频率、磁体偶极子参数、噪声模型等。

### 其他配置
- `sensor_array.yaml`: 传感器阵列几何配置
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
- `mag_pose_estimator`: 姿态估计算法
- `mag_viz`: 可视化工具
- `rosparam_shortcuts`: ROS 参数快捷工具

## 与 mag_sensor_fixture_app 的区别

| 特性 | mag_sensor_fixture_app | mag_sensor_movable_app |
|------|----------------------|----------------------|
| 传感器阵列位置 | 固定 | 可移动 |
| TF 发布方式 | 静态 TF | 动态 TF |
| 适用场景 | 固定安装（仿真/硬件） | 可移动阵列（仅仿真） |
| 运动控制 | 无 | 支持多种运动轨迹 |
| 硬件支持 | 是 | 否（仅仿真） |

