# mag_sensor_fixture_app

固定式磁传感器阵列的应用编排包。它不会重新实现感知、仿真或可视化逻辑，而是通过 launch/配置将现有功能包组合到一个可直接运行的流程中，覆盖两类场景：

## 功能概述

### 1. 仿真场景

完整的仿真链路，用于算法开发和测试：

- **mag_device_magnet**: 负责磁铁运动与 TF 变换
- **mag_device_sensor** (仿真模式): 根据 TF 生成 `mag_core_msgs/MagSensorData` 消息
- **mag_pose_estimator**: 估计磁铁姿态
- **mag_viz**: 展示真值与估计结果
- **mag_core_description**: 提供传感器阵列结构描述

### 2. 硬件场景

实际硬件部署链路：

- **mag_device_sensor** (硬件驱动): 读取阵列信号并广播 TF
- **mag_pose_estimator**: 输出姿态估计结果
- **mag_viz**: 展示估计结果（可叠加外部真值源）

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
roslaunch mag_sensor_fixture_app simulation.launch
```

**自定义配置示例**：

```bash
# 使用圆形轨迹
roslaunch mag_sensor_fixture_app simulation.launch magnet_config:=$(find mag_device_magnet)/config/circular.yaml

# 禁用 RViz
roslaunch mag_sensor_fixture_app simulation.launch use_rviz:=false
```

### 硬件链路

**前置条件**：
1. 连接传感器阵列到计算机
2. 在 `hardware/config/sensor_driver.yaml` 中设置串口等参数（详见 `hardware/README.md`）

**启动命令**：

```bash
roslaunch mag_sensor_fixture_app hardware.launch
```

**自定义配置**：

```bash
# 使用自定义硬件配置
roslaunch mag_sensor_fixture_app hardware.launch \
  sensor_hardware_config:=$(find mag_sensor_fixture_app)/hardware/config/sensor_driver.yaml
```

## 目录结构

```
mag_sensor_fixture_app/
├── launch/              # Launch 文件
│   ├── simulation.launch    # 仿真场景启动文件
│   └── hardware.launch      # 硬件场景启动文件
├── config/              # 应用级配置文件
├── simulation/          # 仿真相关配置和说明
├── hardware/            # 硬件相关配置和说明
│   └── config/
│       └── sensor_driver.yaml  # 硬件驱动配置
├── rviz/                # RViz 配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 相关文档

- `simulation/README.md`: 仿真场景详细说明
- `hardware/README.md`: 硬件配置说明
- `config/README.md`: 配置文件说明

## 依赖包

- `mag_core_description`: 传感器阵列描述
- `mag_core_msgs`: 消息定义
- `mag_device_sensor`: 传感器驱动（仿真/硬件）
- `mag_device_magnet`: 磁体运动仿真
- `mag_pose_estimator`: 姿态估计算法
- `mag_viz`: 可视化工具
