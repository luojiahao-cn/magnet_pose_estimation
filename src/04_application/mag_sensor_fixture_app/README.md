# mag_sensor_fixture_app

固定式磁传感器阵列的应用编排包。它不会重新实现感知、仿真或可视化逻辑，而是通过 launch/配置将现有功能包组合到一个可直接运行的流程中。

## 功能概述

### 仿真场景

完整的仿真链路，用于算法开发和测试：

- **mag_device_magnet**: 负责磁铁运动与 TF 变换
- **mag_device_sensor** (仿真模式): 根据 TF 生成 `mag_core_msgs/MagSensorData` 消息
- **mag_pose_estimator**: 估计磁铁姿态
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
roslaunch mag_sensor_fixture_app simulation.launch
```

**自定义配置示例**：

```bash
# 使用圆形轨迹
roslaunch mag_sensor_fixture_app simulation.launch magnet_config:=$(find mag_device_magnet)/config/circular.yaml

# 禁用 RViz
roslaunch mag_sensor_fixture_app simulation.launch use_rviz:=false
```

## 目录结构

```
mag_sensor_fixture_app/
├── launch/              # Launch 文件
│   └── simulation.launch    # 仿真场景启动文件
├── config/              # 应用级配置文件
├── rviz/                # RViz 配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖包

- `mag_core_description`: 传感器阵列描述
- `mag_core_msgs`: 消息定义
- `mag_device_sensor`: 传感器驱动（仿真）
- `mag_device_magnet`: 磁体运动仿真
- `mag_pose_estimator`: 姿态估计算法
- `mag_viz`: 可视化工具
