# Dual Arm Magnet Tracking Experiment

本功能包用于双臂协作数据采集实验：
* **Magnet Arm (Arm 1)**: 带动永磁体按照预设轨迹（圆形）运动。
* **Sensor Arm (Arm 2)**: 带动传感器阵列，保持在该磁铁上方固定偏移处跟随运动。

## 1. 实验设置

该实验需要两台机械臂（如 FR5，基于 zlab_robots），一台安装永磁体，一台安装传感器阵列。

## 2. 配置文件

配置文件位于 `config/dual_tracking.yaml`。

| 参数 | 描述 | 默认值 |
| :--- | :--- | :--- |
| `magnet_arm_name` | 磁铁臂名称 | arm1 |
| `sensor_arm_name` | 传感器臂名称 | arm2 |
| `center_x`, `center_y`, `center_z` | 磁铁运动圆心的世界坐标 | 0.5, 0.0, 0.3 |
| `radius` | 磁铁运动半径 | 0.1 |
| `z_offset` | 传感器臂相对于磁铁臂的 Z 轴垂直偏移 | 0.2 |

## 3. 运行方法

### 仿真模式
```bash
roslaunch mag_dual_arm_tracking dual_tracking.launch use_sim:=true
```

### 实机模式
确保两台机械臂 IP 已在 `mag_device_arm` 中配置正确。
```bash
roslaunch mag_dual_arm_tracking dual_tracking.launch use_sim:=false
```

## 4. 输出数据

实验过程中会实时记录 ROS Bag 文件，包含传感器原始数据和位姿真值。
文件路径：`src/05_magnetic_slam/mag_dual_arm_tracking/data/experiment.bag`

**记录的话题:**
* `/magnetic/sensor/field`: 传感器阵列原始磁场数据 (`mag_core_msgs/MagSensorData`)
* `/ground_truth/magnet_pose`: 磁铁臂目标位姿 (`geometry_msgs/PoseStamped`)
* `/ground_truth/sensor_pose`: 传感器臂目标位姿 (`geometry_msgs/PoseStamped`)
* 以及系统默认的 TF 树信息 (如包含)
