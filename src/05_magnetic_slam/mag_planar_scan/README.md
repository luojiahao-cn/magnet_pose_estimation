# Planar Scan Experiment

本功能包用于控制单个机械臂带动磁传感器阵列在水平面上进行网格扫描，并记录传感器数据和位置信息。

## 1. 硬件/仿真 设置

确保机械臂（Arm）和传感器阵列（Sensor Array）已正确连接并配置。
在 simulation 模式下，将启动 Gazebo 和虚拟传感器。

## 2. 配置文件

配置文件位于 `config/planar_scan_config.yaml`。

| 参数 | 描述 | 默认值 |
| :--- | :--- | :--- |
| `x_min`, `x_max`, `x_step` | X轴扫描范围及步长 (米) | 0.3, 0.6, 0.05 |
| `y_min`, `y_max`, `y_step` | Y轴扫描范围及步长 (米) | -0.2, 0.2, 0.05 |
| `z_fixed` | 固定的 Z 轴高度 (米) | 0.2 |
| `wait_time` | 每个点停留采集的时间 (秒) | 0.5 |
| `samples_per_point` | 每个点采集的样本数 | 50 |
| `arm_name` | 使用的机械臂名称 | arm1 |

## 3. 运行方法

### 仿真模式

**MoveIt 纯仿真（默认，不启动 Gazebo）:**
```bash
roslaunch mag_planar_scan planar_scan.launch use_sim:=true
```

**Gazebo 物理仿真:**
```bash
roslaunch mag_planar_scan planar_scan.launch use_sim:=true use_gazebo:=true
```

**指定工具（可选，默认为 bracket）:**
```bash
roslaunch mag_planar_scan planar_scan.launch use_sim:=true tool_name:=bracket
```

### 实机模式
```bash
roslaunch mag_planar_scan planar_scan.launch use_sim:=false arm_id:=arm1
```

## 4. 输出数据

数据采集完成后，文件将保存至：
`src/05_magnetic_slam/mag_planar_scan/data/scan_data.csv`

**CSV 格式:**
```
x,y,z,sensor_id,mag_x,mag_y,mag_z
```
* `x, y, z`: 机械臂末端（传感器阵列中心）的世界坐标位置
* `sensor_id`: 传感器编号
* `mag_x, mag_y, mag_z`: 在该位置采集并平均后的磁场强度 (mT)
