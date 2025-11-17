# mag_sensor_fixture_app

固定式磁传感器阵列的应用编排包。它不会重新实现感知、仿真或可视化逻辑，而是通过 launch/配置将现有功能包组合到一个可直接运行的流程中，覆盖两类场景：

1. **仿真**：`mag_device_magnet` 负责磁铁运动与 TF；`mag_device_sensor`（仿真模式）根据 TF 生成 `mag_core_msgs/MagSensorData`；`mag_pose_estimator` 估计磁铁姿态；`mag_viz` 展示真值与估计；阵列结构来自 `mag_core_description`。
2. **实物**：`mag_device_sensor`（硬件驱动）读取阵列信号并广播 TF；`mag_pose_estimator` 输出姿态；`mag_viz` 仅展示估计结果（可叠加外部真值源）。

## 目录总览

```
config/
  fixture_array.yaml        # 传感器阵列描述 (mag_core_description 兼容)
  pose_estimator.yaml       # 固定阵列默认估计参数
  magnet_viz.yaml           # 仿真场景下真值+估计可视化
  magnet_viz_hw.yaml        # 实物场景仅估计可视化
launch/
  simulation.launch         # 一键仿真：磁铁 + 传感器仿真 + 估计 + 可视化
  hardware.launch           # 实物快速联调：传感器驱动 + 估计 + 可视化
simulation/config/
  magnet_motion.yaml        # 磁铁轨迹/TF 配置 (mag_device_magnet)
  sensor_sim.yaml           # 传感器仿真配置 (mag_device_sensor)
hardware/config/
  sensor_driver.yaml        # 传感器硬件驱动参数模板
rviz/
  mag_sensor_fixture.rviz   # 专用 RViz 视图，展示 magnet_viz 与 sensor_array_viz 的输出
```

若需定制：
- **阵列几何**：复制 `config/fixture_array.yaml` 并在 launch 中通过 `array_file` 参数覆盖。
- **轨迹/噪声**：编辑 `simulation/config/*`。保持 `frame` 与 `mag_pose_estimator` 输出帧一致（默认为 `sensor_array/base`）。
- **驱动与偏置**：在 `hardware/config/sensor_driver.yaml` 中调整串口、话题及 TF 发布频率。

## 快速启动

构建并加载工作空间：

```bash
catkin_make
source devel/setup.bash
```

仿真链路：

```bash
roslaunch mag_sensor_fixture_app simulation.launch
```

硬件链路（需先连接阵列并在 `hardware/config/sensor_driver.yaml` 中设置串口等参数）：

```bash
roslaunch mag_sensor_fixture_app hardware.launch
```

如需切换多套配置，可使用 `roslaunch mag_sensor_fixture_app hardware.launch sensor_driver_config:=/path/to/custom.yaml`，并同样支持覆盖 `array_file`、`estimator_config`、`viz_config`、`sensor_viz_config` 以及 `rviz_config`（默认指向 `rviz/mag_sensor_fixture.rviz`）。