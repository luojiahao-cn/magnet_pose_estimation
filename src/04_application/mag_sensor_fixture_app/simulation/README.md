# 仿真配置说明

## 仿真场景组件

仿真链路包含以下组件：

1. **mag_device_magnet**: 磁体运动仿真
   - 生成磁体的运动轨迹
   - 发布磁体姿态的 TF 变换
   - 发布磁体真值姿态话题

2. **mag_device_sensor** (仿真模式): 传感器数据仿真
   - 根据 TF 变换计算各传感器的磁场测量值
   - 发布 `mag_core_msgs/MagSensorData` 消息

3. **mag_pose_estimator**: 姿态估计
   - 接收传感器数据
   - 估计磁体姿态
   - 发布估计结果

4. **mag_viz**: 可视化
   - 显示磁体真值姿态
   - 显示估计姿态
   - 显示传感器阵列

## 配置文件

本目录包含仿真场景的所有配置文件：

- `magnet_config.yaml` - 磁体设备配置（轨迹、运动参数等）
- `sensor_sim_config.yaml` - 传感器仿真配置（更新频率、噪声等）
- `estimator_config.yaml` - 姿态估计器配置（算法参数、优化器设置等）
- `magnet_viz_config.yaml` - 磁体可视化配置
- `sensor_array_viz_config.yaml` - 传感器阵列可视化配置

## 自定义配置

可以通过 launch 文件参数自定义各组件配置：

```bash
# 使用圆形轨迹
roslaunch mag_sensor_fixture_app simulation.launch \
  magnet_config:=$(find mag_device_magnet)/config/circular.yaml

# 使用自定义估计器配置
roslaunch mag_sensor_fixture_app simulation.launch \
  estimator_config:=$(find mag_pose_estimator)/config/mag_pose_estimator.yaml

# 禁用 RViz
roslaunch mag_sensor_fixture_app simulation.launch use_rviz:=false
```

## 支持的磁体轨迹类型

- `static`: 静态轨迹（默认）
- `circular`: 圆形轨迹
- `rectangular`: 矩形轨迹

可以通过修改 `magnet_config.yaml` 中的 `trajectory.type` 字段来切换轨迹类型。

