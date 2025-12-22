# mag_sensor_calibration

磁场传感器校正应用包。通过机械臂自动旋转传感器阵列，采集不同角度下的传感器数据，对每个传感器进行单独校正。

## 功能特性

- **自动旋转采集**：通过机械臂控制传感器阵列进行旋转运动，采集不同角度下的数据
- **多轴支持**：支持绕X、Y、Z轴或组合轴的旋转
- **单独校正**：对25个传感器中的每个传感器进行单独校正
- **误差校正**：
  - 硬磁偏移（Hard Iron Offset）
  - 软磁干扰（Soft Iron Distortion）
  - 比例因子误差（Scale Factor）
  - 正交误差（Non-orthogonality）
- **数据保存**：原始数据保存为CSV格式，校正参数保存为YAML格式
- **可视化**：在RViz中实时显示采集进度和校正结果

## 包结构

```
mag_sensor_calibration/
├── src/                          # 源代码
│   ├── calibration_algorithm.cpp          # 校正算法实现
│   ├── calibration_controller_node.cpp     # 主控制节点
│   ├── calibration_controller_main.cpp     # 主函数
│   ├── data_collector_node.cpp             # 数据采集节点
│   ├── data_collector_main.cpp             # 主函数
│   ├── calibration_viz_node.cpp           # 可视化节点
│   └── calibration_viz_main.cpp           # 主函数
├── include/mag_sensor_calibration/         # 头文件
│   ├── calibration_algorithm.hpp
│   ├── calibration_controller_node.hpp
│   ├── data_collector_node.hpp
│   └── calibration_viz_node.hpp
├── srv/                          # ROS服务定义
│   ├── StartCalibration.srv
│   ├── StopCalibration.srv
│   └── SaveCalibrationData.srv
├── config/                       # 配置文件
│   └── default_config.yaml
├── launch/                       # Launch文件
│   └── calibration.launch
└── rviz/                         # RViz配置（可选）
```

## 使用方法

### 1. 启动校正系统

```bash
roslaunch mag_sensor_calibration calibration.launch
```

### 2. 启动校正流程

通过ROS服务启动校正：

```bash
# 启动校正
rosservice call /calibration_controller/start_calibration \
  "arm: 'arm1'
   start_pose:
     position: {x: 0.5, y: 0.0, z: 0.3}
     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   rotation_center: {x: 0.5, y: 0.0, z: 0.3}
   rotation_axis: 'z'
   angle_range: [0.0, 3.14159]
   angle_step: 0.174533
   velocity_scaling: 0.2
   data_collection_duration: 2.0
   save_raw_data: true
   output_directory: '/tmp/calibration_data'"
```

### 3. 停止校正

```bash
rosservice call /calibration_controller/stop_calibration \
  "session_id: '20240101_120000'
   save_data: true"
```

### 4. 保存校正数据

```bash
rosservice call /calibration_controller/save_calibration_data \
  "session_id: '20240101_120000'
   output_directory: '/tmp/calibration_data'
   compute_calibration: true"
```

## 校正算法

使用椭球拟合方法进行传感器校正：

1. **数据采集**：在不同角度下采集传感器数据
2. **椭球拟合**：使用最小二乘法拟合测量数据为椭球
3. **参数提取**：
   - 硬磁偏移 = 椭球中心
   - 软磁矩阵 = 椭球变换矩阵的逆
4. **校正应用**：`corrected = soft_iron_matrix * (raw - hard_iron_offset)`

## 输出文件

### CSV数据文件

包含以下列：
- `timestamp`: 时间戳
- `sensor_id`: 传感器ID
- `angle_rad`: 角度（弧度）
- `mag_x_mT`, `mag_y_mT`, `mag_z_mT`: 磁场读数（毫特斯拉）
- `sensor_pos_x/y/z`: 传感器位置
- `sensor_quat_x/y/z/w`: 传感器姿态四元数

### YAML校正参数文件

包含每个传感器的校正参数：
- `hard_iron_offset`: 硬磁偏移向量
- `soft_iron_matrix`: 软磁干扰矩阵（3x3）
- `min_field_strength`, `max_field_strength`: 校正后的磁场强度范围
- `fit_error`: 拟合误差
- `coverage`: 角度覆盖度

## 配置参数

编辑 `config/default_config.yaml` 以调整参数：

- `reference_frame`: 参考坐标系
- `sensor_array_frame`: 传感器阵列坐标系
- `arm_service_name`: 机械臂服务名称
- `data_collector/input_topic`: 传感器数据话题
- `visualization/enabled`: 是否启用可视化

## 依赖

- `mag_core_msgs`: 消息定义
- `mag_device_arm`: 机械臂控制
- `mag_device_sensor`: 传感器数据
- `yaml-cpp`: YAML文件处理
- `Eigen3`: 矩阵运算

## 注意事项

1. 确保机械臂和传感器节点已启动
2. 校正前确保传感器阵列已正确安装
3. 根据工作空间限制调整角度范围
4. 建议每个角度位置采集至少2秒的数据
5. 至少需要9个数据点才能进行椭球拟合

