# mag_pose_estimator

磁铁姿态估计 ROS 包，提供基于磁偶极子模型的磁铁位置和姿态估计功能。

## 功能特性

- **多种估计算法**：
  - **EKF（扩展卡尔曼滤波器）**：适合实时单次测量更新，使用归一化磁场方向
  - **优化器（Ceres Solver）**：批量优化，同时估计位置、方向和磁矩强度，使用解析雅可比矩阵

- **数据预处理**：
  - 软/硬铁校准（补偿传感器周围铁磁材料干扰）
  - 低通滤波（抑制传感器噪声）

- **灵活的输入接口**：
  - 支持单个传感器数据（`MagSensorData`，EKF 模式）
  - 支持批量传感器数据（`MagSensorBatch`，优化器模式，推荐）

## 文件结构

```
include/mag_pose_estimator/
├── estimator_base.h              # 估计器基类和配置结构体
├── mag_pose_estimator_ekf.h      # EKF 估计器
├── mag_pose_estimator_optimization.h  # 优化器估计器
├── mag_preprocessor.h            # 磁场数据预处理器
├── mag_pose_estimator_node.h     # ROS 节点主类
└── magnetic_field_model.h        # 磁偶极子模型和解析雅可比

src/
├── mag_pose_estimator_ekf.cpp
├── mag_pose_estimator_optimization.cpp
├── mag_preprocessor.cpp
└── mag_pose_estimator_node.cpp

config/
└── mag_pose_estimator.yaml       # 配置文件示例

launch/
└── mag_pose_estimator.launch     # 启动文件示例
```

## 配置说明

### 基本配置

```yaml
config:
  frames:
    output_frame: sensor_array  # 输出姿态的参考坐标系

  topics:
    mag_field: /magnetic/sensor/field      # 单个传感器数据（EKF 模式）
    mag_batch: /magnetic/sensor/batch      # 批量传感器数据（优化器模式，推荐）
    pose_estimate: /magnetic/pose_estimated  # 输出话题

  params:
    estimator:
      type: optimizer  # "ekf" 或 "optimizer"
      min_sensors: 6   # 优化器所需的最小传感器数量
      tf_timeout: 0.05  # TF 查询超时时间（秒）
```

### EKF 参数（仅当 `type: "ekf"` 时使用）

```yaml
    ekf:
      position_gain: 0.02  # 位置增益系数
      process_noise_position: 1.0e-4  # 位置过程噪声方差
      process_noise_orientation: 5.0e-5  # 姿态过程噪声方差
      measurement_noise: 1.0e-3  # 测量噪声方差
      world_field: [0.02, 0.01, 0.045]  # 地球磁场向量 [x, y, z] (mT)
```

### 优化器参数（仅当 `type: "optimizer"` 时使用）

```yaml
    optimizer:
      initial_position: [0.0, 0.0, 0.8]  # 初始位置估计 [x, y, z] (米)
      initial_direction: [0.0, 0.0, 1.0]  # 初始磁矩方向向量（归一化）
      initial_strength: 10  # 初始磁矩强度 (Am²)
      strength_delta: 5.0  # 磁矩强度优化范围 (±delta)
      optimize_strength: true  # 是否优化磁矩强度
      max_iterations: 60  # 最大迭代次数
      function_tolerance: 1.0e-6  # 函数值收敛容差
      gradient_tolerance: 1.0e-8  # 梯度收敛容差
      parameter_tolerance: 1.0e-8  # 参数收敛容差
      num_threads: 2  # 并行计算线程数
      minimizer_progress: false  # 是否显示优化进度
      linear_solver: DENSE_QR  # 线性求解器类型
```

### 预处理器参数

```yaml
    preprocessor:
      enable_calibration: false  # 是否启用软/硬铁校准
      soft_iron_matrix: [1.02, 0.00, 0.00,  # 软铁校准矩阵 (3x3)
                         0.00, 0.99, 0.01,
                         0.00, 0.01, 1.01]
      hard_iron_offset: [1.0, -2.0, 0.5]  # 硬铁偏移向量 [x, y, z] (mT)
      enable_filter: false  # 是否启用低通滤波器
      low_pass_alpha: 0.2  # 低通滤波器系数 (0-1)
```

## 算法说明

### EKF 估计器

- **状态向量**：`[位置(3), 四元数(4), 偏置(3)]`，共 10 维
- **观测模型**：基于归一化磁场方向，通过旋转世界坐标系磁场向量得到预测值
- **特点**：实时性好，适合单次测量更新，但精度相对较低

### 优化器估计器

- **优化变量**：位置 [x, y, z]、方向向量（归一化）、磁矩强度（可选）
- **残差函数**：基于磁偶极子模型，计算预测磁场与测量磁场的差值
- **雅可比计算**：使用解析公式计算，不依赖数值微分或自动微分
- **特点**：精度高，需要多个传感器数据，计算量较大

## 构建与运行

### 构建

```bash
cd ~/workshop/magnet_pose_estimation
catkin_make
source devel/setup.bash
```

### 运行

```bash
# 使用默认配置
roslaunch mag_pose_estimator mag_pose_estimator.launch

# 使用自定义配置
roslaunch mag_pose_estimator mag_pose_estimator.launch config_file:=$(rospack find mag_pose_estimator)/config/mag_pose_estimator.yaml
```

## 输出消息

节点发布 `mag_core_msgs::MagnetPose` 消息，包含：
- `header`：时间戳和坐标系
- `position`：磁铁位置 [x, y, z] (米)
- `orientation`：磁铁姿态（四元数）
- `magnetic_strength`：磁矩强度 (Am²)

## 依赖

- ROS Melodic/Noetic
- `mag_core_msgs`：消息定义
- `mag_core_utils`：工具库
- `Ceres Solver`：优化库
- `Eigen3`：线性代数库
- `tf2_ros`：TF 变换库

## 注意事项

1. **单位**：所有磁场数据单位均为 mT（毫特斯拉）
2. **坐标系**：确保传感器位置可通过 TF 查询获取
3. **批量数据**：优化器模式推荐使用批量数据接口，提高效率和精度
4. **初始值**：优化器的初始值设置对收敛性有重要影响

