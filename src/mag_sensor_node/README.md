## mag_sensor_node（重构版）

高频磁传感器阵列的统一驱动与仿真节点：

- **硬件驱动**：串口采集原始原始计数，统一转换为毫特斯拉，并按每个传感器帧发布。
- **仿真节点**：基于磁偶极子模型实时生成虚拟测量，发布与硬件完全一致的话题。
- **阵列配置共用**：`config/array.yaml` 描述阵列几何，硬件与仿真共享，TF 发布逻辑一致。
- **TF 标准化**：固定 `parent -> array -> sensor_<id>` 链条，可选固定频率或静态广播。

## 文件结构

- `config/array.yaml`：传感器阵列几何（父帧、阵列帧、姿态、传感器位姿）。
- `config/hardware.yaml`：硬件驱动参数（串口、话题、标定、TF）。
- `config/simulation.yaml`：仿真参数（更新频率、磁偶极子、噪声模型、话题、TF）。
- `launch/mag_sensor_node.launch`：加载阵列 + 硬件配置，启动实机驱动。
- `launch/mag_sensor_sim_node.launch`：加载阵列 + 仿真配置，启动模拟器。

## 配置说明

### 阵列几何 (`config/array.yaml`)

```yaml
array:
    parent_frame: tool_tcp       # 阵列父坐标系
    frame: mag_sensor_array      # 阵列自身的 TF 名称
    sensor_frame_prefix: mag_sensor_
    tf_publish_rate: 30.0        # ≤0 表示使用静态 TF
    pose:
        xyz: [0.03, 0.0, -0.02]    # parent → array 位姿
        rpy: [-1.5708, 3.1416, 0.0]
    sensors:
        - id: 1
            pose:
                xyz: [0.00, 0.04, 0.00]
                rpy: [0.0, 0.0, 0.0]
        # ... 其余传感器同理
```

### 硬件驱动 (`config/hardware.yaml`)

- `driver/*`：串口与运行节拍，例如 `serial_port`、`baud_rate`、`poll_sleep_ms`、`freq_stat_period`。
- `topics/raw`、`topics/field`：输出话题名称，可为空关闭对应发布。
- `calibration/full_scale_mT`、`calibration/raw_max`：计数与物理量换算。
- `tf/enable`、`tf/publish_rate`：是否广播阵列 TF 以及频率（默认沿用 `array.tf_publish_rate`）。

### 仿真节点 (`config/simulation.yaml`)

- `simulation/update_rate_hz`：磁场计算与发布频率。
- `simulation/magnet_frame/{parent,child}`：从 TF 查询磁铁位姿所用的帧。
- `simulation/magnet/{dipole_strength,base_direction}`：偶极子强度与本体方向（归一化即可）。
- `simulation/noise/*`：噪声开关、类型（`gaussian` 或 `uniform`）、均值、方差/幅度。
- `topics/*`、`calibration/*`、`tf/*` 同硬件节点。

## 启动示例

```bash
# 1. 实机采集：
roslaunch mag_sensor_node mag_sensor_node.launch serial_port:=/dev/ttyUSB0 baud_rate:=921600

# 2. 仿真：
roslaunch mag_sensor_node mag_sensor_sim_node.launch magnet_child_frame:=virtual_magnet
```

两个 launch 均会先加载 `array.yaml`，保证 TF 与传感器帧命名一致。可在命令行或上层 launch 覆盖任意 `~` 私有参数。

## 发布话题

- `topics/raw`（默认 `/mag_sensor/data_raw`）：`magnet_msgs/MagSensorData`
    - `header.frame_id = sensor_<id>`（即每个传感器自己的 TF 帧）。
    - `mag_*` 为原始计数或仿真反推的计数。
- `topics/field`（默认 `/mag_sensor/data_mT`）：同一消息类型，数值按毫特斯拉输出。

## TF 链路

- 阵列构造统一由 `SensorArrayTfPublisher` 管理；`tf_publish_rate > 0` 时按频率刷新，≤0 时通过 static TF 一次性广播。
- 框架自动生成：`array.parent_frame -> array.frame -> sensor_frame_prefix + id`。
- 仿真节点会读取同一配置，确保计算磁场与 TF 一致。

## 串口数据格式（硬件）

```
[ID]: mx my mz
```
- 支持任意空白符分隔，`ID` 需可转为整数。
- 非法行会被节流告警，随后丢弃。

## 依赖

- ROS 1（roscpp、geometry_msgs、tf2、tf2_ros、serial、magnet_msgs）。
- Eigen3（用于磁场模型计算）。

## 常见问题

- **串口无法打开**：确认当前用户在 `dialout` 组，或检查设备路径。
- **没有 TF**：确保未禁用 `tf/enable`，若设定静态 TF 需注意只有一次广播。
- **仿真无输出**：检查 `simulation/magnet_frame` 是否在 TF 树中存在，或外部是否提供磁铁位姿。
- **单位不一致**：修改 `calibration/full_scale_mT` 与 `raw_max` 以匹配实际传感器。

## 构建

```bash
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

## License

MIT

