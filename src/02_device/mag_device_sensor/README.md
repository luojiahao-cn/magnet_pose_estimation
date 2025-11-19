# mag_device_sensor

封装磁传感器阵列的硬件驱动与仿真节点，统一读取 `mag_core_description` 的几何配置并发布 `mag_core_msgs` 数据接口。

## 运行示例

```bash
# 实机驱动
roslaunch mag_device_sensor hardware.launch \
	array_file:=/path/to/sensor_array.yaml \
	config_file:=/path/to/hardware.yaml

# 仿真节点
roslaunch mag_device_sensor simulation.launch \
	array_file:=/path/to/sensor_array.yaml \
	config_file:=/path/to/simulation.yaml

# 启用批量打包节点（可选）
roslaunch mag_device_sensor simulation.launch use_batcher:=true

# 或者单独启动批量打包节点
roslaunch mag_device_sensor sensor_batcher.launch \
	config_file:=/path/to/sensor_batcher_config.yaml
```

两个 launch 文件分别加载阵列几何与驱动/仿真配置。默认使用 `mag_core_description/config/sensor_array.yaml` 与本包下的 `config/` 示例。

> 注意：阵列几何 YAML 以 `config:` 为根节点，并通过 `<rosparam ns="array" ...>` 加载到节点的 `~array/config` 路径下。若自行提供文件，也请保持相同结构并在 launch 中指定 `ns="array"`，以便新配置加载器解析。

### 节点说明

1. **sensor_sim_node**：传感器仿真节点，根据磁体 TF 生成磁传感器数据
2. **sensor_driver_node**：传感器硬件驱动节点，读取硬件传感器数据
3. **sensor_array_batcher_node**：传感器批量打包节点，将单个传感器消息打包成批量数据

### 配置结构

#### 传感器驱动/仿真配置

- `driver.*`（仅限硬件驱动）：串口端口、波特率、轮询周期等设置。
- `calibration.*`：ADC 与物理量之间的换算参数。
- `topics.*`：原始计数与物理量化后的磁场话题名称，至少启用一个。
- `simulation.*`（仿真节点）：磁铁 TF 来源、偶极子参数、噪声模型。
- `tf.*`：是否发布 TF 以及动态发布频率（`publish_rate <= 0` 表示仅发送静态 TF）。

#### 批量打包节点配置

- `topics.input`：输入话题（单个传感器数据，如 `/magnetic/sensor/field`）
- `topics.output`：输出话题（批量数据，如 `/magnetic/sensor/batch`）
- `frames.output`：参考坐标系（用于批量消息 header.frame_id，通常是 `sensor_array`）
- `params.publish_rate_hz`：发布频率（Hz，建议 100 Hz）
- `params.timeout_seconds`：传感器数据超时时间（秒）
- `params.min_sensors`：最小传感器数量（低于此数量不发布，0 表示不限制，自动适应传感器数量）

### 批量打包节点优势

- **减少消息数量**：从 N×100Hz 降到 100Hz（N 为传感器数量）
- **时间同步**：所有传感器数据使用统一时间戳
- **避免队列溢出**：订阅者只需处理批量消息，无需大队列缓存
- **自动适应**：传感器数量变化时自动适应，无需修改代码

阵列几何配置需通过 `mag_core_description` 提供的 `SensorArrayDescription` 保持实机与仿真一致。
