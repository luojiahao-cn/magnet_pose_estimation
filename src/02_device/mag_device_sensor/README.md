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
```

两个 launch 文件分别加载阵列几何与驱动/仿真配置。默认使用 `mag_core_description/config/sensor_array.yaml` 与本包下的 `config/` 示例。

> 注意：阵列几何 YAML 以 `config:` 为根节点，并通过 `<rosparam ns="array" ...>` 加载到节点的 `~array/config` 路径下。若自行提供文件，也请保持相同结构并在 launch 中指定 `ns="array"`，以便新配置加载器解析。

### 配置结构

- `driver.*`（仅限硬件驱动）：串口端口、波特率、轮询周期等设置。
- `calibration.*`：ADC 与物理量之间的换算参数。
- `topics.*`：原始计数与物理量化后的磁场话题名称，至少启用一个。
- `simulation.*`（仿真节点）：磁铁 TF 来源、偶极子参数、噪声模型。
- `tf.*`：是否发布 TF 以及动态发布频率（`publish_rate <= 0` 表示仅发送静态 TF）。

阵列几何配置需通过 `mag_core_description` 提供的 `SensorArrayDescription` 保持实机与仿真一致。
