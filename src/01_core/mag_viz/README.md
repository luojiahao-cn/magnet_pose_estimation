# mag_viz

负责磁场、姿态、轨迹等可视化展示，提供 RViz 配置与可视化节点。

## 功能

- 订阅 `mag_core_msgs/MagnetPose` 真值姿态，生成红/蓝半圆柱与磁场强度文本标记。
- 可选订阅 `geometry_msgs/PoseStamped` 类型的 `/magnetic/pose_estimated`，用青绿色圆柱与独立轨迹展示估计姿态。
- 维护可配置的轨迹队列，同时发布 `nav_msgs/Path` 与折线 `Marker`。
- 订阅 `mag_core_msgs/MagSensorData`，根据阵列描述绘制传感器磁场矢量箭头，可选数值文本。
- 提供一键启动的 RViz 视图（包含 TF、Marker Array、Path 及传感器 Marker）。

## 快速开始

```bash
roslaunch mag_viz magnet_viz.launch
```

默认会加载 `config/magnet_viz.yaml` 并打开 `rviz/magnet_viz.rviz`。若只想运行节点可关闭 `use_rviz`：

```bash
roslaunch mag_viz magnet_viz.launch use_rviz:=false
```

传感器阵列可视化默认会与磁铁可视化一同启动，可通过 `enable_sensor_viz:=false` 关闭：

```bash
roslaunch mag_viz magnet_viz.launch enable_sensor_viz:=false
```

若需要自定义传感器渲染参数或阵列描述，可传入 `sensor_config` 与 `sensor_array_config` 参数指向相应 YAML。

常用参数位于 `config/magnet_viz.yaml`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/magnetic/pose_true` | 输入真值姿态话题 |
| `estimated_pose_topic` | `/magnetic/pose_estimated` | 姿态估计话题 (`PoseStamped`) |
| `enable_estimated_pose` | `true` | 是否渲染估计姿态 |
| `frame_id` | `world` | 未提供 frame_id 时的回退 |
| `trail_size` | 600 | 轨迹缓存上限（数量） |
| `trail_duration` | 45.0 | 轨迹保留时长（秒，<=0 表示无限） |
| `trail_min_distance` | 0.001 | 相邻样本的最小距离过滤 |
| `trail_width` | 0.003 | 真值轨迹折线宽度 |
| `estimated_trail_width` | 0.0025 | 估计轨迹折线宽度 |
| `estimated_marker_ns` | `magnet_est` | 估计姿态 marker namespace |
| `magnet_length` | 0.06 | 磁铁整体长度（米） |
| `magnet_radius` | 0.01 | 圆柱半径（米） |
| `marker_lifetime` | 0.0 | Marker 生命周期（秒，0 表示持续） |
| `strength_text_size` | 0.03 | 强度文本字号（米） |
| `strength_offset_scale` | 0.5 | 文本沿磁铁方向的偏移倍数 |
| `show_strength_text` | true | 是否显示强度文本 |
| `strength_label` | `magnet` | 文本前缀标签 |

节点会在私有命名空间内发布：

- `~markers` (`visualization_msgs/MarkerArray`)
- `~trail` (`nav_msgs/Path`)
- `~estimated_trail` (`nav_msgs/Path`，仅当启用估计姿态可视化时发布)

这样可以在同一运行图中并行启动多个磁铁可视化节点。

### 传感器阵列可视化参数

`config/sensor_array_viz.yaml` 控制 `sensor_array_viz_node`，常见字段：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `measurement_topic` | `/magnetic/sensor/field` | 传感器磁场话题 (`MagSensorData`) |
| `array_param` | `array` | `SensorArrayDescription` 在私有命名空间下的参数键（内部会访问 `~array/config`） |
| `vector_scale` | 0.004 | 每 mT 转换成箭头长度（米） |
| `max_arrow_length` | 0.10 | 箭头最大长度（米，<=0 表示无限制） |
| `arrow_origin_offset` | 0.0 | 箭头起点沿传感器法线的偏移量（米） |
| `measurement_timeout` | 0.3 | 超过该时间未更新则将箭头置为灰色 |
| `color_min_mT` / `color_max_mT` | 0.0 / 3.0 | 用于磁场强度着色映射的范围 |
| `show_text` | false | 是否显示每个传感器的磁场数值文本 |
| `text_scale` / `text_offset` | 0.02 / 0.015 | 文本大小与相对传感器的偏移 |

该节点在私有命名空间发布 `~sensor_markers` (`visualization_msgs/MarkerArray`)，仅包含磁场箭头与可选文本。
默认 launch 会把 `mag_core_description/config/sensor_array.yaml` 以 `ns="array"` 的形式加载到节点私有命名空间，因此需要保证自定义文件同样使用 `config:` 根结构，并通过 `<rosparam ns="array" ...>` 引入。
