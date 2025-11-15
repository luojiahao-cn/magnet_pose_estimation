# mag_viz

负责磁场、姿态、轨迹等可视化展示，提供 RViz 配置与可视化节点。

## 功能

- 订阅 `mag_core_msgs/MagnetPose` 真值姿态，生成红/蓝半圆柱与磁场强度文本标记。
- 维护可配置的轨迹队列，同时发布 `nav_msgs/Path` 与折线 `Marker`。
- 提供一键启动的 RViz 视图（包含 TF、Marker Array、Path）。

## 快速开始

```bash
roslaunch mag_viz magnet_viz.launch
```

默认会加载 `config/magnet_viz.yaml` 并打开 `rviz/magnet_viz.rviz`。若只想运行节点可关闭 `use_rviz`：

```bash
roslaunch mag_viz magnet_viz.launch use_rviz:=false
```

常用参数位于 `config/magnet_viz.yaml`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `pose_topic` | `/magnetic/pose_true` | 输入真值姿态话题 |
| `frame_id` | `world` | 未提供 frame_id 时的回退 |
| `trail_size` | 600 | 轨迹缓存上限（数量） |
| `trail_duration` | 45.0 | 轨迹保留时长（秒，<=0 表示无限） |
| `trail_min_distance` | 0.001 | 相邻样本的最小距离过滤 |
| `trail_width` | 0.003 | 轨迹折线宽度 |
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

这样可以在同一运行图中并行启动多个磁铁可视化节点。
