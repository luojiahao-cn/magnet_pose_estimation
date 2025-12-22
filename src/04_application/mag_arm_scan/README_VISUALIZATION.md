# 扫描数据可视化使用说明

## 功能概述

`scan_data_visualizer_node` 节点可以从 CSV 文件读取扫描数据，并将磁场向量发布到 ROS 话题，以便在 RViz 中可视化。

## 使用方法

### 1. 编译节点

首先确保已经编译了可视化节点：

```bash
cd ~/workshop/magnet_pose_estimation
catkin_make
source devel/setup.bash
```

### 2. 启动可视化节点

#### 方法 1：使用 launch 文件（推荐）

```bash
roslaunch mag_arm_scan visualize_scan_data.launch csv_file:="$(find mag_arm_scan)/data/save/scan_20251217_1710/scan_data.csv"
```

或者指定完整路径：

```bash
roslaunch mag_arm_scan visualize_scan_data.launch csv_file:="/home/lawkaho/workshop/magnet_pose_estimation/src/04_application/mag_arm_scan/data/save/scan_20251217_1710/scan_data.csv"
```

#### 方法 2：直接运行节点

```bash
rosrun mag_arm_scan scan_data_visualizer_node _csv_file:="路径/to/scan_data.csv" _frame_id:="world"
```

### 3. 在 RViz 中查看

启动节点后，RViz 会自动打开。如果没有自动打开，可以手动启动：

```bash
rviz
```

在 RViz 中添加以下显示项：

1. **添加 MarkerArray 显示**：
   - 点击 "Add" 按钮
   - 选择 "MarkerArray"
   - 设置 Topic 为 `/scan_data/markers`
   - 设置 Fixed Frame 为 `world`（或你使用的坐标系）

2. **（可选）添加 PointCloud2 显示**：
   - 如果启用了点云发布，可以添加 "PointCloud2" 显示
   - 设置 Topic 为 `/scan_data/pointcloud`

## 参数说明

### Launch 文件参数

- `csv_file`: CSV 文件路径（必须指定）
- `frame_id`: 坐标系名称（默认：`world`）
- `use_rviz`: 是否自动启动 RViz（默认：`true`）
- `marker_topic`: 标记话题名称（默认：`/scan_data/markers`）
- `pointcloud_topic`: 点云话题名称（默认：`/scan_data/pointcloud`）

### 节点参数（可通过 rosparam 设置）

- `csv_file`: CSV 文件路径
- `frame_id`: 坐标系名称
- `marker_topic`: 标记话题名称
- `pointcloud_topic`: 点云话题名称
- `arrow_length`: 箭头长度缩放因子（默认：0.02）
- `arrow_shaft_ratio`: 箭头杆直径比例（默认：0.1）
- `arrow_head_ratio`: 箭头头部直径比例（默认：0.25）
- `arrow_head_length_ratio`: 箭头头部长度比例（默认：0.3）
- `magnitude_min`: 磁场强度最小值，用于颜色映射（默认：0.0）
- `magnitude_max`: 磁场强度最大值，用于颜色映射（默认：5.0）
- `alpha`: 透明度（默认：1.0）
- `min_magnitude_threshold`: 最小磁场强度阈值，低于此值不显示（默认：0.001）
- `color_low`: 低强度颜色（RGB，默认：[0.0, 0.0, 1.0] 蓝色）
- `color_high`: 高强度颜色（RGB，默认：[1.0, 0.0, 0.0] 红色）
- `publish_rate`: 发布频率（Hz，默认：1.0）
- `publish_pointcloud`: 是否发布点云（默认：false）

## 可视化说明

- **箭头颜色**：根据磁场强度从蓝色（低强度）渐变到红色（高强度）
- **箭头方向**：表示磁场向量的方向
- **箭头长度**：与磁场强度成正比
- **数据分组**：按传感器 ID 分组，每个传感器显示一个平均磁场向量

## 故障排除

1. **看不到可视化**：
   - 检查 CSV 文件路径是否正确
   - 检查 RViz 中的 Fixed Frame 是否与 `frame_id` 参数一致
   - 检查话题名称是否正确（`/scan_data/markers`）

2. **数据加载失败**：
   - 检查 CSV 文件格式是否正确
   - 检查文件路径是否有读取权限

3. **编译错误**：
   - 确保已安装所有依赖包
   - 运行 `catkin_make` 查看详细错误信息

## 示例

完整的使用示例：

```bash
# 1. 编译
cd ~/workshop/magnet_pose_estimation
catkin_make
source devel/setup.bash

# 2. 启动可视化（会自动打开 RViz）
roslaunch mag_arm_scan visualize_scan_data.launch \
    csv_file:="$(find mag_arm_scan)/data/save/scan_20251217_1710/scan_data.csv" \
    frame_id:="world"
```

在 RViz 中，你应该能看到：
- 多个彩色箭头，表示不同传感器的磁场向量
- 箭头颜色从蓝色（弱磁场）到红色（强磁场）
- 箭头方向表示磁场方向

