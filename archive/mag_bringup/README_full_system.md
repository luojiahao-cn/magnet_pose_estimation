# 完整磁场扫描系统启动说明

## 一键启动命令

```bash
roslaunch mag_bringup full_system.launch
```

## 包含组件

1. **MoveIt 机械臂控制环境** - Gazebo 仿真或实机连接
2. **工具管理器** - 管理传感器支架的附加/分离
3. **磁传感器仿真节点** - 模拟 25 个传感器的磁场数据（磁铁强度 10.0 mT）
4. **传感器 TF 发布器** - 发布传感器坐标系变换
5. **扫描控制器** - 控制机械臂自动扫描并记录数据
6. **实时可视化** - 在 RViz 中显示磁场矢量场（只显示到位时数据）
7. **RViz 界面** - 可视化界面

## 参数选项

- `use_gazebo` (默认: true) - 是否使用 Gazebo 仿真
- `use_attached_tool` (默认: true) - 是否附加工具支架
- `start_rviz` (默认: true) - 是否启动 RViz

## 输出文件

- `data/scan_data.csv` - 扫描记录的原始数据
- `data/online_samples.csv` - 实时可视化导出的样本

## 使用方法

1. 启动系统：`roslaunch mag_bringup full_system.launch`
2. 系统会自动开始扫描并实时显示磁场
3. 在 RViz 中查看磁场矢量箭头（每个扫描点显示 25 个传感器箭头）
4. 扫描完成后，数据保存在 CSV 文件中

## 注意事项

- 确保 Gazebo 和 RViz 已正确安装
- 扫描过程需要几分钟时间，请耐心等待
- 可视化只在机械臂到位时显示数据，运动过程中不显示