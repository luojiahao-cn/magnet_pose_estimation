# mag_device_magnet

负责磁铁（目标物）运动控制与轨迹仿真，实现磁铁 TF 和真值数据的发布。

## 快速使用

- 示例配置：`config/static.yaml`（静止）、`config/circular.yaml`（圆轨迹）。
- 启动方式：
	- 静态：`roslaunch mag_device_magnet static.launch`
	- 圆轨迹：`roslaunch mag_device_magnet circular.launch`
- 发布接口：
	- TF：`frame.child` 相对于 `frame.parent`。
	- 话题：`/magnetic/pose_true` (`mag_core_msgs/MagnetPose`)。
