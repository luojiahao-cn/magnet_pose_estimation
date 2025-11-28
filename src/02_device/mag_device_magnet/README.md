# mag_device_magnet

负责磁铁（目标物）运动控制与轨迹仿真，实现磁铁 TF 和真值数据的发布。

## 快速使用

- 示例配置：
	- `config/static.yaml`：静止姿态
	- `config/circular.yaml`：圆轨迹（保持固定姿态）
	- `config/rectangular.yaml`：参考 `archive/magnet_motion` 的矩形扫描 + 自旋姿态
- 启动方式：
	- 静态：`roslaunch mag_device_magnet static.launch`
	- 圆轨迹：`roslaunch mag_device_magnet circular.launch`
	- 矩形扫描：`roslaunch mag_device_magnet rectangular.launch`
- 发布接口：
	- TF：`frame.child` 相对于 `frame.parent`（可配置为静态/动态）。
	- 话题：`/magnetic/pose_true` (`mag_core_msgs/MagnetPose`)。

## 配置要点

- `trajectory.type` 现支持 `static` / `circular` / `rectangular`：
	- `rectangular` 需要 `width`、`height`、`center_xyz`（包含 x, y, z）与 `velocity`，会以恒速沿矩形边缘运动，z 坐标由 `center_xyz` 的 z 分量指定。
- `orientation` 可选控制姿态：
	- `mode: trajectory`（默认）直接使用轨迹内置姿态（静态姿态或 `orientation_rpy`）。
	- `mode: fixed` 搭配 `rpy: [roll,pitch,yaw]` 固定姿态。
	- `mode: spin` 搭配 `initial_rpy`、`axis`（如 `xyz`）与 `angular_velocity`，即可实现与 `archive/magnet_motion` 相同的自转效果。
