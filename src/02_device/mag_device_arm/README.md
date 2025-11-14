# mag_device_arm

提供双臂及工具的控制/管理接口，对上层算法暴露标准化的服务或动作。

## 功能概览

- 基于 MoveIt 的机械臂控制封装，支持多机械臂并行管理。
- 通过服务接口执行末端位姿规划、执行 MoveIt Named Target。
- 可选发布机械臂基座与工具挂载点的静态 TF，统一坐标系管理。

## 配置示例

`config/single_arm.yaml`

```yaml
config:
	arms:
		- name: magnet
			group_name: fr5v6_arm
			end_effector_link: frrobot_tool_link
			reference_frame: world
			default_velocity: 0.1
			default_acceleration: 0.1
			planning_time: 5.0
			allow_replanning: false
			goal_position_tolerance: 0.005
			goal_orientation_tolerance: 0.01
			base_tf:
				parent: world
				child: magnet_arm_base
				pose:
					xyz: [0.0, 0.0, 0.0]
					rpy: [0.0, 0.0, 0.0]
			named_targets:
				- name: ready
					 target: ready
				- name: up
						 target: up
```

- 多机械臂配置参考：`config/dual_arm.yaml`。
- 默认示例：`roslaunch mag_device_arm single_arm.launch`。
- 工具挂载可选示例：
	- 单臂（裸端法兰）：`roslaunch mag_device_arm single_arm.launch`（使用 `config/single_arm.yaml`）
	- 单臂（默认带磁传感器支架）：`roslaunch mag_device_arm single_arm_tool.launch`（使用 `config/single_arm_tool.yaml`，其中 `tool_mount.options` 描述 mesh/TF/缩放）
	- 若需进一步自定义，可复制 YAML 后通过 `single_arm.launch config:=<your_yaml>` 指定
	- 双臂：`roslaunch mag_device_arm dual_arm_tool.launch magnet_tool:=none sensor_tool:=magnetic_sensor_bracket`
- `tool_mount.selected` 可以在 launch 或 YAML 中调整，以选择具体安装在某条机械臂上的工具选项；节点会自动发布对应的静态 TF，并在 MoveIt 规划场景中附着对应 mesh。

## 示例：往复运动脚本

- `roslaunch mag_device_arm arm_ping_pong.launch`：启动单臂配置并运行 `scripts/arm_ping_pong.py`，机械臂会在 `pose_a`/`pose_b` 两个位姿之间往复。
- 通过 `arm`、`velocity`、`acceleration`、`wait_time` 参数可快速调整目标机械臂和运动节奏；若需要自定义位姿，可在同一 launch 中重写 `pose_a`、`pose_b` 的 `xyz`/`rpy` 数组，或像默认示例一样以 `named_target: ready/up` 指定 MoveIt 的命名姿态。

## 服务接口

- `~/set_end_effector_pose` (`mag_device_arm/SetEndEffectorPose`)
	- 输入：`arm`、`target`、可选速度/加速度缩放、是否执行。
	- 功能：规划（可选执行）到指定姿态。
- `~/execute_named_target` (`mag_device_arm/ExecuteNamedTarget`)
	- 输入：`arm`、`target`（配置中的别名）、可选速度/加速度缩放、是否执行。
	- 功能：调用 MoveIt Named Target。

调用前需启动对应的 MoveIt 控制/驱动节点（例如 `zlab_robots` 项目中的控制栈）。
