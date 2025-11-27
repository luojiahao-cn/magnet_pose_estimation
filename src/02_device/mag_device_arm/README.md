# mag_device_arm

提供双臂及工具的控制/管理接口，对上层算法暴露标准化的服务或动作。

## 功能概览

- 基于 MoveIt 的机械臂控制封装，支持多机械臂并行管理。
- 通过服务接口执行末端位姿规划、执行 MoveIt Named Target。
- 可选发布机械臂基座的静态 TF，统一坐标系管理。
- 工具配置由 zlab_robots 通过 launch 参数管理，无需在包内配置。

## 配置示例

`config/single_arm.yaml`

- 多机械臂配置参考：`config/dual_arm.yaml`。

## 启动方式

### Gazebo 仿真模式

```bash
# 单臂仿真（默认无工具）
roslaunch mag_device_arm single_arm_gazebo.launch

# 单臂仿真（指定工具，如永久磁铁）
roslaunch mag_device_arm single_arm_gazebo.launch tool_name:=permanent_magnet

# 单臂仿真（指定工具，如磁传感器支架）
roslaunch mag_device_arm single_arm_gazebo.launch tool_name:=magnetic_sensor_bracket
```

### 真实机器人模式

首先启动 zlab_robots 的控制栈（例如 `connect_single_arm.launch`），然后启动：

```bash
# 单臂
roslaunch mag_device_arm single_arm.launch

# 双臂
roslaunch mag_device_arm dual_arm.launch
```

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

## 工具配置

工具配置由 zlab_robots 的 launch 文件通过 `tool_name` 参数管理，支持的工具有：
- `permanent_magnet`：永久磁铁
- `electronic_magnet`：电磁铁
- `magnetic_sensor_bracket`：磁传感器支架
- `none`：无工具（默认）

工具会在 MoveIt 的 URDF 加载时自动配置，无需在 `mag_device_arm` 包内进行额外配置。