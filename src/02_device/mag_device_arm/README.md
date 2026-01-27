# mag_device_arm

提供双臂及工具的控制/管理接口，对上层算法暴露标准化的服务或动作。

## 功能概览

- 基于 MoveIt 的机械臂控制封装，支持多机械臂并行管理。
- 通过服务接口执行末端位姿规划、执行 MoveIt Named Target。
- 可选发布机械臂基座的静态 TF，统一坐标系管理。
- 工具配置由 zlab_robots 通过 launch 参数管理，无需在包内配置。

## 配置示例

### 单臂配置

`config/single_arm.yaml` - 单臂配置，使用 `arm1` 作为机械臂名称，planning group 为 `fr5`。

**注意**：`end_effector_link` 需要根据实际使用的工具名称配置：
- 有工具：`{arm_id}_{tool_name}_tcp_link` 或 `{arm_id}_{tool_name}_link`
- 无工具：`{arm_id}_flange_link`

例如：
- `arm1_bracket_tcp_link`（使用磁传感器支架工具）
- `arm1_permanent_magnet_tcp_link`（使用永久磁铁工具）
- `arm1_flange_link`（无工具）

### 双臂配置

`config/dual_arm.yaml` - 双臂配置，使用 `arm1` 和 `arm2` 作为机械臂名称，planning group 分别为 `arm1` 和 `arm2`。

**注意**：配置中的 `end_effector_link` 需要与实际使用的工具匹配。如果更换工具，需要相应更新配置文件中的 `end_effector_link` 字段。

## 启动方式

### Gazebo 仿真模式

```bash
# 单臂仿真（默认工具：bracket）
roslaunch mag_device_arm fr5/single/fr5_single_gazebo.launch

# 单臂仿真（指定工具，如永久磁铁）
roslaunch mag_device_arm fr5/single/fr5_single_gazebo.launch tool_name:=permanent_magnet

# 单臂仿真（指定工具，如磁传感器支架）
roslaunch mag_device_arm fr5/single/fr5_single_gazebo.launch tool_name:=bracket

# 单臂仿真（指定 arm_id，默认为 arm1）
roslaunch mag_device_arm fr5/single/fr5_single_gazebo.launch arm_id:=arm1 tool_name:=bracket

# 单臂仿真（禁用 RViz）
roslaunch mag_device_arm fr5/single/fr5_single_gazebo.launch use_rviz:=false
```

### 真实机器人模式

启动文件会自动启动 zlab_robots 的硬件接口和 MoveIt 配置：

```bash
# 单臂（默认 arm1，默认 IP 根据 arm_id 自动选择）
roslaunch mag_device_arm fr5/single/fr5_single_bringup.launch

# 单臂（指定 IP 地址）
roslaunch mag_device_arm fr5/single/fr5_single_bringup.launch robot_ip:=192.168.31.202

# 单臂（指定 arm_id 和工具）
roslaunch mag_device_arm fr5/single/fr5_single_bringup.launch arm_id:=arm1 tool_name:=bracket

# 单臂（禁用 RViz）
roslaunch mag_device_arm fr5/single/fr5_single_bringup.launch use_rviz:=false
```

### MoveIt 仿真模式（无 Gazebo）

```bash
# 单臂 MoveIt 仿真（fake execution）
roslaunch mag_device_arm fr5/single/fr5_single_moveit.launch

# 单臂 MoveIt 仿真（指定工具）
roslaunch mag_device_arm fr5/single/fr5_single_moveit.launch tool_name:=bracket
```

### 双臂模式

```bash
# 双臂（默认 IP：arm1=192.168.31.201, arm2=192.168.31.202）
roslaunch mag_device_arm fr5/dual/fr5_dual_bringup.launch

# 双臂（指定 IP 和工具）
roslaunch mag_device_arm fr5/dual/fr5_dual_bringup.launch \
    arm1_ip:=192.168.31.201 \
    arm2_ip:=192.168.31.202 \
    arm1_tool:=permanent_magnet \
    arm2_tool:=bracket

# 双臂 Gazebo 仿真
roslaunch mag_device_arm fr5/dual/fr5_dual_gazebo.launch

# 双臂 Gazebo 仿真（指定工具）
roslaunch mag_device_arm fr5/dual/fr5_dual_gazebo.launch \
    arm1_tool:=permanent_magnet \
    arm2_tool:=bracket

# 双臂 Gazebo 仿真（禁用 RViz）
roslaunch mag_device_arm fr5/dual/fr5_dual_gazebo.launch use_rviz:=false

# 双臂 MoveIt 仿真（fake execution）
roslaunch mag_device_arm fr5/dual/fr5_dual_moveit.launch
```

## 示例：往复运动脚本

- `roslaunch mag_device_arm arm_ping_pong.launch`：启动单臂配置并运行 `scripts/arm_ping_pong.py`，机械臂会在 `pose_a`/`pose_b` 两个位姿之间往复。
- 通过 `arm`（默认为 `arm1`）、`velocity`、`acceleration`、`wait_time` 参数可快速调整目标机械臂和运动节奏；若需要自定义位姿，可在同一 launch 中重写 `pose_a`、`pose_b` 的 `xyz`/`rpy` 数组，或像默认示例一样以 `named_target: ready/up` 指定 MoveIt 的命名姿态。

## 服务接口

- `~/set_end_effector_pose` (`mag_device_arm/SetEndEffectorPose`)
	- 输入：`arm`、`target`、可选速度/加速度缩放、是否执行。
	- 功能：规划（可选执行）到指定姿态。
- `~/execute_named_target` (`mag_device_arm/ExecuteNamedTarget`)
	- 输入：`arm`、`target`（配置中的别名）、可选速度/加速度缩放、是否执行。
	- 功能：调用 MoveIt Named Target。
- `~/execute_cartesian_path` (`mag_device_arm/ExecuteCartesianPath`) **新增**
	- 输入：`arm`、`waypoints`（位姿数组）、`step_size`、`jump_threshold`、可选速度/加速度缩放、是否执行。
	- 功能：使用 MoveIt 的 `computeCartesianPath` 规划连续笛卡尔路径并执行，实现平滑连续运动。
	- 返回：`success`、`message`、`fraction`（路径规划完成度）、`trajectory`（规划的轨迹）。
	- 用途：适用于需要平滑连续跟踪的场景，如磁铁跟踪、轨迹跟踪等。

## 工具配置

工具配置由 zlab_robots 的 launch 文件通过 `tool_name` 参数管理，支持的工具有：
- `permanent_magnet`：永久磁铁
- `electronic_magnet`：电磁铁
- `bracket`：磁传感器支架
- `none`：无工具（默认）

工具会在 MoveIt 的 URDF 加载时自动配置，无需在 `mag_device_arm` 包内进行额外配置。