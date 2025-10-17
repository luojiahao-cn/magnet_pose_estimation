# Magnet Pose Estimation (ROS)

一个面向磁传感器阵列的开源工程，提供“采集/仿真 → 坐标对齐（TF）→ 位姿估计 → 可视化/扫描”的完整流水线。

适配 ROS1（Noetic），默认使用 catkin 构建。工程包含以下功能包：

- mag_bringup：系统编排与集中配置（system.yaml）
- mag_sensor_node：传感器串口节点、仿真节点、阵列静态 TF 发布器
- mag_pose_estimation：磁体位姿估计器（优化/Kalman）
- mag_arm_scan：支架挂载/TF 发布（tool_manager）与扫描控制器
- mag_viz：磁场箭头可视化（RViz MarkerArray）

## 目录结构

```
magnet_pose_estimation/
	├── src/
	│   ├── mag_bringup/
	│   ├── mag_sensor_node/
	│   ├── mag_pose_estimation/
	│   ├── mag_arm_scan/
	│   └── mag_viz/
	└── CONVENTIONS.md   # 帧/话题的命名规范
```

## 依赖

- ROS Noetic（roscpp、tf2_ros、tf2_geometry_msgs、geometry_msgs 等）
- MoveIt（仅在使用 mag_arm_scan/tool_manager_node 时需要）
- Eigen3、Ceres（用于优化估计器）

## 安装与构建

1) 放入工作区并编译：

```bash
cd ~/catkin_ws/src
git clone <this-repo> magnet_pose_estimation
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2) 验证依赖（可选）：确保 MoveIt/机器人驱动可用（若需 tool_manager 或与机械臂联动）。

## 一键启动（仿真通路）

使用集中编排：

```bash
roslaunch mag_bringup bringup.launch use_sim:=true with_rviz:=false
```

默认流程：
- 启动仿真传感器（发布 /magnet_pose/ground_truth 与 /mag_sensor/data_mT）
- 发布阵列静态 TF：tool_tcp → sensor_array → sensor_<id>
- （可选）启动可视化与 RViz（with_rviz 控制）
- 估计器默认关闭（start_estimator:=false），按需开启

更多开关请查看 `src/mag_bringup/launch/bringup.launch`。

## 集中配置（system.yaml）

路径：`src/mag_bringup/config/system.yaml`

- frames
	- global_frame：全局坐标（估计器/可视化默认参考），默认 world
	- parent_frame：传感器阵列父坐标，默认 tool_tcp
	- array_frame：阵列逻辑帧名，默认 sensor_array
- estimator_config：估计器的全局帧、输入/输出话题
- viz_config：可视化默认订阅的话题

bringup 会在启动时自动 load 该文件，尽量减少跨包配置分散。

## TF 约定（关键）

完整链路：

```
frrobot_tool_link  →  tool_frame  →  tool_tcp  →  sensor_array  →  sensor_<id>
```

- tool_manager_node（mag_arm_scan）静态发布：
	- frrobot_tool_link → tool_frame
	- tool_frame → tool_tcp
- sensor_tf_publisher（mag_sensor_node）静态发布：
	- tool_tcp → sensor_array
	- sensor_array → sensor_<id>

默认不强行提供 world→base_link，避免引入不真实世界系；若需要在 world 下查看统一树，可在 bringup 里开启 `start_world_tf` 并配置该静态变换。

更详细的命名与话题规范见根目录 `CONVENTIONS.md`。

## 常用组合

- 只看仿真 + 可视化（不启估计器）：
	```bash
	roslaunch mag_bringup bringup.launch use_sim:=true start_estimator:=false with_rviz:=true
	```

- 真机传感器 + 可视化（需要先写好串口与阵列配置）：
	```bash
	roslaunch mag_bringup bringup.launch use_sim:=false start_sensor_tf:=true with_rviz:=true
	```

- 挂载支架（附着网格 + 发布 TF）：
	```bash
	roslaunch mag_arm_scan mag_arm_scan.launch use_gazebo:=false
	# tool_manager 自动 attach；失败也会发布 TF，保证链路连通
	```

## 故障排查（FAQ）

- RViz 报 world 外推/查询失败：
	- 将 Fixed Frame 改为 tool_tcp 或 sensor_array，或提供 world→base_link 静态 TF。
- TF 树不连贯：
	- 确认支架 TF（tool_frame、tool_tcp）已发布；本工程已采用静态发布且在 attach 失败时也会发布一次。
	- 确认传感器 TF 的 parent_frame 与 `tool_manager_node` 的 tcp_frame_name 一致（默认 tool_tcp）。
- 估计器 TF 报错：
	- 将估计器 global_frame 设为树上存在的帧（默认 world；无 world 时改为 tool_tcp）。

## 贡献与维护

欢迎 PR/Issue：
- 文档与 README（尤其真机接入/标定经验）
- 新的估计算法（例如更鲁棒的优化目标/约束）
- 仿真器与可视化改进（自定义 colormap、批量 marker 优化等）

## License

MIT

