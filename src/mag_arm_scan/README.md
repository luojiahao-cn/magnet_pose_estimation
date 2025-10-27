# mag_arm_scan

机械臂扫描与工具支架管理（最小必需信息）。

核心：
- tool_manager_node：静态发布末端 TF（frrobot_tool_link→tool_frame→tool_tcp），可选择把支架网格附着到末端。
- scan_controller_node（可选）：按网格路径扫描并采样 `/mag_sensor/data_mT`。

最小启动：
```bash
roslaunch mag_arm_scan mag_arm_scan.launch
```

关键参数（~）：
- parent_link（默认 auto，自动取 MoveIt 末端）
- xyz/rpy（tool_frame 相对 parent_link）
- tip_xyz/tip_rpy（tool_tcp 相对 tool_frame）
- tcp_frame_name（默认 tool_tcp）

TF 约定：
```
frrobot_tool_link → tool_frame → tool_tcp
tool_tcp → sensor_array → sensor_<id>  # 由 mag_sensor_node 发布
```

配置与输出说明
----------------

- 在 `config/scan_controller_node.yaml` 中可以设置下面两个新参数来帮助确保每个扫描点采集到固定数量的传感器：
	- `required_num_sensors`：整数，设为 0 表示不强制检测；设置为 25 表示期望每点至少能采集到 25 个不同 `sensor_id`（默认在示例配置中已设置为 25）。
	- `sensor_detect_time`：浮点数，检测这些传感器 id 的超时时间（秒），默认示例为 10.0s。

- 输出 CSV 已扩展包含姿态和来源信息，header 如下：

	timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,sensor_id,frame_id

	这有助于在后处理或可视化时，直接将磁场向量与传感器在世界坐标系下的位置与朝向对应起来。

使用示例：在启动节点的 launch 或参数文件中设置 `required_num_sensors: 25`，并确保至少 25 个传感器在扫描开始前发布过数据（可通过 `sensor_detect_time` 调整检测时长）。
