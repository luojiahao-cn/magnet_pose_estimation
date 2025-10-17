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
