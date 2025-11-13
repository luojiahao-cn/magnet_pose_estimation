# mag_arm_scan

面向机械臂的磁场扫描与可视化。

包含两个节点：
- tool_manager_node：发布末端 TF（frrobot_tool_link→tool_frame→tool_tcp），可选择附着支架网格。
- scan_controller_with_recorder_node：按三维网格移动机械臂，每点采集 25 传感器的多帧数据求均值；写入 CSV；在 RViz 以箭头持久叠加可视化磁场。

快速启动
```bash
roslaunch mag_arm_scan mag_arm_scan.launch
```

主要参数（见 config/scan_controller_node.yaml）
- frame_id：可视化与数据统一到的世界坐标（如 world）
- volume_min / volume_max / step：扫描体积与步长（米）
- mag_topic：传感器数据话题（mag_sensor_node/MagSensorData）
- frames_per_sensor：每传感器每点采集帧数（默认 10）
- required_num_sensors：期望的传感器数量（默认 25；0 表示不强制）
- visualization.*：RViz Arrow 的话题、长度缩放、颜色映射等

话题与服务
- 订阅：~mag_topic（mag_sensor_node/MagSensorData）
- 发布：/scan_at_position（std_msgs/Bool），/scan_complete（std_msgs/String），~visualization/topic（visualization_msgs/MarkerArray）
- 服务：~start_scan（std_srvs/Trigger）

CSV 输出（逐样本）
```
timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,sensor_id,frame_id
```
pos/orientation 为该样本对应传感器在 frame_id 下的位姿，便于后处理与复现。

可视化（RViz）
- 添加 MarkerArray 显示器，订阅 topic：`/mag_field_vectors`（可在 YAML 里改）
- 箭头长度 = |B| × vector_scale，颜色按 magnitude_min/max 线性插值（默认蓝→红）
- vectors 持久叠加，完整展示已扫描区域磁场环境

注意
- 本节点依赖 MoveIt 规划与执行；执行完成后会调用 stop，并按 wait_time 进行短暂停顿。
- 稳定性阈值判据已移除，逻辑更简单直接；若需要可在未来再按开关恢复。
