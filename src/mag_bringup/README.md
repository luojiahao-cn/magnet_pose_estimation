# mag_bringup

系统级编排包：一键启动磁传感器（真机或仿真）、可视化、估计器，以及可选扫描协调。

## 使用

- 仿真（默认）、可视化、估计器：
```bash
roslaunch mag_bringup bringup.launch
```

- 在无图形环境下仍想运行可视化（不启动 RViz）：
```bash
roslaunch mag_bringup bringup.launch with_rviz:=false
```

- 使用真机传感器：
```bash
roslaunch mag_bringup bringup.launch use_sim:=false
```

- 启动扫描协调：
```bash
roslaunch mag_bringup bringup.launch start_scan:=true
```

## 参数
- use_sim (bool, default=true): 使用仿真节点，否则使用串口节点
- with_viz (bool, default=true): 启动可视化节点
- with_rviz (bool, default=false): 是否顺带启动 RViz（headless 环境建议 false）
- start_scan (bool, default=false): 是否启动扫描协调器
- sensor_topic_raw (string): 原始话题名（未用作默认接入）
- sensor_topic_mT (string): 工程单位话题名，默认接入到可视化
- viz_marker_topic (string): MarkerArray 输出话题

## 备注
- 你之前遇到的 "XmlRpcClient::writeRequest: write error (拒绝连接)" 多为无图形/远程环境启动 RViz 导致。将 with_rviz 设为 false 即可规避。
- 本包仅做编排，不包含代码；launch 会安装到包的 share/ 下，方便 roslaunch 查找。

---

## bringup.launch（每节点启动开关）
你也可以使用 `bringup.launch`，并通过参数独立开关各节点：

可用参数：
- start_sensor (bool, default=true): 是否启动传感器（与 use_sim 配合）
- use_sim (bool, default=false): 使用仿真或真机串口
- start_viz (bool, default=with_viz): 是否启动可视化节点
- start_rviz (bool, default=with_rviz): 是否随可视化启动 RViz
- start_estimator (bool, default=true): 是否启动位姿估计器
- start_scan (bool, default=false): 是否启动扫描协调器
- rviz_config (string): 透传到 `mag_viz/mag_sensor_viz.launch` 的 RViz 配置文件
- sensor_topic_mT, viz_marker_topic: 可视化的输入与输出话题

示例：
```bash
roslaunch mag_bringup bringup.launch start_sensor:=true use_sim:=true start_viz:=true start_rviz:=false start_estimator:=true start_scan:=false
```
