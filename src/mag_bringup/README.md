# mag_bringup

系统编排与一键拉起仿真/真机传感器、阵列 TF、可视化、估计器、扫描。

最小启动：
```bash
roslaunch mag_bringup bringup.launch
```

常用开关（见 `launch/bringup.launch`）：
- use_sim（默认 true）
- with_rviz（默认 true）
- start_estimator（默认 false）

配置策略：不再使用集中式 `system.yaml`。每个节点在其 `<node>` 内部通过 `<rosparam>` 加载私有 YAML；公共的传感器阵列配置复用 `mag_sensor_node/config/sensor_array.yaml`。
