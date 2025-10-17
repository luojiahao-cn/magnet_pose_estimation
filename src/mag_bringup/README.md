# mag_bringup

系统编排与集中配置（system.yaml）。一键拉起仿真/真机传感器、阵列 TF、可视化、估计器、扫描。

最小启动：
```bash
roslaunch mag_bringup bringup.launch
```

常用开关（见 `launch/bringup.launch`）：
- use_sim（默认 true）
- with_rviz（默认 true）
- start_estimator（默认 false）

关键配置（`config/system.yaml`）：
- frames.global_frame（默认 world）
- frames.parent_frame（默认 tool_tcp）
- frames.array_frame（默认 sensor_array）
