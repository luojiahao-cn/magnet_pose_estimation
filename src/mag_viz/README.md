# mag_viz

磁传感器数据可视化（Mag Sensor Visualization）。

- 可执行：`mag_sensor_viz_node`
- 输入话题（可改）：`/mag_sensor/raw_data_mT`
- 输出 MarkerArray：`/mag_viz/markers`
- 默认不使用 TF（`frame: ""`），如需在某坐标系（如 `world`）下显示，启动时传 `frame:=world`。

启动：

```
roslaunch mag_viz mag_sensor_viz.launch
```

RViz 配置：`config/mag_sensor_viz.rviz`。旧的 `raw_viz.launch` 仍保留为兼容包装，会包含到新启动文件。
