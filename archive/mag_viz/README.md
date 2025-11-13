# mag_viz

磁传感器数据可视化（Mag Sensor Visualization）。支持在源帧或目标帧（通过 TF2 转换）下绘制磁场箭头。

- 可执行：`mag_sensor_viz_node`
- 输入话题（可改）：`/mag_sensor/data_mT`
- 输出 MarkerArray：`/mag_viz/markers`
- 目标帧 `~frame`：默认空（在消息 `header.frame_id` 下绘制）。若留空且集中参数 `/frames/global_frame` 存在，则回退使用该全局帧。

启动：

```
roslaunch mag_viz mag_sensor_viz.launch
```

RViz 配置：`config/mag_sensor_viz.rviz`。旧的 `raw_viz.launch` 仍保留为兼容包装，会包含到新启动文件。
