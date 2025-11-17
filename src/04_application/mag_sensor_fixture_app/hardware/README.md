# Hardware configs

`sensor_driver.yaml` 是 `mag_device_sensor` 驱动节点的模板：

- 更新 `driver.serial_port` 以匹配当前串口；
- 根据阵列校准结果调节 `topics` 与 `tf.publish_rate`；
- 保持 `raw` 与 `field` 话题命名与 `mag_pose_estimator` 配置一致。

如需区分多套阵列，可复制该文件并在 `launch/hardware.launch` 中通过 `sensor_driver_config` 参数指定。