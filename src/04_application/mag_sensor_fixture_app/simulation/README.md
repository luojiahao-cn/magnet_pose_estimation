# Simulation configs

- `config/magnet_motion.yaml`：`mag_device_magnet` 的参数，指定磁铁在 `sensor_array/base` 下的轨迹与 TF。
- `config/sensor_sim.yaml`：`mag_device_sensor` 仿真节点的参数，引用相同的磁铁 TF 并生成 `/magnetic/sensor/field`。

在 `launch/simulation.launch` 中可通过 `magnet_config`、`sensor_sim_config` 参数覆盖默认路径，实现不同轨迹或噪声模型。