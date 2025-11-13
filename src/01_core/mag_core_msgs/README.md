# mag_core_msgs

共享消息、服务、动作定义。此包只负责接口描述，不包含可执行节点。

## 已提供接口

- `MagSensorData.msg`：单个磁传感器的三轴磁场测量（单位：mT）。
- `MagSensorArray.msg`：阵列级快照，包含阵列姿态及多传感器数据。
- `MagnetPose.msg`：磁铁估计姿态与磁强度。
- `TrackMagnet.action`：用于开启/关闭磁铁追踪的动作接口。
