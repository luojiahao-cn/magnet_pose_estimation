# 批量传感器磁场数据消息
# 用于将多个传感器的测量数据打包成一批，减少消息数量并确保时间同步
#
# 所有磁场数据已在统一坐标系中（header.frame_id），订阅者可直接使用
# 磁场单位：毫特斯拉（mT）
# 注意：传感器位置可通过 sensor_id 从配置或 TF 查询获取

std_msgs/Header header
# header.frame_id: 所有传感器数据的参考坐标系（通常是传感器阵列坐标系）
# header.stamp: 批量数据的时间戳（通常是所有传感器数据的最新时间戳）

# 传感器数据数组（直接使用 MagSensorData）
MagSensorData[] measurements
