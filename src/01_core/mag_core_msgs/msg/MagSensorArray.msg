# 批量传感器磁场数据消息 (SoA 格式，用于高效传输)
std_msgs/Header header

# 传感器 ID 列表
uint32[] sensor_ids

# 磁场数据列表 (x, y, z 分别存储)
float64[] mag_x
float64[] mag_y
float64[] mag_z
