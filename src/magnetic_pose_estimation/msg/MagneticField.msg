# This file is part of the magnetic_pose_estimation package

Header header

#传感器ID
uint32 sensor_id

# 磁场数据 (单位: 特斯拉/Tesla)
float64 mag_x
float64 mag_y
float64 mag_z

# 传感器在世界坐标系中的位姿
geometry_msgs/Pose sensor_pose