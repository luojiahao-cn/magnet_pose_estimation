# magnetic_pose_estimation

该包提供磁场位姿估计算法与节点，包含以下组件：

- 库：`magnetic_field_calculator`
- 节点：`magnet_pose_estimator_node`
  - 实现：`OptimizationMagnetPoseEstimator` / `KalmanMagnetPoseEstimator`

依赖：Eigen3、Ceres、tf2、roscpp、geometry_msgs、sensor_msgs、std_srvs。

消息：不再在本包内定义。统一使用 mag_sensor_node 包中的 `MagSensorData.msg` 与 `MagnetPose.msg`。

构建：`catkin_make`
