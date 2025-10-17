# Project Conventions

This repo standardizes frames and topics for clarity and interoperability.

Frames (TF)
- world: Global fixed frame (Z-up).
- base_link: Robot base frame.
- tool_frame: Tool mount frame on the end-effector.
- tool_tcp: Tool center point (TCP), child of tool_frame.
- sensor_array: Logical sensor array frame, child of tool_tcp (via array_offset).
- sensor_<id>: Each sensor local frame, child of sensor_array.

Topics
- /mag_sensor/data: Raw sensor counts (MagSensorData), frame_id = tool_tcp.
- /mag_sensor/data_mT: Converted milliTesla data (MagSensorData), frame_id = tool_tcp.
- /magnet_pose/estimated: Estimated magnet pose (MagnetPose), frame_id = world.
- /magnet_pose/ground_truth: Simulator ground-truth pose (MagnetPose), frame_id = sim_config/frame_id.

Parameters
- estimator_config/global_frame: Defaults to world.
- estimator_config/mag_topic: Defaults to /mag_sensor/data_mT.
- estimator_config/output_topic: Defaults to /magnet_pose/estimated.
- mag_sensor_node/~frame_id: Defaults to tool_tcp.
- sensor_config.yaml/array_offset: position + orientation (RPY rad).

Launch order
1) Publish static TFs for sensor array (sensor_tf_publisher.launch).
2) Start sensor node (or simulator).
3) Start estimator and visualization.
