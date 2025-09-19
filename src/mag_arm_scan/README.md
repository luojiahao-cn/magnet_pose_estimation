# mag_arm_scan

机械臂磁场扫描协调包：负责生成扫描路径/网格，发布航点，并与 `mag_sensor_node` 配合完成空间初始磁场扫描。

## 功能
- 生成 3D 网格扫描点（体素/层扫描）
- 发布 `geometry_msgs/PoseArray` 作为航点序列（topic: `/mag_arm_scan/scan_waypoints`）
- 提供服务开始/停止扫描（`/mag_arm_scan/start`, `/mag_arm_scan/stop`）
- 预留接口：对接机械臂控制栈（MoveIt!/厂商SDK）与采样触发

## 运行
1. 构建：在工作区根执行 catkin_make
2. 启动：`roslaunch mag_arm_scan mag_arm_scan.launch`
3. 调用服务：`rosservice call /mag_arm_scan/start` 生成并发布航点

## 参数（~ 私有命名空间）
- `volume_min` [x,y,z] (m)：扫描体对角点最小值
- `volume_max` [x,y,z] (m)：扫描体对角点最大值
- `step` [dx,dy,dz] (m)：步进
- `yaw` (rad)：末端朝向（绕Z）
- `frame_id`：航点坐标系
- `autostart` (bool)：节点启动后是否自动发布一次

## 话题
- `/mag_arm_scan/scan_waypoints` (geometry_msgs/PoseArray)

## 服务
- `/mag_arm_scan/start` (std_srvs/Empty)
- `/mag_arm_scan/stop` (std_srvs/Empty)

备注：当前包不直接控制机械臂，仅负责规划与协调。可在后续添加控制器节点对接 MoveIt! 执行航点与 `mag_sensor_node` 数据采样节拍。
