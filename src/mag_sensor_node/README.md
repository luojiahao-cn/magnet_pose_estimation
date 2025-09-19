## mag_sensor_node

高频磁传感器数据串口采集与 ROS 发布包。

## 功能概览
* 串口读取磁传感器输出行，格式示例：`[01]: 123 -456 789`
* 解析并发布自定义消息 `mag_sensor_node/mag_sensor_data`
* 自动加载参数服务器上的传感器阵列配置（`sensor_config.yaml`）并附带每个传感器的位姿 `sensor_pose`
* 发布话题（默认）：`/magnetic_field/raw_data` (原始计数/原始单位)
* 发布第二话题：`/magnetic_field/raw_data_mT` (转换后的毫特斯拉值)
* 周期性输出发布频率统计（参数可调）

## 自定义消息 `mag_sensor_data`
```
Header header        # frame_id = 配置/参数指定的世界或阵列坐标系
uint32 sensor_id     # 传感器 ID (来自串口行 S<ID>: ...)
float64 mag_x        # X 分量（原始或工程单位，取决于传感器）
float64 mag_y
float64 mag_z
geometry_msgs/Pose sensor_pose  # 该传感器在阵列坐标系下的位姿
```

## 运行依赖
* ROS (roscpp, std_msgs, geometry_msgs, message_runtime)
* `serial` 库
* 参数文件：`serial_config.yaml`（串口相关参数，可选）与 `sensor_config.yaml`（传感器阵列描述）

## 主要参数（私有命名空间 ~ 下）
| 参数名 | 类型 | 默认值 | 说明 |
| ------ | ---- | ------ | ---- |
| port | string | /dev/ttyUSB0 | 串口设备路径 |
| baud_rate | int | 921600 | 串口波特率 |
| timeout | int(ms) | 1000 | 串口超时，用于 `serial::Timeout` |
| topic | string | /magnetic_field/raw_data | 原始数据发布话题名称 |
| frame_id | string | world | 消息 header.frame_id |
| sleep_ms | int | 1 | 主循环空转睡眠 | 
| freq_stat_period | double | 5.0 | 发布频率统计窗口（秒） |

启动时还会从全局命名空间读取（若存在）：
* `sensors`（见配置说明）

## 传感器配置 (参数服务器示例)
```yaml
sensors:
    - id: 1
        pose:
            position: {x: 0.01, y: 0.00, z: 0.00}
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    - id: 2
        pose: { position: {x: 0.02, y: 0.00, z: 0.00}, orientation: {x: 0, y:0, z:0, w:1} }
```

## 构建
```bash
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

## 启动
```bash
roslaunch mag_sensor_node mag_sensor_node.launch \
    port:=/dev/ttyUSB0 baud_rate:=921600 frame_id:=world topic:=/magnetic_field/raw_data
```
或直接运行可执行：
```bash
rosrun mag_sensor_node mag_sensor_node _port:=/dev/ttyUSB0 _baud_rate:=921600
```

## 订阅示例（C++）
```cpp
ros::Subscriber sub = nh.subscribe<mag_sensor_node::mag_sensor_data>(
        "/magnetic_field/raw_data", 50,
        [](const mag_sensor_node::mag_sensor_data::ConstPtr& msg){
                ROS_INFO_STREAM("mag xyz=" << msg->mag_x << "," << msg->mag_y << "," << msg->mag_z);
        });
```

## 串口数据格式
一行一个数据包：`[NN]: v1 v2 v3` 例如：`[03]: 123 -456 789`
* `NN` 为两位（或多位）数字的传感器 ID（前导 0 可选）
* v1 v2 v3 为原始整数（或可解析为 double 的数值），节点在 `/raw_data` 保留原值，在 `/raw_data_mT` 中转换为毫特斯拉
* 转换公式：`mT = (raw / 32767.0) * 3.2`

## 运行时日志/频率
默认每 `freq_stat_period` 秒输出一次频率日志；当前实现中频率输出公式包含一个 `/25`（根据原始采集速率假设），如与真实速率不符，可在后续版本移除或改为自适应。

## 常见问题排查
| 问题 | 现象 | 处理 |
| ---- | ---- | ---- |
| 权限不足 | 打开串口失败 | 将用户加入 dialout 组：`sudo usermod -a -G dialout $USER` 后重新登录 |
| 无数据 | 没有任何发布 | 检查串口是否有输出：`sudo cat /dev/ttyUSB0` |
| 频率偏低 | Hz 日志不准 | 调整或移除 `/25` 频率缩放因子 |
| 解析异常 | ROS_WARN_THROTTLE 提示解析错误 | 确认串口行格式是否严格 `[ID]:` 开头 |

## 已知限制 / 规划
* 暂无自动重连机制（串口断开后需要重启节点）
* 未提供诊断整合（可接入 `diagnostic_updater`）
* 频率统计简单，可改用滑动窗口或 EMA
* `/raw_data_mT` 目前复用同一消息类型，仅数值已转为毫特斯拉；如需区分可新增带单位字段的新消息类型
* 可添加参数验证与更严格异常处理

## License
MIT

