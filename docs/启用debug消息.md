# 如何启用 ROS Debug 消息

## 方法 1：使用环境变量（推荐）

在启动 launch 文件之前，设置环境变量：

```bash
export ROSCONSOLE_MIN_SEVERITY=DEBUG
roslaunch mag_sensor_fixture_app simulation.launch
```

或者一行命令：

```bash
ROSCONSOLE_MIN_SEVERITY=DEBUG roslaunch mag_sensor_fixture_app simulation.launch
```

## 方法 2：在 launch 文件中设置

在 `simulation.launch` 文件中添加：

```xml
<env name="ROSCONSOLE_MIN_SEVERITY" value="DEBUG"/>
```

## 方法 3：使用 rosparam 设置（运行时）

在节点运行后，可以通过 rosparam 设置：

```bash
rosparam set /rosconsole/min_severity DEBUG
```

## 方法 4：使用 rqt_console 查看所有消息

```bash
rqt_console
```

这会显示所有级别的消息（包括 DEBUG），可以过滤和查看。

## 日志级别说明

- `DEBUG` - 调试信息（最详细）
- `INFO` - 一般信息
- `WARN` - 警告信息
- `ERROR` - 错误信息
- `FATAL` - 致命错误

## 当前代码中的 DEBUG 消息

1. **optimizer_estimator.cpp**:
   - 优化成功后的估计结果（位置、方向、成本）

2. **mag_pose_estimator_node.cpp**:
   - TF 查询使用最新时间（时间戳太旧时）
   - 批量优化成功信息
   - 传感器数量不足的提示

## 示例：查看特定节点的 DEBUG 消息

```bash
# 只查看 mag_pose_estimator 节点的 DEBUG 消息
ROSCONSOLE_MIN_SEVERITY=DEBUG roslaunch mag_sensor_fixture_app simulation.launch 2>&1 | grep -E "(DEBUG|optimizer_estimator|mag_pose_estimator)"
```

