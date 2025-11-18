# 硬件配置说明

## 配置文件位置

硬件驱动配置文件位于：`hardware/config/sensor_driver.yaml`

## 配置步骤

1. **连接传感器阵列**
   - 确保传感器阵列已正确连接到计算机
   - 确认串口设备路径（通常为 `/dev/ttyUSB0` 或 `/dev/ttyACM0`）

2. **修改配置文件**
   - 编辑 `hardware/config/sensor_driver.yaml`
   - 修改 `serial_port` 为实际串口设备路径
   - 根据传感器规格调整 `baud_rate`、`full_scale_mT` 等参数

3. **更新 launch 文件中的配置路径**
   - 如果使用自定义配置文件，在 `hardware.launch` 中指定：
     ```bash
     roslaunch mag_sensor_fixture_app hardware.launch \
       sensor_hardware_config:=$(find mag_sensor_fixture_app)/hardware/config/sensor_driver.yaml
     ```

## 配置参数说明

- `serial_port`: 串口设备路径
- `baud_rate`: 串口波特率
- `timeout_ms`: 通信超时时间（毫秒）
- `poll_sleep_ms`: 轮询间隔（毫秒）
- `full_scale_mT`: 传感器满量程（毫特斯拉）
- `raw_max`: 原始数据最大值

