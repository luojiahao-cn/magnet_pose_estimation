# mag_core_description

统一维护传感器阵列几何描述与 TF 发布辅助工具，供仿真与实机共用。

## 参数加载约定

- 使用 `mag_core_utils::param::StructReader` 解析 `~array` 配置，统一字段校验与错误提示。
- 业务侧通过 `SensorArrayDescription::load` 接收已标准化的参数格式，确保仿真/实机共用同一套 YAML。
- 可在 node 的私有命名空间下直接加载：`rosparam load config/sensor_array.yaml`。
- 与运行时相关的参数（如 TF 发布频率）应在各自节点的配置中维护，`SensorArrayDescription` 仅关心几何结构。
