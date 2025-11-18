# mag_core_description

统一维护传感器阵列几何描述与 TF 发布辅助工具，供仿真与实机共用。

## 参数加载约定

- 所有 YAML 以 `config:` 作为根节点，`config.frames.*` 与 `config.sensors[]` 分别描述阵列坐标系及传感器列表。
- Launch 文件需在节点私有命名空间下执行 `rosparam` 并指定 `ns="array"`，这样 `~array/config/...` 就映射到阵列配置，避免与其它 `config` 冲突。
- Node 端使用 `rosparam_shortcuts::get(pnh, "array/config", XmlRpcValue&)` 读取原始结构，再调用 `mag_core_description::loadSensorArrayConfig` 与 `SensorArrayDescription::load` 构建运行时对象。
- `SensorArrayDescription` 只负责几何信息；与运行时相关的发布频率、可视化样式等继续由各自包维护。
