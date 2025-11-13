# mag_core_utils

提供磁场跟踪项目的公共工具库，供驱动与算法包复用。

## 参数读取工具

- `mag_core_utils::param::StructReader`：从参数服务器加载结构化配置，统一必选字段检查与错误上下文。
- `mag_core_utils::param::ArrayReader`：在数组场景下提供类型校验与索引检查，配合 `StructReader` 嵌套解析。
- 推荐在 node 中读取配置后，将解析结果传入业务组件（例如 `SensorArrayDescription`），避免在库内直接访问 `ros::NodeHandle`。
