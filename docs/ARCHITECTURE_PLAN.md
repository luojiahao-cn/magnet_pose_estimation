# 磁场跟踪系统包结构规划

## 项目定位

- **目标**：构建基于磁场检测的双臂协同跟踪系统，实现磁传感器阵列与磁铁之间的闭环追踪。
- **总体结构**：采用分层架构，将公共接口、硬件与仿真、算法估计、应用编排、工具测试等模块化拆分，降低耦合、方便协作。

## 命名与分层原则

- **命名规则**：使用 `mag_<domain>_<role>` 形式，保持语义清晰、层次分明，同时保证命名简洁。
- **层级关系**：
  1. 基础层（Core Layer）
  2. 设备与仿真层（Device Layer）
  3. 算法与估计层（Algorithm Layer）
  4. 应用编排层（Application Layer）
  5. 工具与测试层（Tools & Test）

- **依赖约束**：
  - 基础层无上层依赖。
  - 设备层只依赖基础层。
  - 算法层依赖基础层与设备层的公开接口。
  - 应用层通过话题/服务/动作与下层交互，不直接访问内部库。
  - 工具层按需依赖各层模块，主要用于运维与测试。

## 仓库目录结构建议

```
magnet_pose_estimation/
├── CMakeLists.txt
├── README.md
├── docs/
│   ├── ARCHITECTURE_PLAN.md
│   ├── design/
│   ├── calibration/
│   ├── experiments/
│   └── user_guides/
├── scripts/
├── tools/
├── data/
│   ├── logs/
│   └── calibration/
├── config_shared/
├── archive/
├── tests/
└── src/
  ├── 01_core/
  │   ├── mag_core_msgs/
  │   ├── mag_core_utils/
  │   └── mag_core_description/
  ├── 02_device/
  │   ├── mag_device_sensor/
  │   ├── mag_device_magnet/
  │   └── mag_device_arm/
  ├── 03_algorithm/
  │   ├── mag_pose_estimator/
  │   ├── mag_tracking_control/
  │   └── mag_tracking_calib/
  ├── 04_application/
  │   ├── mag_tracking_bringup/
  │   ├── mag_viz/
  │   └── mag_tracking_scan/
  └── 05_tools/
    ├── mag_tools_analysis/
    └── mag_tools_sim/
```

- `docs/`：集中设计方案、实验报告、用户指南等文档，`generated/` 下可放自动化生成的 API 文档。
- `scripts/`：仓库级命令行脚本，如一键仿真、CI 入口。
- `tools/`：非 ROS 包的辅助程序（如数据分析、notebook）。
- `data/`：录制数据、标定结果、示例日志，按日期或实验编号建子目录。
- `config_shared/`：跨包共享的参数模板（例如 MoveIt、仿真全局配置）。
- `archive/`：封存旧代码或暂存模块，需在 README 中标注来源与弃用时间。
- `tests/`：顶层集成或自动化测试脚本，各子包内仍可保留自有 `tests/` 目录。
- `src/`：按层级划分 ROS 包子目录，并使用数字前缀确保在文件浏览器中遵循“基础→上层”的顺序。

## 各层包职责

### 1. 基础层（Core Layer）

| 包名 | 职责 | 说明 |
| --- | --- | --- |
| `mag_core_msgs` | 定义消息、服务、动作接口 | 现有 `magnet_msgs` 的进一步规范化；禁止其他包自带消息定义。 |
| `mag_core_utils` | 公共工具库（C++/Python） | 包含参数解析、TF 辅助、矩阵运算等通用模块，供驱动与算法复用。 |
| `mag_core_description` | 阵列/传感器描述与 TF 发布 | 从 `mag_sensor_node` 中拆出 `SensorArrayDescription`，提供统一几何/TF 接口。 |

### 2. 设备与仿真层（Device Layer）

| 包名 | 职责 | 接口 |
| --- | --- | --- |
| `mag_device_sensor` | 传感器硬件驱动与仿真节点 | 订阅 `~config/*`，发布 `/magnetic/sensor/raw`、`/magnetic/sensor/field`。 |
| `mag_device_magnet` | 磁铁（目标）运动与仿真 | 发布磁铁 TF、真值姿态，加载自身轨迹/运动配置。 |
| `mag_device_arm` | 双臂控制适配（MoveIt 接口） | 提供机械臂操作服务/动作给算法层；封装工具管理。 |

### 3. 算法与估计层（Algorithm Layer）

| 包名 | 职责 | 接口 |
| --- | --- | --- |
| `mag_pose_estimator` | 磁场建模、姿态估计 | 订阅传感器数据、目标 TF；发布 `/magnetic/pose_estimated`、误差诊断。 |
| `mag_tracking_control` | 闭环控制策略 | 订阅估计结果，调用 `mag_device_arm` 服务，实现磁铁臂追踪。 |
| `mag_tracking_calib` | 阵列/噪声/磁场参数标定 | 输出配置供 `mag_core_description` 与驱动节点使用。 |

### 4. 应用编排层（Application Layer）

| 包名 | 职责 | 内容 |
| --- | --- | --- |
| `mag_tracking_bringup` | 系统级 launch 与参数模板 | 提供实机、仿真、回放等模式的启动文件。 |
| `mag_viz` | 可视化节点与 RViz 配置 | 订阅估计/真值/传感器话题，展示 TF、轨迹、场强。 |
| `mag_tracking_scan` | 扫描/记录流程编排 | 调用设备层和算法层实现扫描任务（原 `mag_arm_scan` 拆分后归属此处）。 |

### 5. 工具与测试层（Tools & Test）

| 包名 | 职责 | 备注 |
| --- | --- | --- |
| `mag_tools_analysis` | 离线数据分析脚本 | 支撑标定、回放、性能评估。 |
| `mag_tools_sim` | 场景生成、仿真批量运行脚本 | 便于快速复现不同参数组合。 |
| `tests/` | 统一测试目录（rostest/pytest） | 随各包提供单元/集成测试，提升可维护性。 |

## 典型数据/控制接口

- 传感器输出：`/magnetic/sensor/raw`、`/magnetic/sensor/field` (`mag_core_msgs/MagSensorData`)。
- 磁铁真值：`/tf` 中 `magnet` 帧；可选 `/magnetic/pose_true` 话题提供姿态消息。
- 估计结果：`/magnetic/pose_estimated` (`geometry_msgs/PoseStamped` 或定制消息)。
- 控制接口：`/magnetic/control/goal`（action/service），由 `mag_tracking_control` 提供打通机械臂指令的接口。
- 配置参数：各节点从私有命名空间读取 `config/*.yaml`；阵列几何由 `mag_core_description` 提供，保证实机/仿真一致。

## 参数读取规范

- **统一入口**：所有节点使用 `mag_core_utils::param::StructReader` / `ArrayReader` 访问参数服务器，保持错误信息、默认值语义一致。
- **加载流程**：node 通过 `StructReader::fromParameter(~, key)` 获取根结构，将解析后的配置 struct 传递给业务类（例如 `SensorArrayDescription`）。
- **错误约定**：缺失字段抛出 `std::runtime_error`，上下文采用 `~key.sub_field` 形式，便于快速定位配置问题。
- **文档示例**：各包在 `config/` 目录提供 YAML 模板，字段说明与默认值需在 README 中同步维护，确保仿真/实机共用。
- **职责边界**：几何/描述类包只维护结构化数据；与运行时行为相关的参数（频率、超时等）在各节点或 bringup 配置中定义。

## 实施步骤建议

1. **逐包迁移**：按层级顺序重命名/拆分现有包，优先完成基础层和设备层以保证接口稳定。
2. **统一依赖**：更新 `CMakeLists.txt` 与 `package.xml`，明确 `build_depend` / `exec_depend`，避免隐式耦合。
3. **接口对齐**：在 bringup 包的 launch 中集中加载共享配置，检查 topic/service/action 名称的一致性。
4. **文档维护**：在 `docs/` 中补充 README/接入指南，描述包职责、接口契约、启动方式。
5. **测试体系**：为拆分后的包补齐单元与集成测试，在 CI 或本地脚本中统一运行，以验证改造的正确性。

通过以上规划，可以将原有 `mag_*` 系列包细化成职责清晰的多层结构，降低后续扩展与维护成本，同时便于团队协作与代码审查。
