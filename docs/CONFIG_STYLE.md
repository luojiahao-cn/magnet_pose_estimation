# YAML 配置风格指南

统一的参数文件结构可以让各个 ROS 包共享相同的加载模式，并减少重复配置或字段命名差异。以下约定适用于 `src/` 与 `archive/` 中所有需要加载 YAML 的节点。

## 顶层骨架

所有配置文件使用 `config:` 作为唯一根节点，整体分块建议如下：

```yaml
config:
  meta:               # 可选，描述版本/用途
  frames:             # 坐标系及 TF 相关参数
  topics:             # ROS topic 名称映射
  params:             # 与功能相关的标量/向量参数
  tooling:            # 需要 mesh/工具挂载的设备
  motion:             # 动作/运动学参数（含轨迹）
  simulation:         # 仅仿真用配置（如噪声、电磁模型）
  overrides:          # 针对特定实例的差异化参数
```

并非所有模块都需要上述全部分块，但**禁止**在 `config` 外新增根键，以保证 loader 入口一致。

## 命名与数据类型

- 统一使用 `snake_case`。布尔字段以 `enable`/`disable` 或 `use_*` 表达，避免混用 `allow_*` 等不确定语义。
- 坐标/姿态均使用 `{xyz: [x, y, z], rpy: [roll, pitch, yaw]}`，不要出现 `orientation_rpy` 或拆散的 `center.x`、`center.y`。
- 同类枚举字段（例如 `trajectory.type`、`noise.type`）使用小写字符串，取值范围写在文档或注释中。
- 映射型配置优先使用字典而非数组。例如：
  ```yaml
  named_targets:
    ready: ready
    up: up
  ```
  这样 loader 可以直接通过 key 查找，避免遍历数组。
- 数组元素需要 `name`/`id` 字段且保持唯一性。

## 公用段落约定

### frames

```yaml
config:
  frames:
    reference_frame: world
    base_tf:
      parent: world
      child: robot_base
      pose:
        xyz: [0.0, 0.0, 0.0]
        rpy: [0.0, 0.0, 0.0]
    tool_mount:
      parent: robot_tool
      child: magnet_tool
```

如需发布静态 TF，可在 loader 中判断 `frames.base_tf` 是否启用。

### topics

集中定义输入/输出 topic，使用空字符串表示禁用：

```yaml
config:
  topics:
    mag_field: /magnetic/sensor/field
    pose_estimate: /magnetic/pose_estimated
```

### params

模块相关的标量或数组参数全部放在此处，可再细分子块（例如 `params.optimizer.*`）。

### motion / trajectory

```yaml
config:
  motion:
    update_rate_hz: 100.0
    magnetic_strength: 5.0
  trajectory:
    type: circular
    circular:
      center_xyz: [0.0, 0.0, 0.05]
      radius: 0.1
      angular_speed: 0.5
      phase: 0.0
      orientation_rpy: [0.0, 0.0, 0.0]
```

不同轨迹的特定参数放在 `trajectory.<type>.` 下，避免把不同类型的字段堆在同一层。

### tooling

适用于机械臂等需要工具挂载的模块：

```yaml
config:
  tooling:
    selected: magnetic_sensor_bracket
    options:
      magnetic_sensor_bracket:
        frame: magnet_sensor_tool
        parent_frame: robot_tool
        mesh: package://.../magnetic_sensor_bracket.STL
        pose:
          xyz: [0.0, 0.0, 0.0]
          rpy: [1.5708, 0.0, -1.5708]
        scale: [0.001, 0.001, 0.001]
      none:
        frame: robot_tool_mount
        parent_frame: robot_tool
```

### simulation

仿真与硬件共用同一 YAML 时，将仿真特有项集中在 `simulation` 内，launch 文件通过参数覆盖来启用不同子块。

```yaml
config:
  simulation:
    magnet_frame:
      parent: world
      child: magnet
    noise:
      enable: true
      type: gaussian
      mean: 0.0
      stddev: 5.0e-4
```

## 示例模板

```yaml
config:
  meta:
    profile: default
    description: "示例磁场模拟配置"

  frames:
    reference_frame: world
    base_tf:
      parent: world
      child: magnet_base
      pose:
        xyz: [0.0, 0.0, 0.0]
        rpy: [0.0, 0.0, 0.0]

  topics:
    command: /magnetic/command
    pose_true: /magnetic/pose_true

  params:
    magnetic_strength: 5.0
    update_rate_hz: 100.0

  trajectory:
    type: static
    static:
      pose:
        xyz: [0.0, 0.0, 0.05]
        rpy: [0.0, 0.0, 0.0]

  simulation:
    noise:
      enable: false
```

该模板可复制到任一包的 `config/*.yaml` 作为起点，再根据需要添加或删减字段。后续所有 loader 均假设参数位于 `config.<section>` 下，通过 `rosparam_shortcuts` 或 `xmlrpc_utils` 读取，确保解析逻辑统一。