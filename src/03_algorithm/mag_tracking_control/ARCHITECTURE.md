# mag_tracking_control 架构说明

## 文件结构说明

### 头文件（4个）

#### 1. `tracking_control_strategy_base.h` - 策略基类接口
**作用**：定义所有控制策略的统一接口

**内容**：
- `TrackingControlInput` 结构体：策略输入（当前位姿、磁铁位姿、磁场数据）
- `TrackingControlOutput` 结构体：策略输出（目标位姿、有效性、质量评分）
- `TrackingControlStrategyBase` 抽象基类：定义策略必须实现的接口
  - `initialize()` - 初始化
  - `computeTargetPose()` - 计算目标位姿（核心方法）
  - `name()` - 获取策略名称
  - `reset()` - 重置状态
  - `needsUpdate()` - 是否需要更新

**设计目的**：使用策略模式，让不同算法可以统一接口，便于切换和扩展

---

#### 2. `strategy_fixed_offset.h` - 固定偏移策略
**作用**：实现最简单的控制策略

**算法**：保持传感器阵列相对于磁铁的固定偏移（例如：在磁铁上方5cm）

**适用场景**：
- 磁铁运动范围较小
- 需要简单稳定的跟踪
- 作为初始实现和测试基准

**特点**：
- 实现简单
- 计算快速
- 稳定可靠

---

#### 3. `strategy_adaptive_distance.h` - 自适应距离策略
**作用**：根据磁场强度自动调整传感器位置

**算法**：
- 监测平均磁场强度
- 如果磁场太强 → 远离磁铁
- 如果磁场太弱 → 靠近磁铁
- 目标：保持磁场在最佳测量范围（20-50 mT）

**适用场景**：
- 磁铁运动范围较大
- 需要保持最佳测量精度
- 磁场强度变化较大的场景

**特点**：
- 自适应调整
- 保持最佳测量范围
- 比固定偏移更智能

---

#### 4. `tracking_control_node.h` - ROS节点主类
**作用**：ROS节点封装，整合所有功能

**功能**：
- 订阅磁铁位姿估计结果
- 通过TF获取当前传感器位姿
- 使用策略算法计算目标位姿
- 调用机械臂服务执行运动
- 发布目标位姿（可视化）

**职责**：
1. **参数加载**：从ROS参数服务器读取配置
2. **策略管理**：创建和管理策略实例（工厂模式）
3. **数据流管理**：订阅、发布、服务调用
4. **控制循环**：定时执行控制逻辑

---

## 设计模式说明

### 策略模式（Strategy Pattern）

```
TrackingControlStrategyBase (接口)
    ↑
    ├── FixedOffsetStrategy (实现1)
    ├── AdaptiveDistanceStrategy (实现2)
    └── [未来可添加] FisherOptimalStrategy (实现3)
```

**优点**：
- 易于扩展：添加新策略只需实现基类接口
- 易于切换：通过配置文件即可切换策略
- 代码解耦：策略逻辑与节点逻辑分离

### 工厂模式（Factory Pattern）

在 `TrackingControlNode::createStrategy()` 中：
- 根据配置的 `strategy_type` 字符串
- 动态创建对应的策略实例
- 无需修改节点代码即可支持新策略

---

## 数据流

```
磁铁位姿估计 (/magnetic/pose_estimated)
    ↓
TrackingControlNode::magnetPoseCallback()
    ↓
TrackingControlNode::controlLoop() (定时触发)
    ↓
获取当前传感器位姿 (TF查询)
    ↓
策略::computeTargetPose() (计算目标位姿)
    ↓
发布目标位姿 (/tracking_control/target_pose)
    ↓
调用机械臂服务 (/mag_device_arm/set_end_effector_pose)
    ↓
机械臂执行运动
```

---

## 如何添加新策略

假设要添加 Fisher 信息矩阵策略：

1. **创建头文件** `fisher_optimal_strategy.h`：
```cpp
#include "mag_tracking_control/tracking_control_strategy_base.h"

class FisherOptimalStrategy : public TrackingControlStrategyBase {
    // 实现基类接口
};
```

2. **创建实现文件** `fisher_optimal_strategy.cpp`

3. **在节点中注册**（`tracking_control_node.cpp`）：
```cpp
std::unique_ptr<TrackingControlStrategyBase> TrackingControlNode::createStrategy(...) {
    if (type == "fisher_optimal") {
        return std::make_unique<FisherOptimalStrategy>();
    }
    // ...
}
```

4. **更新配置文件**（添加 fisher_optimal 参数）

5. **更新 CMakeLists.txt**（添加新源文件）

这样就完成了新策略的添加，无需修改其他现有代码！

