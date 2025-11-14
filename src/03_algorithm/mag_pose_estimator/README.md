# mag_pose_estimator

你现在是一名 ROS1（C++）机器人软件工程专家与状态估计算法专家，需要为我生成一个完整可运行的 ROS 包 **mag_pose_estimator**。

# 🎯 项目目标
创建一个 ROS1 C++ 包，从其他节点发布的磁力计数据中估计系统的 3D 姿态（orientation）和位置（position），并发布 PoseStamped。包应具有良好的结构化设计，便于扩展不同的状态估计算法（如 EKF、UKF、非线性优化等）。


# 📦 一、ROS 包结构要求
请生成一个 **完整可编译、可运行的 ROS 包**，包含：


要求使用 **catkin** 构建体系。语言：**C++14 或 C++17**。


# 🧩 二、节点功能说明（核心）
创建一个主节点：`mag_pose_estimator_node`，功能包含：

1. **订阅磁力计数据：**
   - topic：`/mag`
   - 类型：`sensor_msgs/MagneticField`

2. **数据预处理：**
   - 磁力计 soft/hard iron 校正（提供默认的校准函数）
   - 噪声滤波（简单一阶或可切换）

3. **支持多种姿态与位置估计算法（可自由切换）：**
   提供统一接口类 `EstimatorBase`，并实现至少两种算法：
   - `EKFEstimator`
   - `OptimizerEstimator`（基于非线性最小二乘，例如 Gauss-Newton / Levenberg-Marquardt）

   要求：
   - 可通过 ROS 参数（或动态参数）切换算法，例如 `estimator_type: ekf` 或 `optimizer`

4. **输出：**
   - 发布 `geometry_msgs/PoseStamped` 到 `/mag_pose`
   - 自动生成 header.stamp 与 frame_id


# 🧠 三、算法设计要求

## 1. 统一接口（必须）
为所有算法编写一个虚基类：

```cpp
class EstimatorBase {
public:
    virtual void initialize() = 0;
    virtual void update(const sensor_msgs::MagneticField& mag) = 0;
    virtual geometry_msgs::Pose getPose() const = 0;
    virtual ~EstimatorBase() {}
};
```

```

---

## ✅ 实现概览

```
include/mag_pose_estimator/
├── estimator_base.h
├── estimator_factory.h
├── ekf_estimator.h
├── optimizer_estimator.h
├── mag_preprocessor.h
└── mag_pose_estimator_node.h
src/
├── estimator_factory.cpp
├── ekf_estimator.cpp
├── optimizer_estimator.cpp
├── mag_preprocessor.cpp
├── mag_pose_estimator_node.cpp
└── mag_pose_estimator_node_main.cpp
config/mag_pose_estimator.yaml
launch/mag_pose_estimator.launch
```

## 🚀 构建与运行

```bash
cd /home/lawkaho/workshop/magnet_pose_estimation
catkin_make
source devel/setup.bash
roslaunch mag_pose_estimator mag_pose_estimator.launch
```

- 在启动前可通过 `rosparam set /mag_pose_estimator/estimator_type optimizer` 切换优化器版本。
- 修改 `config/mag_pose_estimator.yaml` 可统一管理滤波、标定与噪声参数，或在 `launch` 中传入 `config:=/path/to/custom.yaml` 以覆盖默认配置。

## ⚙️ 参数速查

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `estimator_type` | `ekf` / `optimizer` | `ekf` |
| `pose_topic` | PoseStamped 输出话题 | `mag_pose` |
| `output_frame` | PoseStamped frame_id | `map` |
| `world_field` | 世界系磁场向量 (Tesla) | `[2e-5, 0, 4.8e-5]` |
| `position_gain` | 将磁残差映射为位置更新的比例 | `0.02` |
| `process_noise_position` / `process_noise_orientation` | EKF 过程噪声 | `1e-4` / `5e-5` |
| `measurement_noise` | EKF 观测噪声方差 | `1e-3` |
| `optimizer_iterations` / `optimizer_damping` | GN/LM 迭代次数与阻尼 | `15` / `1e-3` |
| `enable_calibration`, `soft_iron_matrix`, `hard_iron_offset` | 软/硬铁补偿开关与参数 | 见 YAML |
| `enable_filter`, `low_pass_alpha` | 一阶低通滤波 | `true`, `0.2` |

## 🧠 算法说明

### 统一接口

```cpp
class EstimatorBase {
public:
   virtual void initialize() = 0;
   virtual void update(const sensor_msgs::MagneticField &mag) = 0;
   virtual geometry_msgs::Pose getPose() const = 0;
   virtual std::string name() const = 0;
   virtual ~EstimatorBase() = default;
};
```

所有算法共用 `EstimatorConfig`，包含世界系磁向量、噪声与优化参数，便于未来扩展 UKF / 粒子滤波等实现。

### EKFEstimator

- 状态：\(x = [p_x, p_y, p_z, q_x, q_y, q_z, q_w, b_x, b_y, b_z]^T\)。
- 预测：当前实现视作随机游走；借助 `process_noise_*` 参数将时间相关噪声累积到协方差（TODO：可扩展 IMU/里程计动力学）。
- 观测：\(\hat{m}_b = R(q)m_w\)，残差 \(r = \text{normalize}(m_b) - \text{normalize}(\hat{m}_b)\)。
- Jacobian 示例：
   \[
   H_q = \frac{\partial R(q)m_w}{\partial q} = \begin{bmatrix}-R[m_w]_\times & Rm_w\end{bmatrix}
   \]
- 更新：利用标准 EKF 增益修正状态，随后重新归一化四元数，并将残差注入偏置以允许建模磁偏漂移。

### OptimizerEstimator

- 目标函数：\(J(q) = \|m_b - R(q)m_w\|^2\)。
- 方法：采用 Gauss-Newton + LM 阻尼，解 \((J^T J + \lambda I)\delta = J^T r\) 获得小旋量增量，再通过指数映射更新四元数。
- 收敛：`optimizer_iterations` 与 `optimizer_damping` 分别控制迭代轮次与阻尼强度；若 `\|\delta\| < 1e-4` 则提前终止。
- 位置：以 `position_gain` 将残差映射成位置改变量，示例化展示接口扩展能力（TODO：可替换为磁梯度地图匹配或外部定位融合）。

## 📚 参考

- S. O. Madgwick, "An efficient orientation filter for inertial and magnetic sensor arrays", 2010.
- M. Kok, J. D. Hol, T. B. Schön, "Using inertial sensors for position and orientation estimation", 2017.
- B. Kuipers, *Quaternions and Rotation Sequences*, Princeton University Press.

> NOTE：当前实现重点在结构化接口，落地部署前务必重新标定磁力计并根据场景调参。

要求包含：

* 状态量：位置 xyz + 四元数 q + 偏置（可选）
* 预测模型（可简化）
* 观测模型：磁力计方向与世界坐标的对齐
* 线性化 Jacobian 的示例

## 3. 优化法示例（Optimizer）

要求包含：

* 构建残差函数：测量磁场方向 vs. 理论方向
* 使用迭代最优化求解姿态（姿态用四元数）
* 可参考 Gauss-Newton / LM


# 📚 四、代码质量要求

* 所有类文件清晰分层，module 化
* 允许用户后续添加新算法（如 UKFEstimator）
* 包含详细注释，备注数学公式与参考文献
* 提供足够的 TODO 标记便于论文扩展
* 所有函数分离声明与定义（.h/.cpp）
* 确保整个包可直接 `catkin_make` 构建


# 🚀 五、期望输出结构（重要）

请输出以下完整内容：

1. **mag_pose_estimator/package.xml**
2. **mag_pose_estimator/CMakeLists.txt**
3. **include/mag_pose_estimator/** 下全部头文件
4. **src/** 下全部 .cpp 文件
5. **节点主程序** `mag_pose_estimator_node.cpp`
6. **示例 launch 文件** `mag_pose_estimator.launch`
7. **示例 config 参数文件**
8. **使用说明**（如何运行、如何切换算法）
9. **附数学解释（EKF & 优化器）**，用于论文撰写参考

务必确保包结构正确且代码可直接运行。

```
