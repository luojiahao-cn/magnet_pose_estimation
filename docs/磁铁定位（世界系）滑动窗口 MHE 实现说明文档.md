
这份文档包含：

* 系统模型（状态、参数、过程方程）
* 测量模型（磁偶极子 + 地磁 + 偏置）
* 两阶段估计流程
* 滑动窗口 MHE 残差定义
* Jacobian 与优化框架
* 实现要求（C++/Python 均可）



---

# ✅ **《磁铁定位（世界系）滑动窗口 MHE 实现说明文档》

---

# ----------------------

# **1. 坐标系定义与 TF 变换**

# ----------------------

* **W**：世界坐标系（output_frame，通过 ROS TF 定义）
* **S_k**：机械臂端部传感器阵列坐标系（随时间变化，通过 TF 查询获得）
* 传感器阵列中第 i 个传感器在 S 系的坐标固定为：
  [
  s_i^S \in \mathbb{R}^3,\quad i=1..M
  ]

### **1.1 从 TF 获取传感器阵列位姿**

通过 ROS TF 查询获取传感器阵列在世界系中的位姿：

```cpp
// 查询 TF 变换：从 output_frame 到 sensor_array_frame
geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
    output_frame, sensor_array_frame, stamp, timeout);

// 提取旋转矩阵 R_{SW}(k) 和平移向量 t_{SW}(k)
tf2::Quaternion q;
tf2::fromMsg(transform.transform.rotation, q);
Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
Eigen::Matrix3d R_SW = q_eigen.toRotationMatrix();  // 从 S 到 W 的旋转
Eigen::Vector3d t_SW(
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z
);

// 计算从 W 到 S 的旋转（用于将世界系向量转换到传感器系）
Eigen::Matrix3d R_WS = R_SW.transpose();
Eigen::Vector3d t_WS = -R_WS * t_SW;
```

### **1.2 传感器位置转换**

传感器 i 在世界系中的位置：

[
s_{k,i}^W = R_{WS}(k) \cdot s_i^S + t_{WS}(k)
]

其中：
* `s_i^S`：传感器 i 在传感器阵列坐标系 S 中的固定位置（从配置文件读取）
* `R_{WS}(k)`：时刻 k 从世界系到传感器系的旋转矩阵（从 TF 查询获得）
* `t_{WS}(k)`：时刻 k 从世界系到传感器系的平移向量（从 TF 查询获得）

---

# ----------------------

# **2. 状态定义（每个时刻 k）**

# ----------------------

[
x_k =
\begin{bmatrix}
p_k^W \    \text{磁铁在世界系的位置 }(3) \
v_k^W \    \text{磁铁速度 }(3) \
u_k^W       \text{磁铁磁化方向（单位向量）}(3)
\end{bmatrix}
\in \mathbb{R}^9
]

磁化方向满足：

[
|u_k^W|=1
]

---

# ----------------------

# **3. 系统参数（不随时间变化）**

# ----------------------

* 地磁场（世界系）：
  [
  E^W \in \mathbb R^3
  ]

* 传感器偏置：
  [
  b^S = [b_1^S,\dots,b_M^S],\quad b_i^S\in\mathbb R^3
  ]

若阶段 II 中希望允许缓慢变化，也可放入状态优化。

---

# ----------------------

# **4. 过程模型（离散时间）**

# ----------------------

采样周期 Δt：

[
p_{k+1}^W = p_k^W + \Delta t, v_k^W + w_k^p
]
[
v_{k+1}^W = v_k^W + w_k^v
]
[
u_{k+1}^W = u_k^W + w_k^u \quad\text{（随后归一化）}
]

合并为：

[
x_{k+1}=f(x_k)+w_k
]

过程噪声协方差：

[
Q = \mathrm{diag}(Q_p,Q_v,Q_u)
]

---

# ----------------------

# **5. 测量模型（磁偶极子 + 地磁 + 偏置）**

# ----------------------

每个传感器测量：

[
y_{k,i}^S = R_{SW}(k) B_{k,i}^W + R_{SW}(k) E^W + b_i^S + v_{k,i}
]

其中：

### **5.1 偶极子磁场模型**

磁铁磁场（世界系）：

设磁矩向量：

[
m_k^W = m_0 , u_k^W
]

磁铁至传感器向量：

[
r_{k,i}^W = s_{k,i}^W - p_k^W
]

规范化：

[
\hat r = r / |r|
]

磁场表达式：

[
B_{k,i}^W = \frac{\mu_0 m_0}{4\pi |r|^3}
\Big(3\hat r(\hat r^\top u_k^W) - u_k^W\Big)
]

### **5.2 堆叠为 M 传感器的函数**

定义：

[
h(x_k) =
\begin{bmatrix}
R_{SW}(k) B_{k,1}^W \
\vdots \
R_{SW}(k) B_{k,M}^W
\end{bmatrix}
]

地磁项：

[
H_E(k)E^W =
\begin{bmatrix}
R_{SW}(k)E^W \ \vdots \ R_{SW}(k)E^W
\end{bmatrix}
]

偏置项：

[
b^S = [b_1^S,\dots, b_M^S]
]

最终测量方程：

[
y_k^S = h(x_k) + H_E(k)E^W + b^S
]

---

# ----------------------

# **6. 两阶段估计流程**

# ----------------------

## **阶段 I（无磁铁，估计 E^W 和 b^S）**

### **6.1 阶段 I 模型**

当无磁铁时，测量模型退化为：

[
y_{k,i}^S = R_{SW}(k) E^W + b_i^S + v_{k,i}
]

堆叠所有传感器：

[
y_k^S = H_E(k) E^W + b^S + v_k
]

其中：

[
H_E(k) = \begin{bmatrix} R_{SW}(k) \\ \vdots \\ R_{SW}(k) \end{bmatrix} \in \mathbb{R}^{3M \times 3}
]

### **6.2 阶段 I 优化问题**

收集 N 个时刻的数据（N ≥ 10，确保可观性）：

[
Y = \begin{bmatrix} y_1^S \\ \vdots \\ y_N^S \end{bmatrix} \in \mathbb{R}^{3MN}
]

[
H = \begin{bmatrix} H_E(1) & I_{3M} \\ \vdots & \vdots \\ H_E(N) & I_{3M} \end{bmatrix} \in \mathbb{R}^{3MN \times (3+3M)}
]

[
\Theta = \begin{bmatrix} E^W \\ b^S \end{bmatrix} \in \mathbb{R}^{3+3M}
]

线性最小二乘：

[
\hat\Theta = \arg\min_\Theta \|Y - H\Theta\|^2 = (H^\top H)^{-1} H^\top Y
]

### **6.3 阶段 I 实现要求**

**文件：`estimator_mhe_stage1.cpp` / `estimator_mhe_stage1.h`**

需要实现：

1. **数据收集器**：
   - 收集无磁铁时的传感器测量数据
   - 同时记录每个时刻的 TF 变换（R_{SW}(k), t_{SW}(k)）
   - 确保机械臂有足够的姿态变化（至少 10 个不同姿态）

2. **线性最小二乘求解器**：
   - 构建矩阵 H 和向量 Y
   - 使用 SVD 或 Cholesky 分解求解
   - 检查条件数，确保可观性

3. **结果验证**：
   - 计算残差：`r = Y - H * Θ`
   - 检查残差是否在合理范围内（< 1 mT）
   - 输出估计的地磁场和偏置

---

## **阶段 II（有磁铁，滑动窗口 MHE）**

### **6.4 阶段 II 窗口定义**

在窗口：

[
\mathcal W_k = \{k-L+1,\dots, k\}
]

其中 L 为窗口大小（典型值：10-20）。

### **6.5 阶段 II 优化变量**

优化变量包括：

* 各时刻状态：`x_{k-L+1}, ..., x_k`（每个 9 维）
* 地磁场：`E^W`（3 维，可选，也可固定为阶段 I 的估计值）
* 传感器偏置：`b^S`（3M 维，可选，也可固定为阶段 I 的估计值）

总优化变量维度：

[
\text{dim}(X) = L \times 9 + 3 + 3M
]

（如果 E^W 和 b^S 固定，则维度为 `L × 9`）

---

# ----------------------

# **7. 残差定义（MHE 所需）**

# ----------------------

总变量：

[
X = {x_{k-L+1},\dots,x_k, E^W, b^S}
]

### **7.1 测量残差**

[
r_j^{\text{meas}}(X) = y_j^S - h(x_j) - H_E(j)E^W - b^S
]

### **7.2 过程残差**

[
r_j^{\text{proc}}(X) = x_{j+1} - f(x_j)
]

### **7.3 单位向量约束（软约束）**

[
r_j^{\text{unit}}(X) = |u_j^W|^2 - 1
]

### **7.4 先验残差（边缘化得到）**

当滑动窗口已满（达到最大长度 L）时，需要边缘化最老的状态 x_{k-L+1}，保留其信息作为先验约束。

#### **7.4.1 边缘化过程**

假设当前窗口状态为：

[
X_{\text{old}} = [x_{k-L+1}, x_{k-L+2}, ..., x_k, E^W, b^S]
]

边缘化最老状态 x_{k-L+1} 后，新窗口状态为：

[
X_{\text{new}} = [x_{k-L+2}, ..., x_k, x_{k+1}, E^W, b^S]
]

边缘化步骤：

1. **构建完整残差向量**（包含所有残差项）：
   [
   r_{\text{full}} = \begin{bmatrix} r^{\text{meas}} \\ r^{\text{proc}} \\ r^{\text{unit}} \end{bmatrix}
   ]

2. **计算完整雅可比矩阵**：
   [
   J_{\text{full}} = \frac{\partial r_{\text{full}}}{\partial X_{\text{old}}}
   ]

3. **构建信息矩阵**（假设单位协方差）：
   [
   \Lambda_{\text{full}} = J_{\text{full}}^\top J_{\text{full}}
   ]

4. **分块矩阵**（分离要边缘化的状态 x_{k-L+1} 和要保留的状态）：
   [
   \Lambda_{\text{full}} = \begin{bmatrix} \Lambda_{11} & \Lambda_{12} \\ \Lambda_{21} & \Lambda_{22} \end{bmatrix}
   ]
   
   其中：
   * `Λ_{11}`：对应 x_{k-L+1} 的信息矩阵块（9×9）
   * `Λ_{22}`：对应其他状态的信息矩阵块
   * `Λ_{12} = Λ_{21}^\top`：交叉项

5. **Schur 补（边缘化）**：
   [
   \Lambda_{\text{marg}} = \Lambda_{22} - \Lambda_{21} \Lambda_{11}^{-1} \Lambda_{12}
   ]

6. **先验残差线性化**：
   在最优解 X^* 处线性化：
   [
   r^{\text{prior}} = r_0 + J_0 \delta X_{\text{new}}
   ]
   
   其中：
   * `r_0`：在 X^* 处的残差值（通常为 0，因为已优化）
   * `J_0`：从 `Λ_{\text{marg}}` 的 Cholesky 分解得到：`Λ_{\text{marg}} = J_0^\top J_0`

#### **7.4.2 先验残差在优化中的使用**

在下一窗口优化时，将先验残差加入代价函数：

[
J(X) = \|r^{\text{meas}}\|_{R^{-1}}^2 + \|r^{\text{proc}}\|_{Q^{-1}}^2 + \|r^{\text{unit}}\|_{S^{-1}}^2 + \|r^{\text{prior}}\|_{\Lambda}^2
]

其中 `Λ = Λ_{\text{marg}}` 是边缘化后的信息矩阵。

#### **7.4.3 实现注意事项**

* **数值稳定性**：使用 Cholesky 分解或 LDLT 分解计算 Schur 补
* **稀疏性**：利用信息矩阵的稀疏结构（x_{k-L+1} 只与 x_{k-L+2} 有过程残差关联）
* **增量更新**：如果窗口大小固定，可以增量更新信息矩阵，避免重复计算

---

# ----------------------

# **8. 总体优化问题（加权最小二乘）**

# ----------------------

代价函数：

[
J(X) =
|r^{\text{meas}}|*{R^{-1}}^2 +
|r^{\text{proc}}|*{Q^{-1}}^2 +
|r^{\text{unit}}|*{S^{-1}}^2 +
|r^{\text{prior}}|*{\Lambda}^2
]

求解：

[
\hat X = \arg\min_X J(X)
]

使用：

* 高斯牛顿
* 或 Levenberg–Marquardt
* 或 Ceres / g2o / CasADi 自带求解器

每次迭代后对 (u_j^W) 做归一化：

[
u_j^W \leftarrow u_j^W / |u_j^W|
]

---

# ----------------------

# **9. 解析雅可比矩阵（必须实现）**

# ----------------------

**重要**：必须实现解析雅可比，避免有限差分的数值误差和计算开销。

### **9.1 偶极子磁场对位置的偏导**

偶极子磁场（世界系）：

[
B_{k,i}^W = \frac{\mu_0 m_0}{4\pi |r|^3} \Big(3\hat r(\hat r^\top u_k^W) - u_k^W\Big)
]

其中 `r = s_{k,i}^W - p_k^W`，`\hat r = r / |r|`。

对位置 p_k^W 的偏导：

[
\frac{\partial B_{k,i}^W}{\partial p_k^W} = -\frac{\partial B_{k,i}^W}{\partial r}
]

详细推导：

[
\frac{\partial B_{k,i}^W}{\partial r} = \frac{\mu_0 m_0}{4\pi} \left[
\frac{3}{|r|^5} \left( u_k^W \hat r^\top + \hat r (u_k^W)^\top + (\hat r^\top u_k^W) I \right)
- \frac{15}{|r|^7} (\hat r^\top u_k^W) \hat r \hat r^\top
- \frac{3}{|r|^5} u_k^W \hat r^\top
\right]
]

简化后：

[
\frac{\partial B_{k,i}^W}{\partial r} = \frac{\mu_0 m_0}{4\pi |r|^5} \left[
3(\hat r^\top u_k^W) I + 3 \hat r (u_k^W)^\top - 15 (\hat r^\top u_k^W) \hat r \hat r^\top - 3 u_k^W \hat r^\top
\right]
]

测量模型中的雅可比（传感器系）：

[
\frac{\partial y_{k,i}^S}{\partial p_k^W} = -R_{SW}(k) \frac{\partial B_{k,i}^W}{\partial r}
]

### **9.2 偶极子磁场对方向的偏导**

对磁化方向 u_k^W 的偏导：

[
\frac{\partial B_{k,i}^W}{\partial u_k^W} = \frac{\mu_0 m_0}{4\pi |r|^3} \left(3\hat r \hat r^\top - I\right)
]

测量模型中的雅可比（传感器系）：

[
\frac{\partial y_{k,i}^S}{\partial u_k^W} = R_{SW}(k) \frac{\mu_0 m_0}{4\pi |r|^3} \left(3\hat r \hat r^\top - I\right)
]

### **9.3 地磁场和偏置的雅可比**

对地磁场 E^W 的偏导：

[
\frac{\partial y_{k,i}^S}{\partial E^W} = R_{SW}(k)
]

对传感器偏置 b_i^S 的偏导：

[
\frac{\partial y_{k,i}^S}{\partial b_i^S} = I_3
]

### **9.4 过程模型雅可比**

过程模型：

[
f(x_k) = \begin{bmatrix} p_k^W + \Delta t \cdot v_k^W \\ v_k^W \\ u_k^W \end{bmatrix}
]

雅可比矩阵：

[
\frac{\partial f}{\partial x_k} = \begin{bmatrix}
I_3 & \Delta t \cdot I_3 & 0_{3 \times 3} \\
0_{3 \times 3} & I_3 & 0_{3 \times 3} \\
0_{3 \times 3} & 0_{3 \times 3} & I_3
\end{bmatrix}
]

### **9.5 单位向量约束雅可比**

约束：`\|u_k^W\|^2 - 1 = 0`

残差：`r^{\text{unit}} = \|u_k^W\|^2 - 1`

雅可比：

[
\frac{\partial r^{\text{unit}}}{\partial u_k^W} = 2 (u_k^W)^\top
]

### **9.6 完整测量残差雅可比结构**

对于窗口中的时刻 j，测量残差：

[
r_j^{\text{meas}} = y_j^S - h(x_j) - H_E(j) E^W - b^S
]

雅可比矩阵结构（按优化变量顺序）：

[
\frac{\partial r_j^{\text{meas}}}{\partial X} = \begin{bmatrix}
0 & \cdots & \frac{\partial r_j}{\partial x_j} & \cdots & 0 & \frac{\partial r_j}{\partial E^W} & \frac{\partial r_j}{\partial b^S}
\end{bmatrix}
]

其中：

* `\frac{\partial r_j}{\partial x_j}`：9×9 矩阵，包含对 p_j, v_j, u_j 的偏导
  * 对 p_j：`-R_{SW}(j) \frac{\partial B^W}{\partial r}`（3×3）
  * 对 v_j：0（3×3，测量不依赖速度）
  * 对 u_j：`-R_{SW}(j) \frac{\mu_0 m_0}{4\pi |r|^3} (3\hat r \hat r^\top - I)`（3×3）

* `\frac{\partial r_j}{\partial E^W}`：3M×3 矩阵，每 3 行为 `-R_{SW}(j)`

* `\frac{\partial r_j}{\partial b^S}`：3M×3M 矩阵，为 `-I_{3M}`

### **9.7 过程残差雅可比**

过程残差：

[
r_j^{\text{proc}} = x_{j+1} - f(x_j)
]

雅可比：

[
\frac{\partial r_j^{\text{proc}}}{\partial X} = \begin{bmatrix}
0 & \cdots & -\frac{\partial f}{\partial x_j} & I_9 & \cdots & 0 & 0 & 0
\end{bmatrix}
]

---

# ----------------------

# **10. 实现细节要求**

# ----------------------

## **10.1 代码文件结构**

需要实现两个独立的代码文件：

### **文件 1：`estimator_mhe_stage1.cpp` / `estimator_mhe_stage1.h`**

**功能**：阶段 I - 无磁铁时的地磁场和传感器偏置估计

**主要类**：

```cpp
class MHEStage1Estimator {
public:
    // 添加测量数据（无磁铁时）
    void addMeasurement(const std::vector<sensor_msgs::MagneticField>& measurements,
                       const std::vector<Eigen::Matrix3d>& R_SW_list,
                       const std::vector<Eigen::Vector3d>& t_SW_list);
    
    // 执行校准（线性最小二乘）
    bool calibrate(Eigen::Vector3d& E_W_est, 
                   std::vector<Eigen::Vector3d>& bias_est);
    
    // 检查数据是否足够（可观性检查）
    bool checkObservability();
    
private:
    std::vector<Eigen::VectorXd> measurements_;  // 收集的测量数据
    std::vector<Eigen::Matrix3d> R_SW_list_;    // 对应的旋转矩阵
    int min_samples_;                            // 最小样本数（默认 10）
};
```

### **文件 2：`estimator_mhe_stage2.cpp` / `estimator_mhe_stage2.h`**

**功能**：阶段 II - 有磁铁时的滑动窗口 MHE 估计

**主要类**：

```cpp
class MHEStage2Estimator {
public:
    // 初始化（使用阶段 I 的估计结果）
    void initialize(const Eigen::Vector3d& E_W_init,
                    const std::vector<Eigen::Vector3d>& bias_init);
    
    // 添加新测量并优化
    bool addMeasurementAndOptimize(
        const std::vector<sensor_msgs::MagneticField>& measurements,
        const Eigen::Matrix3d& R_SW,
        const Eigen::Vector3d& t_SW,
        geometry_msgs::Pose& pose_out);
    
    // 边缘化最老状态
    void marginalizeOldestState();
    
private:
    // 核心方法
    Eigen::VectorXd computeResidual(const Eigen::VectorXd& X);
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& X);  // 使用解析雅可比
    Eigen::VectorXd optimizeWindow(const Eigen::VectorXd& X_init);
    
    // 边缘化相关
    void updatePriorResidual(const Eigen::MatrixXd& Lambda_marg,
                             const Eigen::VectorXd& r0);
    
    // 数据成员
    std::deque<WindowFrame> window_;           // 滑动窗口
    PriorResidual prior_residual_;             // 先验残差
    Eigen::Vector3d E_W_;                      // 地磁场（可优化或固定）
    std::vector<Eigen::Vector3d> bias_;       // 传感器偏置（可优化或固定）
    int window_size_;                          // 窗口大小
    bool use_prior_;                           // 是否使用先验残差
};
```

## **10.2 核心数据结构**

### **WindowFrame（窗口中的一帧）**

```cpp
struct WindowFrame {
    ros::Time timestamp;
    Eigen::VectorXd measurement;              // 3M 维测量向量
    Eigen::Matrix3d R_SW;                      // 传感器阵列旋转矩阵
    Eigen::Vector3d t_SW;                     // 传感器阵列平移向量
    std::vector<Eigen::Vector3d> sensor_positions_W;  // 传感器在世界系中的位置
};
```

### **PriorResidual（先验残差）**

```cpp
struct PriorResidual {
    Eigen::VectorXd r0;                        // 先验残差常数项
    Eigen::MatrixXd J0;                        // 先验残差雅可比
    Eigen::MatrixXd Lambda;                    // 信息矩阵
    bool is_valid;                             // 是否有效
};
```

## **10.3 解析雅可比实现要求**

**必须实现以下函数**（在 `estimator_mhe_stage2.cpp` 中）：

```cpp
// 偶极子磁场对位置的雅可比（世界系）
Eigen::Matrix3d computeDipoleJacobianPosition(
    const Eigen::Vector3d& r,           // 相对位置向量
    const Eigen::Vector3d& u,            // 磁化方向
    double m0,                           // 磁矩强度
    double mu0);                          // 真空磁导率

// 偶极子磁场对方向的雅可比（世界系）
Eigen::Matrix3d computeDipoleJacobianDirection(
    const Eigen::Vector3d& r,           // 相对位置向量
    double m0,                           // 磁矩强度
    double mu0);                         // 真空磁导率

// 完整测量残差雅可比（对状态 x_j）
Eigen::MatrixXd computeMeasurementJacobianState(
    int j,                               // 窗口中的时刻索引
    const Eigen::VectorXd& x_j,         // 状态向量 [p, v, u]
    const WindowFrame& frame);           // 该时刻的测量数据

// 完整测量残差雅可比（对地磁场和偏置）
void computeMeasurementJacobianParams(
    int j,
    const Eigen::Matrix3d& R_SW,
    Eigen::MatrixXd& J_E,                // 对 E^W 的雅可比 (3M × 3)
    Eigen::MatrixXd& J_b);               // 对 b^S 的雅可比 (3M × 3M)
```

## **10.4 边缘化实现**

在 `MHEStage2Estimator::marginalizeOldestState()` 中实现：

```cpp
void MHEStage2Estimator::marginalizeOldestState() {
    // 1. 计算当前窗口的完整残差和雅可比
    Eigen::VectorXd X_current = packState();
    Eigen::VectorXd r_full = computeResidual(X_current);
    Eigen::MatrixXd J_full = computeJacobian(X_current);
    
    // 2. 构建信息矩阵
    Eigen::MatrixXd Lambda_full = J_full.transpose() * J_full;
    
    // 3. 分块（分离要边缘化的状态 x_oldest）
    int dim_oldest = 9;  // x_k 的维度
    int dim_rest = Lambda_full.rows() - dim_oldest;
    
    Eigen::MatrixXd Lambda_11 = Lambda_full.block(0, 0, dim_oldest, dim_oldest);
    Eigen::MatrixXd Lambda_12 = Lambda_full.block(0, dim_oldest, dim_oldest, dim_rest);
    Eigen::MatrixXd Lambda_21 = Lambda_full.block(dim_oldest, 0, dim_rest, dim_oldest);
    Eigen::MatrixXd Lambda_22 = Lambda_full.block(dim_oldest, dim_oldest, dim_rest, dim_rest);
    
    // 4. Schur 补
    Eigen::LDLT<Eigen::MatrixXd> solver(Lambda_11);
    Eigen::MatrixXd Lambda_marg = Lambda_22 - Lambda_21 * solver.solve(Lambda_12);
    
    // 5. 从信息矩阵恢复雅可比（Cholesky 分解）
    Eigen::LDLT<Eigen::MatrixXd> cholesky(Lambda_marg);
    Eigen::MatrixXd L = cholesky.matrixL();
    Eigen::VectorXd D = cholesky.vectorD();
    
    // 构建 J0：Lambda_marg = J0^T * J0
    Eigen::MatrixXd J0 = (L * D.cwiseSqrt().asDiagonal()).transpose();
    
    // 6. 更新先验残差
    prior_residual_.Lambda = Lambda_marg;
    prior_residual_.J0 = J0;
    prior_residual_.r0.setZero();  // 在最优解处，残差为 0
    prior_residual_.is_valid = true;
}
```

## **10.5 优化流程（Levenberg-Marquardt）**

在 `MHEStage2Estimator::optimizeWindow()` 中实现：

```cpp
Eigen::VectorXd MHEStage2Estimator::optimizeWindow(const Eigen::VectorXd& X_init) {
    Eigen::VectorXd X = X_init;
    double lambda = lambda_init_;
    
    for (int iter = 0; iter < max_iters_; ++iter) {
        // 1. 归一化磁化方向
        renormalizeOrientations(X);
        
        // 2. 计算残差和雅可比
        Eigen::VectorXd r = computeResidual(X);
        Eigen::MatrixXd J = computeJacobian(X);  // 使用解析雅可比
        
        // 3. 构建正规方程
        Eigen::MatrixXd H = J.transpose() * J;
        Eigen::VectorXd g = J.transpose() * r;
        
        // 4. LM 阻尼
        Eigen::MatrixXd H_damped = H + lambda * Eigen::MatrixXd::Identity(H.rows(), H.cols());
        
        // 5. 求解
        Eigen::LDLT<Eigen::MatrixXd> solver(H_damped);
        Eigen::VectorXd dX = solver.solve(g);
        
        // 6. 试探更新
        Eigen::VectorXd X_new = X - dX;
        Eigen::VectorXd r_new = computeResidual(X_new);
        
        // 7. 自适应调整阻尼
        if (r_new.squaredNorm() < r.squaredNorm()) {
            X = X_new;
            lambda *= 0.5;
        } else {
            lambda *= 2.0;
        }
        
        // 8. 收敛判定
        if (dX.norm() < 1e-6) {
            break;
        }
    }
    
    return X;
}
```

## **10.6 TF 查询集成**

在 `MHEStage2Estimator::addMeasurementAndOptimize()` 中：

```cpp
bool MHEStage2Estimator::addMeasurementAndOptimize(...) {
    // 1. 查询 TF 获取传感器阵列位姿
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(
            output_frame_, sensor_array_frame_, stamp, timeout_);
    } catch (...) {
        return false;
    }
    
    // 2. 提取旋转矩阵和平移向量
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix3d R_SW = q_eigen.toRotationMatrix();
    Eigen::Vector3d t_SW(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
    );
    
    // 3. 计算传感器在世界系中的位置
    Eigen::Matrix3d R_WS = R_SW.transpose();
    Eigen::Vector3d t_WS = -R_WS * t_SW;
    
    std::vector<Eigen::Vector3d> sensor_positions_W;
    for (const auto& s_i_S : sensor_positions_S_) {
        sensor_positions_W.push_back(R_WS * s_i_S + t_WS);
    }
    
    // 4. 构建窗口帧并添加到窗口
    WindowFrame frame;
    frame.timestamp = stamp;
    frame.measurement = packMeasurements(measurements);
    frame.R_SW = R_SW;
    frame.t_SW = t_SW;
    frame.sensor_positions_W = sensor_positions_W;
    
    window_.push_back(frame);
    
    // 5. 如果窗口已满，执行优化
    if (window_.size() >= window_size_) {
        // 如果超过窗口大小，边缘化最老状态
        if (window_.size() > window_size_) {
            marginalizeOldestState();
            window_.pop_front();
        }
        
        // 执行优化
        Eigen::VectorXd X_opt = optimizeWindow(getInitialState());
        
        // 解析结果
        parseOptimizedState(X_opt, pose_out);
        return true;
    }
    
    return false;
}
```

---

# ----------------------

# **11. 可观性要求（用于调参验证）**

# ----------------------

为了避免 rank 缺失，需确保：

### **阶段 I 可观性**

* 机械臂在阶段 I 有足够姿态变化（至少 10 个不同姿态，角度变化 > 30°）
* 阵列传感器数量 M ≥ 4 且不共面
* 检查矩阵 H 的条件数：`cond(H) < 1e6`

### **阶段 II 可观性**

* 磁铁相对阵列有运动（位置或方向变化）
* 滑动窗口长度 L 足够（建议 L ≥ 10）
* 传感器数量 M ≥ 4 且不共面
* 检查优化后的信息矩阵条件数

---

# ----------------------

# **12. 实现检查清单**

# ----------------------

## **阶段 I 实现检查**

- [ ] 实现数据收集器（收集无磁铁时的测量和 TF）
- [ ] 实现线性最小二乘求解器
- [ ] 实现可观性检查（条件数计算）
- [ ] 实现结果验证（残差检查）
- [ ] 输出地磁场和偏置估计结果

## **阶段 II 实现检查**

- [ ] 实现 TF 查询和坐标转换
- [ ] 实现解析雅可比矩阵（所有偏导数）
- [ ] 实现测量残差计算
- [ ] 实现过程残差计算
- [ ] 实现单位向量约束残差
- [ ] 实现先验残差（边缘化）
- [ ] 实现 Levenberg-Marquardt 优化
- [ ] 实现边缘化过程（Schur 补）
- [ ] 实现滑动窗口管理
- [ ] 实现状态初始化和归一化

## **代码质量检查**

- [ ] 所有雅可比使用解析公式，不使用有限差分
- [ ] 边缘化过程数值稳定（使用 LDLT 或 Cholesky）
- [ ] 优化过程有收敛判定
- [ ] 错误处理和日志记录完善
- [ ] 代码注释清晰，包含公式引用

---

# 完整说明结束

## **总结**

本文档提供了完整的磁铁定位滑动窗口 MHE 实现规范，包括：

1. **两阶段估计**：阶段 I 校准地磁场和偏置，阶段 II 滑动窗口 MHE
2. **完整模型**：磁偶极子模型 + 地磁场 + 传感器偏置
3. **解析雅可比**：所有偏导数的详细公式
4. **边缘化**：Schur 补实现先验残差
5. **TF 集成**：从 ROS TF 获取传感器阵列位姿
6. **实现细节**：代码结构、数据结构和算法流程

**可以直接依据本文档实现完整的 C++ 代码**，包括：

* `estimator_mhe_stage1.cpp/h`：阶段 I 校准
* `estimator_mhe_stage2.cpp/h`：阶段 II 滑动窗口 MHE
* 所有解析雅可比函数
* 边缘化和先验残差管理
* TF 查询和坐标转换

**建议使用 Eigen 库进行矩阵运算，使用 ROS TF2 进行坐标变换。**

