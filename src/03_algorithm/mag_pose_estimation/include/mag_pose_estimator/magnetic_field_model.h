#pragma once

#include <ceres/jet.h>

#include <Eigen/Dense>
#include <cmath>

namespace mag_pose_estimator {

/**
 * @brief 磁偶极子磁场模型
 * 
 * 用于计算磁偶极子在空间中产生的磁场强度。
 * 基于磁偶极子物理模型：B = (μ₀/4π) * (3(m·r)r/r⁵ - m/r³)
 * 
 * 注意：返回值为 mT（毫特斯拉），以匹配测量数据单位。
 */
class MagneticFieldModel {
 public:
  /**
   * @brief 计算磁偶极子在传感器位置产生的磁场
   * 
   * @tparam T 数值类型（支持 double 和 Ceres Jet 类型，用于自动微分）
   * @param sensor_pos 传感器位置 [x, y, z] (米)
   * @param magnet_pos 磁铁位置 [x, y, z] (米)
   * @param magnet_dir 磁矩方向向量（归一化）
   * @param strength 磁矩强度 (Am²，安培·米²)
   * @return 磁场向量 [Bx, By, Bz] (mT，毫特斯拉)
   * 
   * 公式：
   *   r = sensor_pos - magnet_pos
   *   m = strength * magnet_dir
   *   B = (μ₀/4π) * (3(m·r)r/r⁵ - m/r³) * 1000
   * 
   * 其中 μ₀ = 4π × 10⁻⁷ H/m（真空磁导率）
   */
  template <typename T>
  static Eigen::Matrix<T, 3, 1> dipoleField(const Eigen::Matrix<T, 3, 1> &sensor_pos,
                                             const Eigen::Matrix<T, 3, 1> &magnet_pos,
                                             const Eigen::Matrix<T, 3, 1> &magnet_dir,
                                             T strength) {
    const T mu0 = T(4.0 * M_PI * 1e-7);  // 真空磁导率 μ₀ = 4π × 10⁻⁷ H/m
    const T four_pi = T(4.0 * M_PI);

    // 计算从磁铁到传感器的向量
    Eigen::Matrix<T, 3, 1> r = sensor_pos - magnet_pos;
    
    // 计算距离的平方，添加小量避免除零
    const T r2 = r.squaredNorm() + T(1e-18);
    
    // 计算 1/r, 1/r³, 1/r⁵
    const T inv_r = ceres::sqrt(T(1.0) / r2);
    const T inv_r3 = inv_r * inv_r * inv_r;
    const T inv_r5 = inv_r3 * inv_r * inv_r;

    // 计算磁矩向量
    const Eigen::Matrix<T, 3, 1> moment = strength * magnet_dir;
    const T m_dot_r = moment.dot(r);

    // 磁偶极子模型：B = (μ₀/4π) * (3(m·r)r/r⁵ - m/r³)
    Eigen::Matrix<T, 3, 1> B = (mu0 / four_pi) * (T(3.0) * m_dot_r * r * inv_r5 - moment * inv_r3);
    
    // 转换为 mT（毫特斯拉）以匹配测量数据单位
    return B * T(1e3);
  }
};

}  // 命名空间 mag_pose_estimator
