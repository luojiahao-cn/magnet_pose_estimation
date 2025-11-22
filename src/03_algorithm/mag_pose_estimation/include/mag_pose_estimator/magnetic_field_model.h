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

  /**
   * @brief 计算磁偶极子磁场的解析雅可比矩阵
   * 
   * 磁偶极子模型：B = scale * (3(m·r)r/r⁵ - m/r³)
   * 其中 r = sensor_pos - magnet_pos, m = strength * magnet_dir
   * 
   * 对位置 p 的雅可比：∂B/∂p = -∂B/∂r（因为 r = sensor_pos - magnet_pos，∂r/∂p = -I）
   * 对方向 d 的雅可比：∂B/∂d = ∂B/∂m * strength（因为 m = strength * d）
   * 对强度 s 的雅可比：∂B/∂s = ∂B/∂m * d（因为 m = s * d）
   * 
   * @param sensor_pos 传感器位置 [x, y, z] (米)
   * @param magnet_pos 磁铁位置 [x, y, z] (米)
   * @param magnet_dir 磁矩方向向量（归一化）
   * @param strength 磁矩强度 (Am²)
   * @param jacobian_position 输出的位置雅可比矩阵 [3×3] (mT/m)
   * @param jacobian_direction 输出的方向雅可比矩阵 [3×3] (mT/unit)
   * @param jacobian_strength 输出的强度雅可比向量 [3×1] (mT/(Am²))，可为 nullptr
   */
  static void dipoleFieldJacobian(const Eigen::Vector3d &sensor_pos,
                                   const Eigen::Vector3d &magnet_pos,
                                   const Eigen::Vector3d &magnet_dir,
                                   double strength,
                                   Eigen::Matrix3d &jacobian_position,
                                   Eigen::Matrix3d &jacobian_direction,
                                   Eigen::Vector3d *jacobian_strength = nullptr) {
    const double mu0 = 4.0 * M_PI * 1e-7;  // 真空磁导率
    const double four_pi = 4.0 * M_PI;
    const double scale = mu0 / four_pi * 1e3;  // 转换为 mT 的缩放因子

    // 计算从磁铁到传感器的向量
    Eigen::Vector3d r = sensor_pos - magnet_pos;
    double r2 = r.squaredNorm() + 1e-18;  // 避免除零
    double r_norm = std::sqrt(r2);
    double inv_r = 1.0 / r_norm;
    double inv_r2 = inv_r * inv_r;
    double inv_r3 = inv_r2 * inv_r;
    double inv_r5 = inv_r3 * inv_r2;
    double inv_r7 = inv_r5 * inv_r2;

    // 计算磁矩向量
    Eigen::Vector3d m = strength * magnet_dir;
    double m_dot_r = m.dot(r);

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d r_outer = r * r.transpose();  // 外积 r * r^T
    Eigen::Matrix3d m_outer = m * r.transpose();  // 外积 m * r^T

    // 计算 ∂B/∂r（对位置向量 r 的雅可比）
    // B = scale * (3(m·r)r/r⁵ - m/r³)
    // ∂/∂r [3(m·r)r/r⁵] = 3(m·r)/r⁵ * I + 3/r⁵ * (m*r^T) - 15(m·r)/r⁷ * (r*r^T)
    // ∂/∂r [-m/r³] = 3/r³ * (m*r^T)
    // 合并：∂B/∂r = scale * [3(m·r)/r⁵ * I + 3/r⁵ * (m*r^T) - 15(m·r)/r⁷ * (r*r^T) + 3/r³ * (m*r^T)]
    Eigen::Matrix3d dB_dr = scale * (
      3.0 * m_dot_r * inv_r5 * I +
      3.0 * inv_r5 * m_outer -
      15.0 * m_dot_r * inv_r7 * r_outer +
      3.0 * inv_r3 * m_outer
    );
    
    // 由于 r = sensor_pos - magnet_pos，所以 ∂r/∂p = -I
    // 因此 ∂B/∂p = ∂B/∂r * (-I) = -∂B/∂r
    jacobian_position = -dB_dr;

    // 计算 ∂B/∂m（对磁矩向量 m 的雅可比）
    // ∂B/∂m = scale * (3r/r⁵ * r^T - I/r³)
    Eigen::Matrix3d dB_dm = scale * (
      3.0 * inv_r5 * r_outer -
      inv_r3 * I
    );

    // 计算 ∂B/∂d（对方向的雅可比）
    // 由于 m = strength * d，所以 ∂B/∂d = ∂B/∂m * strength
    jacobian_direction = dB_dm * strength;

    // 计算 ∂B/∂s（对强度的雅可比，如果请求）
    if (jacobian_strength != nullptr) {
      // 由于 m = s * d，所以 ∂B/∂s = ∂B/∂m * d
      *jacobian_strength = dB_dm * magnet_dir;
    }
  }
};

}  // 命名空间 mag_pose_estimator
