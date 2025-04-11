#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <ros/ros.h>

namespace magnetic_pose_estimation {

/**
 * @brief 计算给定位置的磁场强度
 * 
 * 使用偶极子磁场模型计算多个传感器位置处的磁场强度。
 * 公式：B = (μ₀/4π) * (3(m·r)r/r⁵ - m/r³)
 * 其中：
 * - B：磁场强度向量
 * - μ₀：真空磁导率
 * - m：磁矩向量
 * - r：从磁偶极子到观测点的位置矢量
 * - r：位置矢量的模
 * 
 * @param sensor_positions 传感器位置矩阵，每行表示一个传感器的 [x, y, z] 坐标，形状为 (N, 3)
 * @param magnetic_position 磁偶极子的位置向量 [x, y, z]
 * @param magnetic_direction 磁偶极子的方向单位向量 [dx, dy, dz]
 * @param magnetic_moment_size 磁矩大小（标量值） 单位为安培·米²(A·m²)
 * 
 * @return Eigen::MatrixXd 返回每个传感器位置处的磁场强度矩阵，形状为 (N, 3)，单位为毫特斯拉(mT)
 * 
 * @note 计算结果默认单位为毫特斯拉(mT)，内部计算使用特斯拉(T)后转换
 */
Eigen::MatrixXd MagneticFieldCalculator::calculateMagneticField(
    const Eigen::Matrix<double, -1, 3>& sensor_positions,
    const Eigen::Vector3d& magnetic_position,
    const Eigen::Vector3d& magnetic_direction,
    double magnetic_moment_size
) {
    const double mu_0 = 4 * M_PI * 1e-7;  // 真空磁导率
    
    // 计算位置差向量
    Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnetic_position.transpose();
    
    // 计算距离
    Eigen::VectorXd r_norm = r_vec.rowwise().norm();

    // 检查是否有传感器距离过近
    if ((r_norm.array() < 0.001).any())
    {
        for (int i = 0; i < r_norm.size(); ++i)
        {
            if (r_norm(i) < 0.001)
            {
                ROS_WARN_STREAM("传感器 " << i << " 与磁铁距离过近（"
                                          << r_norm(i) * 1000 << "mm），可能影响计算精度");
            }
        }
    }
    r_norm = r_norm.array().max(1e-12);  // 防止除零
    
    // 计算磁矩向量
    Eigen::Vector3d magnetic_moment = magnetic_moment_size * magnetic_direction;
    
    // 计算点积
    Eigen::VectorXd m_dot_r = r_vec * magnetic_moment;
    
    // 计算磁场
    Eigen::MatrixXd B = (mu_0 / (4 * M_PI)) * (
        3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5)))
        - magnetic_moment.transpose().replicate(r_vec.rows(), 1).array().colwise() 
        / r_norm.array().pow(3)
    );
    
    return B * 1e3;  // 从特斯拉转换为毫特斯拉(mT)
}

} // namespace magnetic_pose_estimation