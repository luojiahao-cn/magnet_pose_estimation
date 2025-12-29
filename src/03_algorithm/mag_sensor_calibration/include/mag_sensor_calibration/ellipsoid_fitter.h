#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mag_sensor_calibration {

/**
 * @brief 椭圆拟合结果
 */
struct EllipsoidFitResult {
  Eigen::Vector3d hard_iron_offset;  ///< 硬铁偏移（椭球中心）
  Eigen::Matrix3d soft_iron_matrix;  ///< 软铁矩阵（椭球形状校正）
  double fit_error;                  ///< 拟合误差
  bool success;                       ///< 拟合是否成功
};

/**
 * @brief 椭圆拟合器
 * 
 * 功能：对三维磁场数据进行椭圆拟合，计算硬铁偏移和软铁矩阵
 * 
 * 算法原理：
 * - 理想情况下，磁传感器在均匀磁场中旋转应形成球面
 * - 实际测量形成椭球：(B - B_offset)^T * A * (B - B_offset) = 1
 * - 通过最小二乘法拟合椭球参数
 * - 分解得到硬铁偏移 B_offset 和软铁矩阵 A
 */
class EllipsoidFitter {
public:
  /**
   * @brief 对磁场数据进行椭圆拟合
   * @param measurements 磁场测量数据向量（每个元素是一个3D向量 [x, y, z]）
   * @return 拟合结果
   */
  EllipsoidFitResult fit(const std::vector<Eigen::Vector3d> &measurements);

  /**
   * @brief 应用校正
   * @param raw_field 原始磁场向量
   * @param hard_iron_offset 硬铁偏移
   * @param soft_iron_matrix 软铁矩阵
   * @return 校正后的磁场向量
   */
  static Eigen::Vector3d applyCalibration(
      const Eigen::Vector3d &raw_field,
      const Eigen::Vector3d &hard_iron_offset,
      const Eigen::Matrix3d &soft_iron_matrix);

private:
  /**
   * @brief 使用最小二乘法拟合椭球参数
   * @param measurements 测量数据
   * @return 拟合结果
   */
  EllipsoidFitResult fitEllipsoidLSQ(const std::vector<Eigen::Vector3d> &measurements);

  /**
   * @brief 从椭球方程系数计算硬铁偏移和软铁矩阵
   * @param coeffs 椭球方程系数 [a, b, c, d, e, f, g, h, i, j]
   *                对应方程: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz + j = 0
   * @return 拟合结果
   */
  EllipsoidFitResult extractCalibrationParams(const Eigen::VectorXd &coeffs);
};

}  // namespace mag_sensor_calibration

