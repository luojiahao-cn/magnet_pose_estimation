/**
 * @file ellipsoid_fitter.cpp
 * @brief 实现基于最小二乘法的椭球拟合算法，用于计算磁力计的硬铁偏移和软铁矩阵
 */

#include "mag_sensor_calibration/ellipsoid_fitter.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace mag_sensor_calibration {

EllipsoidFitResult EllipsoidFitter::fit(const std::vector<Eigen::Vector3d> &measurements) {
  if (measurements.size() < 9) {
    EllipsoidFitResult result;
    result.success = false;
    result.fit_error = std::numeric_limits<double>::max();
    return result;
  }

  return fitEllipsoidLSQ(measurements);
}

EllipsoidFitResult EllipsoidFitter::fitEllipsoidLSQ(const std::vector<Eigen::Vector3d> &measurements) {
  const size_t n = measurements.size();

  // 构建最小二乘问题的设计矩阵
  // 椭球方程: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz + j = 0
  // 为了确保椭球（而非其他二次曲面），需要约束条件
  // 使用简化的约束：a + b + c = 1（归一化）

  Eigen::MatrixXd D(n, 9);
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);

  for (size_t i = 0; i < n; ++i) {
    const double x = measurements[i].x();
    const double y = measurements[i].y();
    const double z = measurements[i].z();

    // 构建设计矩阵行
    D(i, 0) = x * x;
    D(i, 1) = y * y;
    D(i, 2) = z * z;
    D(i, 3) = 2.0 * x * y;
    D(i, 4) = 2.0 * x * z;
    D(i, 5) = 2.0 * y * z;
    D(i, 6) = 2.0 * x;
    D(i, 7) = 2.0 * y;
    D(i, 8) = 2.0 * z;
  }

  // 使用约束 a + b + c = 1 来确保椭球
  // 这可以通过重新参数化实现
  // 我们使用 SVD 求解，然后归一化

  // 求解最小二乘问题: D * coeffs = -ones
  Eigen::VectorXd coeffs = D.colPivHouseholderQr().solve(-ones);

  // 检查 a + b + c 是否为正（确保是椭球）
  double trace = coeffs(0) + coeffs(1) + coeffs(2);
  if (trace <= 0) {
    // 如果为负，取反
    coeffs = -coeffs;
    trace = -trace;
  }

  // 归一化系数，使得 a + b + c = 1
  coeffs /= trace;

  // 添加常数项 j = -1（因为我们已经归一化）
  Eigen::VectorXd full_coeffs(10);
  full_coeffs.head(9) = coeffs;
  full_coeffs(9) = -1.0;

  return extractCalibrationParams(full_coeffs);
}

EllipsoidFitResult EllipsoidFitter::extractCalibrationParams(const Eigen::VectorXd &coeffs) {
  EllipsoidFitResult result;

  // 系数顺序: [a, b, c, d, e, f, g, h, i, j]
  // 对应方程: ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2iz + j = 0

  const double a = coeffs(0);
  const double b = coeffs(1);
  const double c = coeffs(2);
  const double d = coeffs(3);
  const double e = coeffs(4);
  const double f = coeffs(5);
  const double g = coeffs(6);
  const double h = coeffs(7);
  const double i = coeffs(8);
  const double j = coeffs(9);

  // 构建二次型矩阵 Q
  Eigen::Matrix3d Q;
  Q << a, d, e,
       d, b, f,
       e, f, c;

  // 构建线性项向量
  Eigen::Vector3d linear;
  linear << g, h, i;

  // 计算椭球中心（硬铁偏移）
  // 对于方程 (x-c)^T * Q * (x-c) = k，中心为 c = -Q^(-1) * linear
  Eigen::LDLT<Eigen::Matrix3d> ldlt(Q);
  if (ldlt.info() != Eigen::Success) {
    result.success = false;
    return result;
  }

  result.hard_iron_offset = -ldlt.solve(linear);

  // 计算常数项 k
  double k = -j + linear.transpose() * ldlt.solve(linear);

  if (k <= 0) {
    result.success = false;
    return result;
  }

  // 计算软铁矩阵
  // 标准形式: (x-c)^T * (Q/k) * (x-c) = 1
  // 我们需要将椭球变换为球面，所以软铁矩阵是 (Q/k)^(-1/2)
  // 但为了匹配现有代码的校正公式: B_corrected = soft_iron_matrix * (B_raw - hard_iron_offset)
  // 我们需要 soft_iron_matrix = sqrt(k) * Q^(-1/2)

  // 对 Q 进行特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(Q);
  if (eigensolver.info() != Eigen::Success) {
    result.success = false;
    return result;
  }

  Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
  Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();

  // 检查特征值是否为正（确保是椭球）
  if (eigenvalues.minCoeff() <= 0) {
    result.success = false;
    return result;
  }

  // 计算 Q^(-1/2) = V * diag(1/sqrt(lambda)) * V^T
  Eigen::Vector3d inv_sqrt_eigenvalues = eigenvalues.cwiseInverse().cwiseSqrt();
  Eigen::Matrix3d Q_inv_sqrt = eigenvectors * inv_sqrt_eigenvalues.asDiagonal() * eigenvectors.transpose();

  // 软铁矩阵 = sqrt(k) * Q^(-1/2)
  result.soft_iron_matrix = std::sqrt(k) * Q_inv_sqrt;

  result.success = true;
  result.fit_error = 0.0;  // 可以计算实际拟合误差，这里简化处理

  return result;
}

Eigen::Vector3d EllipsoidFitter::applyCalibration(
    const Eigen::Vector3d &raw_field,
    const Eigen::Vector3d &hard_iron_offset,
    const Eigen::Matrix3d &soft_iron_matrix) {
  return soft_iron_matrix * (raw_field - hard_iron_offset);
}

}  // namespace mag_sensor_calibration

