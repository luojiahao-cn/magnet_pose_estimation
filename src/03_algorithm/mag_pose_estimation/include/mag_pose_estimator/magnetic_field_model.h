#pragma once

#include <ceres/jet.h>

#include <Eigen/Dense>
#include <cmath>

namespace mag_pose_estimator {

/**
 * @brief 优化器残差里使用的磁偶极模型。
 */
class MagneticFieldModel {
 public:
  template <typename T>
  static Eigen::Matrix<T, 3, 1> dipoleField(const Eigen::Matrix<T, 3, 1> &sensor_pos,
                                             const Eigen::Matrix<T, 3, 1> &magnet_pos,
                                             const Eigen::Matrix<T, 3, 1> &magnet_dir,
                                             T strength) {
    const T mu0 = T(4.0 * M_PI * 1e-7);
    const T four_pi = T(4.0 * M_PI);

    Eigen::Matrix<T, 3, 1> r = sensor_pos - magnet_pos;
    const T r2 = r.squaredNorm() + T(1e-18);
    const T inv_r = ceres::sqrt(T(1.0) / r2);
    const T inv_r3 = inv_r * inv_r * inv_r;
    const T inv_r5 = inv_r3 * inv_r * inv_r;

    const Eigen::Matrix<T, 3, 1> moment = strength * magnet_dir;
    const T m_dot_r = moment.dot(r);

    Eigen::Matrix<T, 3, 1> B = (mu0 / four_pi) * (T(3.0) * m_dot_r * r * inv_r5 - moment * inv_r3);
    return B;  // 单位：特斯拉
  }
};

}  // 命名空间 mag_pose_estimator
