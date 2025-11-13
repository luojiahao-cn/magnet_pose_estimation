#pragma once
#include <ceres/jet.h>

#include <Eigen/Dense>

namespace mag_pose_estimation
{

    class MagneticFieldCalculator
    {
    public:
        static Eigen::MatrixXd calculateMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                      const Eigen::Vector3d &magnetic_position,
                                                      const Eigen::Vector3d &magnetic_direction,
                                                      double magnetic_moment_size);

        template <typename T>
        static Eigen::Matrix<T, 3, 1> calculateMagneticFieldT(const Eigen::Matrix<T, 3, 1> &sensor_pos,
                                                              const Eigen::Matrix<T, 3, 1> &position,
                                                              const Eigen::Matrix<T, 3, 1> &direction,
                                                              T strength)
        {
            // Magnetic dipole model (Tesla). We output mT to match the non-templated variant.
            const T mu_0 = T(4.0 * M_PI * 1e-7);
            const T four_pi = T(4.0 * M_PI);

            Eigen::Matrix<T, 3, 1> r = sensor_pos - position;
            // Use squared norm to avoid sqrt until needed; add small epsilon for stability.
            const T r2 = r.squaredNorm() + T(1e-24);
            const T inv_r = ceres::sqrt(T(1) / r2);  // 1 / r
            const T inv_r3 = inv_r * inv_r * inv_r;  // 1 / r^3
            const T inv_r5 = inv_r3 * inv_r * inv_r; // 1 / r^5

            const Eigen::Matrix<T, 3, 1> m = strength * direction;
            const T m_dot_r = m.dot(r);

            Eigen::Matrix<T, 3, 1> B = (mu_0 / four_pi) * (T(3.0) * m_dot_r * r * inv_r5 - m * inv_r3);
            return B * T(1e3);
        }
    };

} // namespace mag_pose_estimation
