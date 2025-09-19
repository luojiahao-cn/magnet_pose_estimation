#include <ceres/jet.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <mag_pose_estimation/mag_field_calculator.hpp>

namespace mag_pose_estimation
{

    Eigen::MatrixXd MagneticFieldCalculator::calculateMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                                    const Eigen::Vector3d &magnetic_position,
                                                                    const Eigen::Vector3d &magnetic_direction,
                                                                    double magnetic_moment_size)
    {
        const double mu_0 = 4 * M_PI * 1e-7;
        Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnetic_position.transpose();
        Eigen::VectorXd r_norm = r_vec.rowwise().norm();
        r_norm = r_norm.array().max(1e-12);
        Eigen::Vector3d magnetic_moment = magnetic_moment_size * magnetic_direction;
        Eigen::VectorXd m_dot_r = r_vec * magnetic_moment;
        Eigen::MatrixXd B =
            (mu_0 / (4 * M_PI)) *
            (3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5))) -
             magnetic_moment.transpose().replicate(r_vec.rows(), 1).array().colwise() / r_norm.array().pow(3));
        return B * 1e3;
    }

    // templated calculateMagneticFieldT is defined inline in the header to allow Ceres Jet instantiation.

} // namespace mag_pose_estimation
