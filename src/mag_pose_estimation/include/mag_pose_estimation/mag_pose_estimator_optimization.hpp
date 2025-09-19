#pragma once

#include <ceres/ceres.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <mag_pose_estimation/mag_field_calculator.hpp>
#include <mag_pose_estimation/mag_pose_estimator_base.hpp>
#include <map>

namespace mag_pose_estimation
{

    struct MagnetFieldResidual
    {
        MagnetFieldResidual(const Eigen::Vector3d &sensor_pos, const Eigen::Vector3d &measured_field, double strength)
            : sensor_pos_(sensor_pos), measured_field_(measured_field), strength_(strength) {}

        template <typename T>
        bool operator()(const T *const position, const T *const direction, T *residuals) const
        {
            Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
            Eigen::Matrix<T, 3, 1> position_T(position[0], position[1], position[2]);
            Eigen::Matrix<T, 3, 1> direction_T(direction[0], direction[1], direction[2]);
            // Predicted magnetic field at sensor position
            Eigen::Matrix<T, 3, 1> pred =
                MagneticFieldCalculator::calculateMagneticFieldT<T>(sensor_pos_T, position_T, direction_T, T(strength_));

            Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
            Eigen::Matrix<T, 3, 1> r = pred - measured_T;
            residuals[0] = r[0];
            residuals[1] = r[1];
            residuals[2] = r[2];
            return true;
        }

        const Eigen::Vector3d sensor_pos_;
        const Eigen::Vector3d measured_field_;
        const double strength_;
    };

    struct MagnetFieldResidualWithStrength
    {
        MagnetFieldResidualWithStrength(const Eigen::Vector3d &sensor_pos, const Eigen::Vector3d &measured_field)
            : sensor_pos_(sensor_pos), measured_field_(measured_field) {}

        template <typename T>
        bool operator()(const T *const position, const T *const direction, const T *const strength, T *residuals) const
        {
            Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
            Eigen::Matrix<T, 3, 1> position_T(position[0], position[1], position[2]);
            Eigen::Matrix<T, 3, 1> direction_T(direction[0], direction[1], direction[2]);
            Eigen::Matrix<T, 3, 1> pred =
                MagneticFieldCalculator::calculateMagneticFieldT<T>(sensor_pos_T, position_T, direction_T, strength[0]);

            Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
            Eigen::Matrix<T, 3, 1> r = pred - measured_T;
            residuals[0] = r[0];
            residuals[1] = r[1];
            residuals[2] = r[2];
            return true;
        }

        const Eigen::Vector3d sensor_pos_;
        const Eigen::Vector3d measured_field_;
    };

} // namespace mag_pose_estimation

namespace mag_pose_estimation
{
    class OptimizationMagnetPoseEstimator : public BaseMagnetPoseEstimator
    {
    public:
        explicit OptimizationMagnetPoseEstimator(ros::NodeHandle &nh);

        void reset() override;
        bool estimate(const std::map<int, MagneticField> &measurements, MagnetPose &out_pose, double *out_error) override;

    private:
        void loadParameters();
        void resetToInitialParameters();

        ros::NodeHandle nh_;

        Eigen::Vector3d initial_position_;
        Eigen::Vector3d initial_direction_;
        double initial_strength_;
        double strength_delta_;
        bool optimize_strength_;
        double strength_min_, strength_max_;

        int max_iterations_;
        double function_tolerance_;
        double gradient_tolerance_;
        double parameter_tolerance_;
        int num_threads_;
        bool minimizer_progress_to_stdout_;

        ceres::LinearSolverType linear_solver_type_;

        Eigen::Vector3d current_position_;
        Eigen::Vector3d magnetic_direction_;
        double magnet_strength_;
    };

} // namespace mag_pose_estimation
