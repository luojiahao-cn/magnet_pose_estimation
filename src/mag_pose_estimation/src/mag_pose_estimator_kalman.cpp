#include <ceres/ceres.h>
#include <ceres/jet.h>

#include <algorithm>
#include <cmath>
#include <mag_pose_estimation/mag_pose_estimator_kalman.hpp>

namespace mag_pose_estimation
{
    KalmanMagnetPoseEstimator::KalmanMagnetPoseEstimator(ros::NodeHandle &nh) : nh_(nh)
    {
        loadParameters();
        ROS_INFO("磁铁位置估计器（卡尔曼滤波）已初始化");
    }

    void KalmanMagnetPoseEstimator::loadParameters()
    {
        std::vector<double> position_vec, direction_vec;
        nh_.param("/estimator_config/magnet/position", position_vec, std::vector<double>{0.01, 0.01, 0.05});
        nh_.param("/estimator_config/magnet/direction", direction_vec, std::vector<double>{0, 0, 1});
        nh_.param<double>("/estimator_config/magnet/strength", initial_strength_, 2.0);

        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]);
        initial_direction_.normalize();

        state_dim_ = 5;
        state_ = Eigen::VectorXd::Zero(state_dim_);
        state_.segment<3>(0) = initial_position_;

        double theta = std::acos(std::max(-1.0, std::min(1.0, initial_direction_(2))));
        double phi = std::atan2(initial_direction_(1), initial_direction_(0));
        state_(3) = theta;
        state_(4) = phi;

        double position_process_noise, angle_process_noise, measurement_noise, initial_covariance;
        nh_.param<double>("/estimator_config/kalman/position_process_noise", position_process_noise, 1e-7);
        nh_.param<double>("/estimator_config/kalman/angle_process_noise", angle_process_noise, 1e-8);
        nh_.param<double>("/estimator_config/kalman/measurement_noise", measurement_noise, 1e-2);
        nh_.param<double>("/estimator_config/kalman/initial_covariance", initial_covariance, 0.01);
        measurement_noise_ = measurement_noise;

        P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * initial_covariance;
        Q_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
        Q_.diagonal().segment<3>(0).setConstant(position_process_noise);
        Q_.diagonal().segment<2>(3).setConstant(angle_process_noise);

        // R_ 将在每次 update() 时根据当前使用的测量数量设置尺寸
        R_.resize(0, 0);

        ROS_INFO("卡尔曼滤波器初始状态: 位置=[%.4f, %.4f, %.4f], 角度=[%.4f, %.4f], 强度=%.4f",
                 state_(0),
                 state_(1),
                 state_(2),
                 state_(3),
                 state_(4),
                 initial_strength_);
    }

    void KalmanMagnetPoseEstimator::reset()
    {
        state_.setZero(state_dim_);
        state_.segment<3>(0) = initial_position_;
        double theta = std::acos(std::max(-1.0, std::min(1.0, initial_direction_(2))));
        double phi = std::atan2(initial_direction_(1), initial_direction_(0));
        state_(3) = theta;
        state_(4) = phi;
        P_.setIdentity(state_dim_, state_dim_);
    }

    bool KalmanMagnetPoseEstimator::estimate(const std::map<int, MagneticField> &measurements,
                                             MagnetPose &out_pose,
                                             double *out_error)
    {
        int n = measurements.size();
        if (n == 0)
            return false;
        Eigen::VectorXd measurement(n * 3);
        Eigen::MatrixXd sensor_positions(n, 3);
        int i = 0;
        for (const auto &m : measurements)
        {
            measurement.segment<3>(i * 3) = Eigen::Vector3d(m.second.mag_x, m.second.mag_y, m.second.mag_z);
            sensor_positions.row(i) << m.second.sensor_pose.position.x, m.second.sensor_pose.position.y,
                m.second.sensor_pose.position.z;
            i++;
        }
        predict();
        update(measurement, sensor_positions);

        // Fill out_pose
        out_pose.header.stamp = ros::Time::now();
        out_pose.header.frame_id = "world";
        out_pose.position.x = state_(0);
        out_pose.position.y = state_(1);
        out_pose.position.z = state_(2);

        double theta = state_(3);
        double phi = state_(4);
        Eigen::Vector3d direction(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));

        tf2::Vector3 z_axis(0, 0, 1);
        tf2::Vector3 direction_vec(direction(0), direction(1), direction(2));
        tf2::Vector3 rotation_axis = z_axis.cross(direction_vec);
        double rotation_angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(direction_vec))));

        if (rotation_axis.length() < 1e-6)
        {
            if (direction_vec.z() > 0)
            {
                out_pose.orientation.w = 1.0;
                out_pose.orientation.x = 0.0;
                out_pose.orientation.y = 0.0;
                out_pose.orientation.z = 0.0;
            }
            else
            {
                out_pose.orientation.w = 0.0;
                out_pose.orientation.x = 1.0;
                out_pose.orientation.y = 0.0;
                out_pose.orientation.z = 0.0;
            }
        }
        else
        {
            rotation_axis.normalize();
            tf2::Quaternion q(rotation_axis, rotation_angle);
            q.normalize();
            out_pose.orientation = tf2::toMsg(q);
        }

        out_pose.magnetic_strength = initial_strength_;
        if (out_error)
            *out_error = 0.0; // 未计算误差，先置0
        return true;
    }

    void KalmanMagnetPoseEstimator::predict()
    {
        P_ = P_ + Q_;
    }

    void KalmanMagnetPoseEstimator::update(const Eigen::VectorXd &measurement, const Eigen::MatrixXd &sensor_positions)
    {
        double theta = state_(3);
        double phi = state_(4);
        Eigen::Vector3d direction(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta));

        int n = sensor_positions.rows();

        Eigen::MatrixXd predicted_fields = MagneticFieldCalculator::calculateMagneticField(
            sensor_positions, state_.segment<3>(0), direction, initial_strength_);
        Eigen::VectorXd predicted = Eigen::Map<Eigen::VectorXd>(predicted_fields.data(), predicted_fields.size());

        Eigen::VectorXd y = measurement - predicted;

        Eigen::MatrixXd H(n * 3, state_dim_);
        const double delta = 1e-6;
        for (int j = 0; j < state_dim_; ++j)
        {
            Eigen::VectorXd state_plus = state_;
            state_plus(j) += delta;
            double theta_plus = state_plus(3);
            double phi_plus = state_plus(4);
            Eigen::Vector3d direction_plus(
                std::sin(theta_plus) * std::cos(phi_plus), std::sin(theta_plus) * std::sin(phi_plus), std::cos(theta_plus));
            Eigen::MatrixXd pred_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, state_plus.segment<3>(0), direction_plus, initial_strength_);
            Eigen::VectorXd pred_plus_vec = Eigen::Map<Eigen::VectorXd>(pred_plus.data(), pred_plus.size());
            H.col(j) = (pred_plus_vec - predicted) / delta;
        }

        // 根据测量数量构造测量噪声协方差
        if (R_.rows() != n * 3 || R_.cols() != n * 3)
        {
            R_ = Eigen::MatrixXd::Identity(n * 3, n * 3) * measurement_noise_;
        }
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;

        state_(3) = std::max(0.0, std::min(M_PI, state_(3)));
        if (state_(4) > M_PI)
            state_(4) -= 2.0 * M_PI;
        if (state_(4) < -M_PI)
            state_(4) += 2.0 * M_PI;
    }

    // publish and ROS service are removed; node handles them

} // namespace mag_pose_estimation
