#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <mag_pose_estimation/mag_field_calculator.hpp>
#include <mag_pose_estimation/mag_pose_estimator_base.hpp>
#include <map>

namespace mag_pose_estimation
{
    class KalmanMagnetPoseEstimator : public BaseMagnetPoseEstimator
    {
    public:
        // Expect a node's private NodeHandle; parameters are read from "~estimator_config/*"
        explicit KalmanMagnetPoseEstimator(ros::NodeHandle &pnh);
        void reset() override;
        bool estimate(const std::map<int, MagneticField> &measurements, MagnetPose &out_pose, double *out_error) override;

    private:
        void loadParameters();
        void predict();
        void update(const Eigen::VectorXd &measurement, const Eigen::MatrixXd &sensor_positions);
    // Private node handle for this node ("~")
    ros::NodeHandle pnh_;

        Eigen::VectorXd state_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd R_;
        double measurement_noise_{1e-2};

        Eigen::Vector3d initial_position_;
        Eigen::Vector3d initial_direction_;
        double initial_strength_;

        int state_dim_;
    };

} // namespace mag_pose_estimation
