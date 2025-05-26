#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <map>
#include <std_srvs/Empty.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/magnet_pose_estimator_base.hpp>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace magnetic_pose_estimation
{

    class KalmanMagnetPoseEstimator : public BaseMagnetPoseEstimator
    {
    public:
        explicit KalmanMagnetPoseEstimator(ros::NodeHandle &nh);

        void magneticFieldCallback(const MagneticField::ConstPtr &msg) override;
        bool resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) override;

    private:
        void loadParameters();
        void predict();
        void update(const Eigen::VectorXd &measurement);
        void publishMagnetPose();

        ros::NodeHandle nh_;
        ros::Publisher magnet_pose_pub_;
        ros::Subscriber magnetic_field_sub_;
        ros::ServiceServer reset_localization_service_;

        std::map<int, MagneticField> measurements_;

        // 卡尔曼滤波相关变量
        Eigen::VectorXd state_; // 状态向量
        Eigen::MatrixXd P_;     // 协方差矩阵
        Eigen::MatrixXd Q_;     // 过程噪声
        Eigen::MatrixXd R_;     // 测量噪声

        Eigen::Vector3d initial_position_;
        Eigen::Vector3d initial_direction_;
        double initial_strength_;

        int state_dim_;
    };

} // namespace magnetic_pose_estimation