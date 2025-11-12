#pragma once

#include <geometry_msgs/Point.h>
#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

class MagSensorSimNode
{
public:
    explicit MagSensorSimNode(ros::NodeHandle &nh);

private:
    static Eigen::MatrixXd computeMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                const Eigen::Vector3d &magnetic_position,
                                                const Eigen::Vector3d &magnetic_direction,
                                                double magnetic_moment_size);

    void loadParams();
    void setupPublishers();
    void onTimer(const ros::TimerEvent &);
    void publishSensorMagneticFields(const magnet_msgs::MagnetPose &magnet_pose);
    

    ros::NodeHandle nh_;
    ros::Publisher magnetic_field_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string parent_frame_;
    std::string child_frame_;
    double magnet_strength_{};
    double update_rate_{};
    Eigen::Vector3d magnetic_direction_;
    bool noise_enable_{false};
    std::string noise_type_;
    double noise_mean_{};
    double noise_stddev_{};
    double noise_amplitude_{};
    std::string frame_id_;
    std::string array_parent_frame_;
};