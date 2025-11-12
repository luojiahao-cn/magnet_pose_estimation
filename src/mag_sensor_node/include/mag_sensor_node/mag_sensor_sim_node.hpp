#pragma once

#include <geometry_msgs/Point.h>
#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
    void initializeMotionSystem();
    void onTimer(const ros::TimerEvent &);
    void updateMagnetPose(magnet_msgs::MagnetPose &magnet_pose);
    void updateMagnetPosition(magnet_msgs::MagnetPose &magnet_pose);
    void updateMagnetOrientation(magnet_msgs::MagnetPose &magnet_pose);
    void calculateDynamicOrientation(double &roll, double &pitch, double &yaw);
    geometry_msgs::Point calculatePositionFromVelocity(double elapsed_time);
    void publishSensorMagneticFields(const magnet_msgs::MagnetPose &magnet_pose);

    enum class MotionType
    {
        STATIC,
        TRANSLATE_WITH_PATH,
        TRANSLATE_STATIONARY,
        CENTER_FIXED
    };

    MotionType getMotionType() const;

    ros::NodeHandle nh_;
    ros::Publisher magnet_pose_pub_;
    ros::Publisher magnetic_field_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string parent_frame_;
    std::string child_frame_;
    double magnet_strength_{};
    Eigen::Vector3d magnetic_direction_;
    double rect_width_{};
    double rect_height_{};
    double rect_z_{};
    double x_center_{};
    double y_center_{};
    double update_rate_{};
    double translation_velocity_{};
    double path_total_length_{};
    ros::Time start_time_;
    geometry_msgs::Point current_position_;
    std::string motion_mode_;
    std::string rotation_axis_;
    double angular_velocity_{};
    double initial_roll_{};
    double initial_pitch_{};
    double initial_yaw_{};
    Eigen::Vector3d static_position_;
    Eigen::Vector3d static_orientation_;
    bool translation_enable_{true};
    bool rotation_enable_{false};
    bool follow_path_{true};
    bool noise_enable_{false};
    std::string noise_type_;
    double noise_mean_{};
    double noise_stddev_{};
    double noise_amplitude_{};
    std::string frame_id_;
};