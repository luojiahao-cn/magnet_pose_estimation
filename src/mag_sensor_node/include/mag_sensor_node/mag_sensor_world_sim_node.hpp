#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <mutex>
#include <string>
#include <vector>

namespace mag_sensor_node
{
    class SensorConfig;
}

class MagSensorWorldSimNode
{
public:
    explicit MagSensorWorldSimNode(ros::NodeHandle &nh);

private:
    enum class MotionType
    {
        STATIC,
        TRANSLATE_WITH_PATH,
        TRANSLATE_STATIONARY,
        CENTER_FIXED
    };

    enum class ArrayPoseSource
    {
        STATIC,
        TOPIC
    };

    static Eigen::MatrixXd computeMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                const Eigen::Vector3d &magnetic_position,
                                                const Eigen::Vector3d &magnetic_direction,
                                                double magnetic_moment_size);

    void loadParams();
    void setupPublishers();
    void initializeMotionSystem();
    void onTimer(const ros::TimerEvent &event);
    void updateMagnetPose(magnet_msgs::MagnetPose &magnet_pose);
    void updateMagnetPosition(magnet_msgs::MagnetPose &magnet_pose);
    void updateMagnetOrientation(magnet_msgs::MagnetPose &magnet_pose);
    void calculateDynamicOrientation(double &roll, double &pitch, double &yaw);
    geometry_msgs::Point calculatePositionFromVelocity(double elapsed_time);
    MotionType getMotionType() const;
    void publishSensorMagneticFields(const magnet_msgs::MagnetPose &magnet_pose);
    bool getCurrentArrayPose(geometry_msgs::Pose &pose) const;
    void handleArrayPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void publishArrayTf(const geometry_msgs::Pose &array_pose_world, const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::Publisher magnet_pose_pub_;
    ros::Publisher magnetic_field_pub_;
    ros::Timer timer_;
    ros::Subscriber array_pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string world_frame_id_;
    std::string array_frame_id_{"sensor_array"};

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

    ArrayPoseSource array_pose_source_{ArrayPoseSource::STATIC};
    mutable std::mutex array_pose_mutex_;
    geometry_msgs::Pose latest_array_pose_;
    ros::Time latest_array_pose_stamp_;
    ros::Duration array_pose_timeout_{0.0};
    bool array_pose_ready_{false};
    bool publish_tf_{false};
};
