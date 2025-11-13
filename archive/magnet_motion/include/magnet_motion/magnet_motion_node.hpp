#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <string>

class MagnetMotionNode
{
public:
    explicit MagnetMotionNode(ros::NodeHandle &nh);

private:
    void loadParams(ros::NodeHandle &pnh);
    void onTimer(const ros::TimerEvent &);
    void updateMagnetPose(geometry_msgs::TransformStamped &tf_msg, ros::Time now);
    void updateMagnetPosition(geometry_msgs::TransformStamped &tf_msg);
    void updateMagnetOrientation(geometry_msgs::TransformStamped &tf_msg);
    void calculateDynamicOrientation(double &roll, double &pitch, double &yaw);
    geometry_msgs::Point calculatePositionFromVelocity(double elapsed_time);

    ros::NodeHandle nh_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Timer timer_;

    // params
    std::string parent_frame_;
    std::string child_frame_;
    double rect_width_ = 0.2;
    double rect_height_ = 0.2;
    double rect_z_ = 0.0;
    double x_center_ = 0.0;
    double y_center_ = 0.0;
    double update_rate_ = 30.0;
    double translation_velocity_ = 0.1;
    std::string motion_mode_ = "static";
    std::string rotation_axis_ = "z";
    double angular_velocity_ = 0.0;
    double initial_roll_ = 0.0;
    double initial_pitch_ = 0.0;
    double initial_yaw_ = 0.0;
    Eigen::Vector3d static_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d static_orientation_ = Eigen::Vector3d::Zero();
    bool translation_enable_ = false;
    bool rotation_enable_ = false;
    bool follow_path_ = true;
    double path_total_length_ = 0.0;
    ros::Time start_time_;
    geometry_msgs::Point current_position_;
};
