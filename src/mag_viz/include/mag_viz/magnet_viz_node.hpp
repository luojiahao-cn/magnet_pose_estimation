#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mag_sensor_node/MagnetPose.h>

#include <string>

struct Color { double r, g, b, a; };

class MagnetPoseViz
{
public:
    explicit MagnetPoseViz(ros::NodeHandle &nh);

private:
    void onMsg(const mag_sensor_node::MagnetPose::ConstPtr &msg);

    ros::NodeHandle &nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string topic_;
    std::string target_frame_;
    std::string marker_topic_;
    double marker_lifetime_;
    double magnet_scale_;
};