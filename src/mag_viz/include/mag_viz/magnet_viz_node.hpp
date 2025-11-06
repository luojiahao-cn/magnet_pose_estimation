#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <mag_sensor_node/MagnetPose.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

class MagnetPoseViz
{
public:
    explicit MagnetPoseViz(ros::NodeHandle &nh);

private:
    struct SourceConfig
    {
        std::string topic;
        std::string ns;
        std::string label;
    };

    void onMsg(const mag_sensor_node::MagnetPose::ConstPtr &msg, std::size_t source_index);

    ros::NodeHandle &nh_;
    std::vector<ros::Subscriber> subs_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<SourceConfig> sources_;
    std::string target_frame_;
    std::string marker_topic_;
    double marker_lifetime_{};
    double magnet_scale_{};
};