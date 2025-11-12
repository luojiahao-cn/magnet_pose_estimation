#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mag_sensor_node/array_description.hpp>

#include <magnet_msgs/MagSensorData.h>

#include <string>

struct Color { double r, g, b, a; };

class MagSensorViz
{
public:
    explicit MagSensorViz(ros::NodeHandle &nh);

private:
    static Color lerp(const Color &a, const Color &b, double t);
    Color colormap(double mag) const;
    void onMsg(const magnet_msgs::MagSensorData::ConstPtr &msg);
    ros::NodeHandle &nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    mag_sensor_node::SensorArrayDescription sensor_array_;
    bool sensor_array_loaded_{false};

    std::string topic_;
    std::string target_frame_;
    std::string marker_topic_;
    double field_scale_;
    double marker_lifetime_;
    double color_max_;
    Color zero_color_;
};