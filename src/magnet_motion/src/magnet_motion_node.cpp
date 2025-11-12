#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include <magnet_motion/magnet_motion_node.hpp>

MagnetMotionNode::MagnetMotionNode(ros::NodeHandle &nh) : nh_(nh)
{
    ros::NodeHandle pnh("~");
    loadParams(pnh);
    start_time_ = ros::Time::now();
    current_position_.x = x_center_ - rect_width_ / 2.0;
    current_position_.y = y_center_ - rect_height_ / 2.0;
    current_position_.z = rect_z_;
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MagnetMotionNode::onTimer, this);
}

void MagnetMotionNode::loadParams(ros::NodeHandle &pnh)
{
    // local small helpers to keep this file self-contained
    auto require = [&](const std::string &key, auto &var) {
        if (!pnh.getParam(key, var))
            throw std::runtime_error("缺少必需参数: " + key);
    };
    auto requireVec3 = [&](const std::string &key, Eigen::Vector3d &out) {
        std::vector<double> v;
        if (!pnh.getParam(key, v) || v.size() != 3)
            throw std::runtime_error("缺少或非法参数: " + key);
        out = Eigen::Vector3d(v[0], v[1], v[2]);
    };

    // required TF frames
    require("magnet_motion/motion/parent_frame", parent_frame_);
    require("magnet_motion/motion/child_frame", child_frame_);

    // path parameters
    require("magnet_motion/path/width", rect_width_);
    require("magnet_motion/path/height", rect_height_);
    require("magnet_motion/path/z", rect_z_);
    require("magnet_motion/path/center/x", x_center_);
    require("magnet_motion/path/center/y", y_center_);
    require("magnet_motion/path/update_rate", update_rate_);
    require("magnet_motion/path/velocity", translation_velocity_);

    // motion parameters
    require("magnet_motion/motion/mode", motion_mode_);
    require("magnet_motion/motion/axis", rotation_axis_);
    require("magnet_motion/motion/angular_velocity", angular_velocity_);
    require("magnet_motion/motion/initial_roll", initial_roll_);
    require("magnet_motion/motion/initial_pitch", initial_pitch_);
    require("magnet_motion/motion/initial_yaw", initial_yaw_);

    requireVec3("magnet_motion/motion/static_position", static_position_);
    requireVec3("magnet_motion/motion/static_orientation", static_orientation_);

    // map motion mode to internal flags
    translation_enable_ = false;
    rotation_enable_ = false;
    follow_path_ = true;
    if (motion_mode_ == "static")
    {
        translation_enable_ = false;
        rotation_enable_ = false;
    }
    else if (motion_mode_ == "translate_only")
    {
        translation_enable_ = true;
        rotation_enable_ = false;
        follow_path_ = true;
    }
    else if (motion_mode_ == "rotate_only")
    {
        translation_enable_ = false;
        rotation_enable_ = true;
    }
    else if (motion_mode_ == "translate_and_rotate")
    {
        translation_enable_ = true;
        rotation_enable_ = true;
        follow_path_ = true;
    }
    else
    {
        throw std::runtime_error("非法 motion mode: " + motion_mode_);
    }

    path_total_length_ = 2.0 * (rect_width_ + rect_height_);
}

void MagnetMotionNode::onTimer(const ros::TimerEvent &)
{
    ros::Time now = ros::Time::now();

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;

    updateMagnetPose(tf_msg, now);

    // publish TF
    tf_broadcaster_.sendTransform(tf_msg);
}

void MagnetMotionNode::updateMagnetPose(geometry_msgs::TransformStamped &tf_msg, ros::Time now)
{
    // Fill translation + rotation in tf_msg based on configured motion
    geometry_msgs::Point p;
    // compute position
    switch (translation_enable_ ? (follow_path_ ? 1 : 2) : 0)
    {
    case 0: // center fixed or static
        if (motion_mode_ == "static")
        {
            p.x = static_position_.x();
            p.y = static_position_.y();
            p.z = static_position_.z();
        }
        else
        {
            p.x = x_center_;
            p.y = y_center_;
            p.z = rect_z_;
        }
        break;
    case 1: // translate with path
    {
        double elapsed_time = (now - start_time_).toSec();
        p = calculatePositionFromVelocity(elapsed_time);
        break;
    }
    case 2: // translate stationary
        p.x = x_center_ - rect_width_ / 2.0;
        p.y = y_center_ - rect_height_ / 2.0;
        p.z = rect_z_;
        break;
    }

    tf_msg.transform.translation.x = p.x;
    tf_msg.transform.translation.y = p.y;
    tf_msg.transform.translation.z = p.z;

    // compute orientation
    double roll, pitch, yaw;
    if (motion_mode_ == "static")
    {
        roll = static_orientation_.x();
        pitch = static_orientation_.y();
        yaw = static_orientation_.z();
    }
    else if (rotation_enable_)
    {
        calculateDynamicOrientation(roll, pitch, yaw);
    }
    else
    {
        roll = initial_roll_;
        pitch = initial_pitch_;
        yaw = initial_yaw_;
    }
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    tf_msg.transform.rotation = tf2::toMsg(q);
}

void MagnetMotionNode::calculateDynamicOrientation(double &roll, double &pitch, double &yaw)
{
    static ros::Time rotation_start_time = ros::Time::now();
    double elapsed_time = (ros::Time::now() - rotation_start_time).toSec();
    double rotation_angle = elapsed_time * angular_velocity_;
    roll = initial_roll_;
    pitch = initial_pitch_;
    yaw = initial_yaw_;
    if (rotation_axis_ == "x" || rotation_axis_ == "xyz")
        roll += rotation_angle;
    if (rotation_axis_ == "y" || rotation_axis_ == "xyz")
        pitch += rotation_angle;
    if (rotation_axis_ == "z" || rotation_axis_ == "xyz")
        yaw += rotation_angle;
}

geometry_msgs::Point MagnetMotionNode::calculatePositionFromVelocity(double elapsed_time)
{
    geometry_msgs::Point position;
    double distance_traveled = translation_velocity_ * elapsed_time;
    double distance_on_path = fmod(distance_traveled, path_total_length_);
    double bottom_edge = rect_width_;
    double right_edge = rect_height_;
    double top_edge = rect_width_;
    double left_edge = rect_height_;
    double left = x_center_ - rect_width_ / 2.0;
    double right = x_center_ + rect_width_ / 2.0;
    double bottom = y_center_ - rect_height_ / 2.0;
    double top = y_center_ + rect_height_ / 2.0;
    position.z = rect_z_;
    if (distance_on_path <= bottom_edge)
    {
        double ratio = distance_on_path / bottom_edge;
        position.x = left + ratio * rect_width_;
        position.y = bottom;
    }
    else if (distance_on_path <= bottom_edge + right_edge)
    {
        double ratio = (distance_on_path - bottom_edge) / right_edge;
        position.x = right;
        position.y = bottom + ratio * rect_height_;
    }
    else if (distance_on_path <= bottom_edge + right_edge + top_edge)
    {
        double ratio = (distance_on_path - bottom_edge - right_edge) / top_edge;
        position.x = right - ratio * rect_width_;
        position.y = top;
    }
    else
    {
        double ratio = (distance_on_path - bottom_edge - right_edge - top_edge) / left_edge;
        position.x = left;
        position.y = top - ratio * rect_height_;
    }
    return position;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnet_motion_node");
    ros::NodeHandle nh;
    try
    {
        MagnetMotionNode node(nh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("magnet_motion_node 退出: %s", e.what());
        return 1;
    }
    return 0;
}
