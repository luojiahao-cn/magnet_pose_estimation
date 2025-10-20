#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mag_sensor_node/MagnetPose.h>
#include <mag_viz/magnet_viz_node.hpp>

#include <string>

MagnetPoseViz::MagnetPoseViz(ros::NodeHandle &nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    if (!nh_.getParam("topic", topic_))
        throw std::runtime_error("缺少参数: ~topic");
    if (!nh_.getParam("frame", target_frame_))
        throw std::runtime_error("缺少参数: ~frame");
    if (!nh_.getParam("marker_topic", marker_topic_))
        throw std::runtime_error("缺少参数: ~marker_topic");
    if (!nh_.getParam("marker_lifetime", marker_lifetime_))
        throw std::runtime_error("缺少参数: ~marker_lifetime");
    if (!nh_.getParam("magnet_scale", magnet_scale_))
        throw std::runtime_error("缺少参数: ~magnet_scale");

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    sub_ = nh_.subscribe(topic_, 10, &MagnetPoseViz::onMsg, this);

    ROS_INFO_STREAM("[magnet_pose_viz] subscribing: " << topic_ << ", publishing: " << marker_topic_
                                    << ", target_frame: " << target_frame_);
}

void MagnetPoseViz::onMsg(const mag_sensor_node::MagnetPose::ConstPtr &msg)
{
    visualization_msgs::MarkerArray arr;

    // 获取磁铁位姿
    geometry_msgs::Pose pose_in;
    pose_in.position = msg->position;
    pose_in.orientation = msg->orientation;

    geometry_msgs::Pose pose_tf = pose_in;
    std::string frame_id = target_frame_.empty() ? msg->header.frame_id : target_frame_;
    try
    {
        if (!target_frame_.empty() && target_frame_ != msg->header.frame_id)
        {
            geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
            tf2::doTransform(pose_in, pose_tf, T);
        }
    }
    catch (const std::exception &e)
    {
        frame_id = msg->header.frame_id;
        ROS_WARN_THROTTLE(5.0,
                          "TF to target_frame '%s' unavailable (e.g., %s). Falling back to source frame '%s'.",
                          target_frame_.c_str(), e.what(), msg->header.frame_id.c_str());
    }

    // 磁铁正极（红色半圆柱体）
    visualization_msgs::Marker north_pole;
    north_pole.header.stamp = msg->header.stamp;
    north_pole.header.frame_id = frame_id;
    north_pole.ns = "magnet";
    north_pole.id = 0;
    north_pole.type = visualization_msgs::Marker::CYLINDER;
    north_pole.action = visualization_msgs::Marker::ADD;
    north_pole.scale.x = magnet_scale_ * 0.5; // 直径 - 更细
    north_pole.scale.y = magnet_scale_ * 0.5; // 直径 - 更细
    north_pole.scale.z = magnet_scale_ * 0.25; // 高度 - 半高度
    north_pole.lifetime = ros::Duration(marker_lifetime_);

    north_pole.pose = pose_tf;
    north_pole.pose.position.z += magnet_scale_ * 0.125; // 上移半高度

    north_pole.color.r = 1.0; // 红色
    north_pole.color.g = 0.0;
    north_pole.color.b = 0.0;
    north_pole.color.a = 0.9;

    arr.markers.push_back(north_pole);

    // 磁铁负极（蓝色半圆柱体）
    visualization_msgs::Marker south_pole;
    south_pole.header = north_pole.header;
    south_pole.ns = "magnet";
    south_pole.id = 1;
    south_pole.type = visualization_msgs::Marker::CYLINDER;
    south_pole.action = visualization_msgs::Marker::ADD;
    south_pole.scale = north_pole.scale; // 相同大小
    south_pole.lifetime = ros::Duration(marker_lifetime_);

    south_pole.pose = pose_tf;
    south_pole.pose.position.z -= magnet_scale_ * 0.125; // 下移半高度

    south_pole.color.r = 0.0; // 蓝色
    south_pole.color.g = 0.0;
    south_pole.color.b = 1.0;
    south_pole.color.a = 0.9;

    arr.markers.push_back(south_pole);

    // 磁场强度文本标签
    double strength = msg->magnetic_strength;
    visualization_msgs::Marker strength_text;
    strength_text.header = north_pole.header;
    strength_text.ns = "magnet";
    strength_text.id = 2; // 文本标签ID改为2
    strength_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    strength_text.action = visualization_msgs::Marker::ADD;
    strength_text.scale.z = magnet_scale_ * 0.3; // 文本大小
    strength_text.lifetime = ros::Duration(marker_lifetime_);

    strength_text.pose = north_pole.pose;
    strength_text.pose.position.z += magnet_scale_ * 0.4; // 在磁铁上方显示文本

    char text_buf[64];
    snprintf(text_buf, sizeof(text_buf), "%.2f T", strength);
    strength_text.text = text_buf;

    strength_text.color.r = 1.0;
    strength_text.color.g = 1.0;
    strength_text.color.b = 1.0;
    strength_text.color.a = 1.0;

    arr.markers.push_back(strength_text);

    pub_.publish(arr);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnet_pose_viz");
    ros::NodeHandle nh("~");
    MagnetPoseViz node(nh);
    ros::spin();
    return 0;
}
