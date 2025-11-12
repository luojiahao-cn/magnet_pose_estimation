#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <XmlRpcValue.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace mag_sensor_node
{

struct SensorEntry
{
    int id{0};
    geometry_msgs::Pose pose;      // 在阵列坐标系下的位姿
    std::string frame_id;          // 传感器对应的 TF 名称
};

class SensorArrayDescription
{
public:
    void load(const ros::NodeHandle &nh, const std::string &key = "array");

    const std::string &parentFrame() const { return parent_frame_; }
    const std::string &arrayFrame() const { return array_frame_; }
    const geometry_msgs::Pose &arrayPose() const { return array_pose_; }
    const std::vector<SensorEntry> &sensors() const { return sensors_; }
    const SensorEntry *findSensor(int id) const;
    std::string sensorFrameName(int id) const;

    double tfPublishRate() const { return tf_publish_rate_; }
    const std::string &sensorFramePrefix() const { return sensor_frame_prefix_; }

private:
    static double asDouble(const XmlRpc::XmlRpcValue &value, const std::string &context);
    static std::vector<double> asVector3(const XmlRpc::XmlRpcValue &value, const std::string &context);
    static geometry_msgs::Pose buildPose(const std::vector<double> &xyz, const std::vector<double> &rpy);

    std::string parent_frame_;
    std::string array_frame_;
    geometry_msgs::Pose array_pose_;
    std::vector<SensorEntry> sensors_;
    std::unordered_map<int, size_t> index_by_id_;
    std::string sensor_frame_prefix_ = "sensor_";
    double tf_publish_rate_{30.0};
};

class SensorArrayTfPublisher
{
public:
    explicit SensorArrayTfPublisher(const SensorArrayDescription &description);

    void publishDynamic(const ros::Time &stamp);
    void publishStatic();

private:
    std::vector<geometry_msgs::TransformStamped> buildTransforms(const ros::Time &stamp) const;

    const SensorArrayDescription &description_;
    tf2_ros::TransformBroadcaster dynamic_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    bool static_sent_{false};
};

} // namespace mag_sensor_node
