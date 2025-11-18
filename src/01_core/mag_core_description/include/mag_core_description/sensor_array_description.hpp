#pragma once

#include <mag_core_description/sensor_array_config_loader.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace mag_core_description
{

struct SensorEntry
{
    int id{0};
    geometry_msgs::Pose pose;
    std::string frame_id;
};

class SensorArrayDescription
{
public:
    void load(const SensorArrayConfig &config);

    const std::string &parentFrame() const { return parent_frame_; }
    const std::string &arrayFrame() const { return array_frame_; }
    const geometry_msgs::Pose &arrayPose() const { return array_pose_; }
    const std::vector<SensorEntry> &sensors() const { return sensors_; }
    const SensorEntry *findSensor(int id) const;
    std::string sensorFrameName(int id) const;

    const std::string &sensorFramePrefix() const { return sensor_frame_prefix_; }

private:
    static geometry_msgs::Pose poseFromXyzRpy(const std::vector<double> &xyz, const std::vector<double> &rpy);

    std::string parent_frame_;
    std::string array_frame_;
    geometry_msgs::Pose array_pose_;
    std::vector<SensorEntry> sensors_;
    std::unordered_map<int, std::size_t> index_by_id_;
    std::string sensor_frame_prefix_ = "sensor_";
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

} // namespace mag_core_description
