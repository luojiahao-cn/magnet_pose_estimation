#pragma once

#include <geometry_msgs/Pose.h>

#include <XmlRpcValue.h>

#include <string>
#include <vector>

namespace mag_core_description
{

struct SensorArraySensorConfig
{
    int id{0};
    geometry_msgs::Pose pose;
    std::string frame_id;
};

struct SensorArrayConfig
{
    std::string parent_frame;
    std::string array_frame;
    std::string sensor_frame_prefix{"sensor_"};
    geometry_msgs::Pose array_pose;
    std::vector<SensorArraySensorConfig> sensors;
};

SensorArrayConfig loadSensorArrayConfig(const XmlRpc::XmlRpcValue &root,
                                        const std::string &context);

} // namespace mag_core_description
