#include <mag_core_description/sensor_array_description.hpp>

#include <stdexcept>
#include <string>

namespace mag_core_description
{
namespace
{
geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::Pose &pose,
                                                const std::string &parent,
                                                const std::string &child,
                                                const ros::Time &stamp)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    return tf;
}

} // namespace

void SensorArrayDescription::load(const SensorArrayConfig &config)
{
    parent_frame_ = config.parent_frame;
    array_frame_ = config.array_frame;
    array_pose_ = config.array_pose;
    if (!config.sensor_frame_prefix.empty())
    {
        sensor_frame_prefix_ = config.sensor_frame_prefix;
    }

    sensors_.clear();
    index_by_id_.clear();
    sensors_.reserve(config.sensors.size());

    for (const auto &sensor_cfg : config.sensors)
    {
        if (index_by_id_.count(sensor_cfg.id) != 0)
        {
            throw std::runtime_error("duplicate sensor id: " + std::to_string(sensor_cfg.id));
        }

        SensorEntry entry;
        entry.id = sensor_cfg.id;
        entry.pose = sensor_cfg.pose;
        entry.frame_id = sensor_cfg.frame_id.empty()
                             ? sensor_frame_prefix_ + std::to_string(sensor_cfg.id)
                             : sensor_cfg.frame_id;

        index_by_id_.emplace(entry.id, sensors_.size());
        sensors_.push_back(entry);
    }

    if (sensors_.empty())
    {
        throw std::runtime_error("sensor array must contain at least one sensor");
    }
}

const SensorEntry *SensorArrayDescription::findSensor(int id) const
{
    const auto it = index_by_id_.find(id);
    if (it == index_by_id_.end())
    {
        return nullptr;
    }
    return &sensors_.at(it->second);
}

std::string SensorArrayDescription::sensorFrameName(int id) const
{
    const auto *entry = findSensor(id);
    if (!entry)
    {
        return sensor_frame_prefix_ + std::to_string(id);
    }
    return entry->frame_id;
}

SensorArrayTfPublisher::SensorArrayTfPublisher(const SensorArrayDescription &description)
    : description_(description)
{
}

std::vector<geometry_msgs::TransformStamped> SensorArrayTfPublisher::buildTransforms(const ros::Time &stamp) const
{
    std::vector<geometry_msgs::TransformStamped> result;
    result.reserve(description_.sensors().size() + 1);

    result.push_back(poseToTransform(description_.arrayPose(),
                                     description_.parentFrame(),
                                     description_.arrayFrame(),
                                     stamp));

    for (const auto &sensor : description_.sensors())
    {
        result.push_back(poseToTransform(sensor.pose,
                                         description_.arrayFrame(),
                                         sensor.frame_id,
                                         stamp));
    }
    return result;
}

void SensorArrayTfPublisher::publishDynamic(const ros::Time &stamp)
{
    dynamic_broadcaster_.sendTransform(buildTransforms(stamp));
}

void SensorArrayTfPublisher::publishStatic()
{
    if (static_sent_)
    {
        return;
    }
    static_broadcaster_.sendTransform(buildTransforms(ros::Time::now()));
    static_sent_ = true;
}

} // namespace mag_core_description
