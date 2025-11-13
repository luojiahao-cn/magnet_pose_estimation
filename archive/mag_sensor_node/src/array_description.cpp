#include <mag_sensor_node/array_description.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdexcept>

namespace mag_sensor_node
{
namespace
{
    geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::Pose &pose,
                                                     const std::string &parent,
                                                     const std::string &child,
                                                     const ros::Time &stamp)
    {
        // 将 Pose 转换为标准 TransformStamped，便于统一广播
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
}

void SensorArrayDescription::load(const ros::NodeHandle &nh, const std::string &key)
{
    XmlRpc::XmlRpcValue root;
    if (!nh.getParam(key, root))
    {
        throw std::runtime_error("未在参数服务器找到键 '" + key + "'");
    }
    if (root.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        throw std::runtime_error("参数 '" + key + "' 必须是字典结构");
    }

    if (!root.hasMember("parent_frame") || !root.hasMember("frame"))
    {
        throw std::runtime_error("array 配置缺少 parent_frame 或 frame 字段");
    }
    parent_frame_ = static_cast<std::string>(root["parent_frame"]);
    array_frame_ = static_cast<std::string>(root["frame"]);

    if (root.hasMember("sensor_frame_prefix"))
    {
        sensor_frame_prefix_ = static_cast<std::string>(root["sensor_frame_prefix"]);
    }

    if (root.hasMember("tf_publish_rate"))
    {
        tf_publish_rate_ = asDouble(root["tf_publish_rate"], "array.tf_publish_rate");
    }

    if (!root.hasMember("pose"))
    {
        throw std::runtime_error("array 配置缺少 pose 字段");
    }
    XmlRpc::XmlRpcValue pose_node = root["pose"];
    if (pose_node.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !pose_node.hasMember("xyz") || !pose_node.hasMember("rpy"))
    {
        throw std::runtime_error("array.pose 必须包含 xyz 与 rpy");
    }
    const auto xyz = asVector3(pose_node["xyz"], "array.pose.xyz");
    const auto rpy = asVector3(pose_node["rpy"], "array.pose.rpy");
    array_pose_ = buildPose(xyz, rpy);

    if (!root.hasMember("sensors"))
    {
        throw std::runtime_error("array 配置缺少 sensors 数组");
    }
    XmlRpc::XmlRpcValue sensor_block = root["sensors"];
    if (sensor_block.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        throw std::runtime_error("array.sensors 必须是数组");
    }

    sensors_.clear();
    index_by_id_.clear();
    sensors_.reserve(sensor_block.size());

    for (int i = 0; i < sensor_block.size(); ++i)
    {
        XmlRpc::XmlRpcValue sensor = sensor_block[i];
        if (sensor.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            throw std::runtime_error("array.sensors[" + std::to_string(i) + "] 必须是字典");
        }
        if (!sensor.hasMember("id"))
        {
            throw std::runtime_error("array.sensors[" + std::to_string(i) + "] 缺少 id");
        }
        int id = static_cast<int>(sensor["id"]);
        if (index_by_id_.count(id) > 0)
        {
            throw std::runtime_error("检测到重复的传感器 id: " + std::to_string(id));
        }

        if (!sensor.hasMember("pose"))
        {
            throw std::runtime_error("array.sensors[" + std::to_string(i) + "] 缺少 pose");
        }
        XmlRpc::XmlRpcValue pose_node = sensor["pose"];
        if (pose_node.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !pose_node.hasMember("xyz") || !pose_node.hasMember("rpy"))
        {
            throw std::runtime_error("array.sensors[" + std::to_string(i) + "] pose 字段非法");
        }
        const auto sensor_xyz = asVector3(pose_node["xyz"], "array.sensors.pose.xyz");
        const auto sensor_rpy = asVector3(pose_node["rpy"], "array.sensors.pose.rpy");

        SensorEntry entry;
        entry.id = id;
        entry.pose = buildPose(sensor_xyz, sensor_rpy);
        if (sensor.hasMember("frame"))
        {
            entry.frame_id = static_cast<std::string>(sensor["frame"]);
        }
        else
        {
            entry.frame_id = sensor_frame_prefix_ + std::to_string(id);
        }

        index_by_id_[entry.id] = sensors_.size();
        sensors_.push_back(entry);
    }

    if (sensors_.empty())
    {
        throw std::runtime_error("array.sensors 不能为空");
    }
}

const SensorEntry *SensorArrayDescription::findSensor(int id) const
{
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end())
    {
        return nullptr;
    }
    return &sensors_[it->second];
}

std::string SensorArrayDescription::sensorFrameName(int id) const
{
    auto *entry = findSensor(id);
    if (!entry)
    {
        return sensor_frame_prefix_ + std::to_string(id);
    }
    return entry->frame_id;
}

double SensorArrayDescription::asDouble(const XmlRpc::XmlRpcValue &value, const std::string &context)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(value);
    }
    throw std::runtime_error(context + " 需为数字类型");
}

std::vector<double> SensorArrayDescription::asVector3(const XmlRpc::XmlRpcValue &value, const std::string &context)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
    {
        throw std::runtime_error(context + " 需为长度为 3 的数组");
    }
    std::vector<double> out(3);
    for (int i = 0; i < 3; ++i)
    {
        out[i] = asDouble(value[i], context + "[" + std::to_string(i) + "]");
    }
    return out;
}

geometry_msgs::Pose SensorArrayDescription::buildPose(const std::vector<double> &xyz, const std::vector<double> &rpy)
{
    geometry_msgs::Pose pose;
    pose.position.x = xyz[0];
    pose.position.y = xyz[1];
    pose.position.z = xyz[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.orientation = tf2::toMsg(q);
    return pose;
}

SensorArrayTfPublisher::SensorArrayTfPublisher(const SensorArrayDescription &description)
    : description_(description)
{
}

std::vector<geometry_msgs::TransformStamped> SensorArrayTfPublisher::buildTransforms(const ros::Time &stamp) const
{
    std::vector<geometry_msgs::TransformStamped> tfs;
    tfs.reserve(description_.sensors().size() + 1);
    tfs.push_back(poseToTransform(description_.arrayPose(),
                                  description_.parentFrame(),
                                  description_.arrayFrame(),
                                  stamp));
    for (const auto &sensor : description_.sensors())
    {
        tfs.push_back(poseToTransform(sensor.pose,
                                      description_.arrayFrame(),
                                      sensor.frame_id,
                                      stamp));
    }
    return tfs;
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

} // namespace mag_sensor_node
