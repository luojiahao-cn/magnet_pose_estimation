#include <mag_core_description/sensor_array_description.hpp>

#include <mag_core_utils/param_reader.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    // 将阵列或传感器相对父坐标系的姿态转换为 TransformStamped
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

void SensorArrayDescription::load(const ros::NodeHandle &nh, const std::string &key)
{
    using mag_core_utils::param::StructReader;

    // 读取并校验阵列配置的根结构
    auto root = StructReader::fromParameter(nh, key);

    // 记录父坐标系与阵列自身坐标系名称
    parent_frame_ = root.requireString("parent_frame");
    array_frame_ = root.requireString("frame");

    // 允许自定义传感器帧前缀
    sensor_frame_prefix_ = root.optionalString("sensor_frame_prefix", sensor_frame_prefix_);

    // 解析阵列在父坐标系下的位姿
    const auto pose_node = root.childStruct("pose");
    const auto array_xyz = pose_node.requireVector3("xyz");
    const auto array_rpy = pose_node.requireVector3("rpy");
    array_pose_ = poseFromXyzRpy(array_xyz, array_rpy);

    // 遍历传感器列表并构建数组
    const auto sensors_array = root.childArray("sensors");

    sensors_.clear();
    index_by_id_.clear();
    sensors_.reserve(sensors_array.size());

    for (int i = 0; i < sensors_array.size(); ++i)
    {
        const auto sensor_node = sensors_array.structAt(i);
        const int id = sensor_node.requireInt("id");
        if (index_by_id_.count(id) != 0)
        {
            throw std::runtime_error("duplicate sensor id " + std::to_string(id));
        }

        // 解析单个传感器在阵列坐标系下的位姿
        const auto pose = sensor_node.childStruct("pose");
        const auto sensor_xyz = pose.requireVector3("xyz");
        const auto sensor_rpy = pose.requireVector3("rpy");

        SensorEntry entry;
        entry.id = id;
        entry.pose = poseFromXyzRpy(sensor_xyz, sensor_rpy);
        entry.frame_id = sensor_node.has("frame")
                              ? sensor_node.requireString("frame")
                              : sensor_frame_prefix_ + std::to_string(id);

        // 建立 ID 到数组索引的映射，便于快速查询
        index_by_id_.emplace(entry.id, sensors_.size());
        sensors_.push_back(entry);
    }

    if (sensors_.empty())
    {
        throw std::runtime_error("parameter '" + root.context() + ".sensors' must not be empty");
    }
}

const SensorEntry *SensorArrayDescription::findSensor(int id) const
{
    // 根据传感器 ID 查找数组中的信息
    const auto it = index_by_id_.find(id);
    if (it == index_by_id_.end())
    {
        return nullptr;
    }
    return &sensors_.at(it->second);
}

std::string SensorArrayDescription::sensorFrameName(int id) const
{
    // 查找传感器对应的 TF 帧名称（缺省时按规则生成）
    const auto *entry = findSensor(id);
    if (!entry)
    {
        return sensor_frame_prefix_ + std::to_string(id);
    }
    return entry->frame_id;
}

geometry_msgs::Pose SensorArrayDescription::poseFromXyzRpy(const std::vector<double> &xyz,
                                                           const std::vector<double> &rpy)
{
    // 将 xyz 与 rpy 组合为 geometry_msgs::Pose
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
    // 生成阵列及所有传感器的 TF 列表
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
    // 实时发送动态 TF
    dynamic_broadcaster_.sendTransform(buildTransforms(stamp));
}

void SensorArrayTfPublisher::publishStatic()
{
    if (static_sent_)
    {
        return;
    }
    // 仅初次发送静态 TF
    static_broadcaster_.sendTransform(buildTransforms(ros::Time::now()));
    static_sent_ = true;
}

} // namespace mag_core_description
