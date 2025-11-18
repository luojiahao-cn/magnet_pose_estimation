#include <mag_core_description/sensor_array_config_loader.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdexcept>
#include <string>

namespace mag_core_description
{
namespace
{
namespace xml = mag_core_utils::xmlrpc;

geometry_msgs::Pose poseFromXyzRpy(const std::vector<double> &xyz,
                                   const std::vector<double> &rpy)
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

SensorArraySensorConfig parseSensorEntry(const XmlRpc::XmlRpcValue &node,
                                         const std::string &context,
                                         const std::string &sensor_frame_prefix)
{
    const auto &entry = xml::asStruct(node, context);
    SensorArraySensorConfig cfg;
    cfg.id = static_cast<int>(xml::readNumber(xml::requireMember(entry, "id", context),
                                              xml::makeContext(context, "id")));
    cfg.frame_id = sensor_frame_prefix + std::to_string(cfg.id);
    if (xml::hasMember(entry, "frame"))
    {
        cfg.frame_id = xml::requireStringField(entry, "frame", context);
    }

    const auto pose_ctx = xml::makeContext(context, "pose");
    const auto &pose_node = xml::requireStructField(entry, "pose", context);
    const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
    const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
    cfg.pose = poseFromXyzRpy(xyz, rpy);
    return cfg;
}

} // namespace

SensorArrayConfig loadSensorArrayConfig(const XmlRpc::XmlRpcValue &root,
                                        const std::string &context)
{
    const auto &node = xml::asStruct(root, context);

    SensorArrayConfig cfg;
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(node, "frames", context);
    cfg.parent_frame = xml::requireStringField(frames, "parent", frames_ctx);
    cfg.array_frame = xml::requireStringField(frames, "array", frames_ctx);
    cfg.sensor_frame_prefix = xml::optionalStringField(frames, "sensor_frame_prefix", frames_ctx, cfg.sensor_frame_prefix);

    const auto pose_ctx = xml::makeContext(frames_ctx, "pose");
    const auto &pose_node = xml::requireStructField(frames, "pose", frames_ctx);
    const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
    const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
    cfg.array_pose = poseFromXyzRpy(xyz, rpy);

    const auto sensors_ctx = xml::makeContext(context, "sensors");
    const auto &sensors = xml::requireArrayField(node, "sensors", context);
    cfg.sensors.reserve(sensors.size());
    for (int i = 0; i < sensors.size(); ++i)
    {
        const auto entry_ctx = xml::makeContext(sensors_ctx, "[" + std::to_string(i) + "]");
        cfg.sensors.push_back(parseSensorEntry(sensors[i], entry_ctx, cfg.sensor_frame_prefix));
    }

    if (cfg.sensors.empty())
    {
        throw std::runtime_error(context + ": sensors 列表不可为空");
    }

    return cfg;
}

} // namespace mag_core_description
