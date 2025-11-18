#include <mag_device_magnet/magnet_config_loader.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace mag_device_magnet
{
namespace
{
namespace xml = mag_core_utils::xmlrpc;

std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

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

geometry_msgs::Quaternion quaternionFromRpy(const std::vector<double> &rpy)
{
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    return tf2::toMsg(q);
}

void parseAxisFlags(const std::string &axes, bool &axis_x, bool &axis_y, bool &axis_z)
{
    axis_x = axis_y = axis_z = false;
    for (const auto c_raw : axes)
    {
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(c_raw)));
        if (c == 'x')
        {
            axis_x = true;
        }
        else if (c == 'y')
        {
            axis_y = true;
        }
        else if (c == 'z')
        {
            axis_z = true;
        }
    }
    if (!axis_x && !axis_y && !axis_z)
    {
        throw std::runtime_error("orientation.spin.axis 至少需要包含 x/y/z 之一");
    }
}

FrameConfig parseFrames(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(root, "frames", context);
    FrameConfig cfg;
    cfg.parent_frame = xml::requireStringField(frames, "parent", frames_ctx);
    cfg.child_frame = xml::requireStringField(frames, "child", frames_ctx);
    return cfg;
}

TopicConfig parseTopics(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    TopicConfig cfg;
    if (!xml::hasMember(root, "topics"))
    {
        return cfg;
    }
    const auto topics_ctx = xml::makeContext(context, "topics");
    const auto &topics = xml::requireStructField(root, "topics", context);
    cfg.pose_topic = xml::optionalStringField(topics, "pose", topics_ctx, cfg.pose_topic);
    return cfg;
}

MotionConfig parseMotion(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    MotionConfig cfg;
    if (xml::hasMember(root, "motion"))
    {
        const auto motion_ctx = xml::makeContext(context, "motion");
        const auto &motion = xml::requireStructField(root, "motion", context);
        cfg.update_rate_hz = xml::optionalNumberField(motion, "update_rate_hz", motion_ctx, cfg.update_rate_hz);
        cfg.magnetic_strength = xml::optionalNumberField(motion, "magnetic_strength", motion_ctx, cfg.magnetic_strength);
    }
    if (cfg.update_rate_hz <= 0.0)
    {
        throw std::runtime_error("motion.update_rate_hz 需为正数");
    }
    return cfg;
}

TfConfig parseTf(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    TfConfig cfg;
    if (!xml::hasMember(root, "tf"))
    {
        return cfg;
    }
    const auto tf_ctx = xml::makeContext(context, "tf");
    const auto &tf = xml::requireStructField(root, "tf", context);
    cfg.enable = xml::optionalBoolField(tf, "enable", tf_ctx, cfg.enable);
    cfg.use_static_tf = xml::optionalBoolField(tf, "static", tf_ctx, cfg.use_static_tf);
    return cfg;
}

CircularTrajectoryConfig parseCircularConfig(const XmlRpc::XmlRpcValue &trajectory,
                                             const std::string &context)
{
    const auto circular_ctx = xml::makeContext(context, "circular");
    const auto &circular = xml::requireStructField(trajectory, "circular", context);

    CircularTrajectoryConfig cfg;
    const auto center = xml::requireVector3Field(circular, "center_xyz", circular_ctx);
    cfg.center.x = center[0];
    cfg.center.y = center[1];
    cfg.center.z = center[2];
    cfg.radius = xml::optionalNumberField(circular, "radius", circular_ctx, cfg.radius);
    cfg.angular_speed = xml::optionalNumberField(circular, "angular_speed", circular_ctx, cfg.angular_speed);
    cfg.phase = xml::optionalNumberField(circular, "phase", circular_ctx, cfg.phase);
    if (cfg.radius <= 0.0)
    {
        throw std::runtime_error("trajectory.circular.radius 需为正数");
    }
    if (xml::hasMember(circular, "orientation_rpy"))
    {
        const auto rpy = xml::requireVector3Field(circular, "orientation_rpy", circular_ctx);
        cfg.orientation = quaternionFromRpy(rpy);
    }
    return cfg;
}

RectangularTrajectoryConfig parseRectangularConfig(const XmlRpc::XmlRpcValue &trajectory,
                                                   const std::string &context)
{
    const auto rect_ctx = xml::makeContext(context, "rectangular");
    const auto &rect = xml::requireStructField(trajectory, "rectangular", context);
    RectangularTrajectoryConfig cfg;
    cfg.width = xml::readNumber(xml::requireMember(rect, "width", rect_ctx), xml::makeContext(rect_ctx, "width"));
    cfg.height = xml::readNumber(xml::requireMember(rect, "height", rect_ctx), xml::makeContext(rect_ctx, "height"));
    cfg.velocity = xml::optionalNumberField(rect, "velocity", rect_ctx, cfg.velocity);
    cfg.z = xml::optionalNumberField(rect, "z", rect_ctx, cfg.z);
    if (xml::hasMember(rect, "center_xyz"))
    {
        const auto center = xml::requireVector3Field(rect, "center_xyz", rect_ctx);
        cfg.center.x = center[0];
        cfg.center.y = center[1];
        cfg.center.z = center[2];
    }
    if (xml::hasMember(rect, "orientation_rpy"))
    {
        const auto rpy = xml::requireVector3Field(rect, "orientation_rpy", rect_ctx);
        cfg.orientation = quaternionFromRpy(rpy);
    }
    if (cfg.width <= 0.0 || cfg.height <= 0.0)
    {
        throw std::runtime_error("trajectory.rectangular.width/height 需为正数");
    }
    if (cfg.velocity <= 0.0)
    {
        throw std::runtime_error("trajectory.rectangular.velocity 需为正数");
    }
    cfg.perimeter = 2.0 * (cfg.width + cfg.height);
    if (cfg.perimeter <= 0.0)
    {
        throw std::runtime_error("trajectory.rectangular 宽高配置异常");
    }
    return cfg;
}

TrajectoryConfig parseTrajectory(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto traj_ctx = xml::makeContext(context, "trajectory");
    const auto &trajectory = xml::requireStructField(root, "trajectory", context);
    const auto type_raw = xml::requireStringField(trajectory, "type", traj_ctx);
    const auto type = toLower(type_raw);

    TrajectoryConfig cfg;
    if (type == "static")
    {
        const auto static_ctx = xml::makeContext(traj_ctx, "static");
        const auto &static_block = xml::requireStructField(trajectory, "static", traj_ctx);
        const auto pose_ctx = xml::makeContext(static_ctx, "pose");
        const auto &pose_node = xml::requireStructField(static_block, "pose", static_ctx);
        const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
        const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
        cfg.type = TrajectoryType::Static;
        cfg.static_config.pose = poseFromXyzRpy(xyz, rpy);
    }
    else if (type == "circular")
    {
        cfg.type = TrajectoryType::Circular;
        cfg.circular_config = parseCircularConfig(trajectory, traj_ctx);
    }
    else if (type == "rectangular")
    {
        cfg.type = TrajectoryType::Rectangular;
        cfg.rectangular_config = parseRectangularConfig(trajectory, traj_ctx);
    }
    else
    {
        throw std::runtime_error("不支持的 trajectory.type: " + type_raw);
    }

    return cfg;
}

OrientationConfig parseOrientation(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    OrientationConfig cfg;
    cfg.fixed_orientation.w = 1.0;
    if (!xml::hasMember(root, "orientation"))
    {
        cfg.mode = OrientationMode::FromTrajectory;
        return cfg;
    }

    const auto orient_ctx = xml::makeContext(context, "orientation");
    const auto &orientation = xml::requireStructField(root, "orientation", context);
    const auto mode_raw = xml::optionalStringField(orientation, "mode", orient_ctx, std::string("trajectory"));
    const auto mode = toLower(mode_raw);

    if (mode == "trajectory")
    {
        cfg.mode = OrientationMode::FromTrajectory;
        return cfg;
    }

    if (mode == "fixed")
    {
        cfg.mode = OrientationMode::Fixed;
        const auto fixed_ctx = xml::makeContext(orient_ctx, "fixed");
        const auto &fixed = xml::requireStructField(orientation, "fixed", orient_ctx);
        const auto rpy = xml::requireVector3Field(fixed, "rpy", fixed_ctx);
        cfg.fixed_orientation = quaternionFromRpy(rpy);
        return cfg;
    }

    if (mode == "spin")
    {
        cfg.mode = OrientationMode::Spin;
        const auto spin_ctx = xml::makeContext(orient_ctx, "spin");
        const auto &spin = xml::requireStructField(orientation, "spin", orient_ctx);
        const auto initial = xml::requireVector3Field(spin, "initial_rpy", spin_ctx);
        cfg.initial_roll = initial[0];
        cfg.initial_pitch = initial[1];
        cfg.initial_yaw = initial[2];
        cfg.angular_velocity = xml::optionalNumberField(spin, "angular_velocity", spin_ctx, cfg.angular_velocity);
        const auto axis = xml::optionalStringField(spin, "axis", spin_ctx, std::string("z"));
        parseAxisFlags(axis, cfg.axis_x, cfg.axis_y, cfg.axis_z);
        return cfg;
    }

    throw std::runtime_error("未知的 orientation.mode: " + mode_raw);
}

} // namespace

MagnetConfigBundle loadMagnetConfig(const XmlRpc::XmlRpcValue &root,
                                    const std::string &context)
{
    const auto &node = xml::asStruct(root, context);
    MagnetConfigBundle bundle;
    bundle.frame = parseFrames(node, context);
    bundle.topics = parseTopics(node, context);
    bundle.motion = parseMotion(node, context);
    bundle.tf = parseTf(node, context);
    bundle.trajectory = parseTrajectory(node, context);
    bundle.orientation = parseOrientation(node, context);
    return bundle;
}

} // namespace mag_device_magnet
