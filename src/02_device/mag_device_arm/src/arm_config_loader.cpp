#include <mag_device_arm/arm_node.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace mag_device_arm
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

std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

ToolMountOption parseToolOption(const XmlRpc::XmlRpcValue &tool_node,
                                const std::string &context,
                                const std::string &default_parent)
{
    ToolMountOption option;
    option.name = xml::requireStringField(tool_node, "name", context);
    option.child_frame = xml::optionalStringField(tool_node, "frame", context, option.name);
    option.parent_frame = xml::optionalStringField(tool_node, "parent_frame", context, default_parent);
    option.mesh_resource = xml::optionalStringField(tool_node, "mesh", context, std::string());
    option.pose.position.x = 0.0;
    option.pose.position.y = 0.0;
    option.pose.position.z = 0.0;
    option.pose.orientation.x = 0.0;
    option.pose.orientation.y = 0.0;
    option.pose.orientation.z = 0.0;
    option.pose.orientation.w = 1.0;

    if (xml::hasMember(tool_node, "pose"))
    {
        const auto &pose_node = xml::requireStructField(tool_node, "pose", context);
        const auto pose_ctx = xml::makeContext(context, "pose");
        const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
        const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
        option.pose = poseFromXyzRpy(xyz, rpy);
    }

    if (xml::hasMember(tool_node, "scale"))
    {
        const auto scale_vec = xml::requireVector3Field(tool_node, "scale", context);
        option.scale = { scale_vec[0], scale_vec[1], scale_vec[2] };
    }

    return option;
}

ArmConfig parseArmConfig(const XmlRpc::XmlRpcValue &arm_node,
                         const std::string &context)
{
    const auto &node = xml::asStruct(arm_node, context);
    ArmConfig cfg;
    cfg.name = xml::requireStringField(node, "name", context);
    cfg.group_name = xml::requireStringField(node, "group_name", context);
    cfg.end_effector_link = xml::requireStringField(node, "end_effector_link", context);
    cfg.reference_frame = xml::optionalStringField(node, "reference_frame", context, cfg.reference_frame);
    cfg.default_velocity = xml::optionalNumberField(node, "default_velocity", context, cfg.default_velocity);
    cfg.default_acceleration = xml::optionalNumberField(node, "default_acceleration", context, cfg.default_acceleration);
    cfg.planning_time = xml::optionalNumberField(node, "planning_time", context, cfg.planning_time);
    cfg.allow_replanning = xml::optionalBoolField(node, "allow_replanning", context, cfg.allow_replanning);
    cfg.position_tolerance = xml::optionalNumberField(node, "goal_position_tolerance", context, cfg.position_tolerance);
    cfg.orientation_tolerance = xml::optionalNumberField(node, "goal_orientation_tolerance", context, cfg.orientation_tolerance);
    cfg.default_named_target = xml::optionalStringField(node, "default_named_target", context, cfg.default_named_target);

    if (xml::hasMember(node, "base_tf"))
    {
        const auto &base_tf = xml::requireStructField(node, "base_tf", context);
        cfg.base_tf.enabled = true;
        const auto base_ctx = xml::makeContext(context, "base_tf");
        cfg.base_tf.parent_frame = xml::requireStringField(base_tf, "parent", base_ctx);
        cfg.base_tf.child_frame = xml::requireStringField(base_tf, "child", base_ctx);
        cfg.base_tf.pose.orientation.w = 1.0;
        if (xml::hasMember(base_tf, "pose"))
        {
            const auto &pose_node = xml::requireStructField(base_tf, "pose", base_ctx);
            const auto pose_ctx = xml::makeContext(base_ctx, "pose");
            const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
            const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
            cfg.base_tf.pose = poseFromXyzRpy(xyz, rpy);
        }
    }

    if (xml::hasMember(node, "named_targets"))
    {
        const auto &named_targets = xml::requireArrayField(node, "named_targets", context);
        for (int i = 0; i < named_targets.size(); ++i)
        {
            const auto idx_ctx = xml::makeContext(context, "named_targets[" + std::to_string(i) + "]");
            const auto &entry = xml::asStruct(named_targets[i], idx_ctx);
            const auto alias = xml::requireStringField(entry, "name", idx_ctx);
            const auto target = xml::requireStringField(entry, "target", idx_ctx);
            cfg.named_targets.emplace(alias, target);
        }
    }

    if (xml::hasMember(node, "tool_mount"))
    {
        const auto &tool_mount = xml::requireStructField(node, "tool_mount", context);
        if (!xml::hasMember(tool_mount, "options"))
        {
            throw std::runtime_error("tool_mount 需要提供 options 数组");
        }

        const auto tool_ctx = xml::makeContext(context, "tool_mount");
        const auto &options = xml::requireArrayField(tool_mount, "options", tool_ctx);
        if (options.size() == 0)
        {
            throw std::runtime_error("tool_mount.options 不可为空");
        }

        cfg.tool_options.reserve(options.size());
        std::unordered_set<std::string> option_names;
        for (int i = 0; i < options.size(); ++i)
        {
            const auto entry_ctx = xml::makeContext(tool_ctx, "options[" + std::to_string(i) + "]");
            const auto &entry = xml::asStruct(options[i], entry_ctx);
            ToolMountOption option = parseToolOption(entry, entry_ctx, cfg.end_effector_link);
            auto lower = toLower(option.name);
            if (!option_names.insert(lower).second)
            {
                throw std::runtime_error("重复的工具选项名称: " + option.name + " (arm=" + cfg.name + ")");
            }
            cfg.tool_options.push_back(option);
        }

        std::string selected = xml::optionalStringField(tool_mount, "selected", tool_ctx, std::string());
        if (selected.empty())
        {
            selected = cfg.tool_options.front().name;
        }

        for (const auto &option : cfg.tool_options)
        {
            if (option.name == selected)
            {
                cfg.selected_tool = option;
                break;
            }
        }

        if (!cfg.selected_tool.has_value())
        {
            throw std::runtime_error("tool_mount.selected 未匹配任何选项: " + selected + " (arm=" + cfg.name + ")");
        }
    }

    return cfg;
}

} // namespace

std::vector<ArmConfig> loadArmConfigs(const XmlRpc::XmlRpcValue &root,
                                      const std::string &context)
{
    const auto &cfg_root = xml::asStruct(root, context);
    std::vector<ArmConfig> result;
    if (!xml::hasMember(cfg_root, "arms"))
    {
        throw std::runtime_error("缺少 config.arms 配置");
    }
    const auto &arms_array = xml::requireArrayField(cfg_root, "arms", context);
    result.reserve(arms_array.size());
    std::unordered_set<std::string> names;
    for (int i = 0; i < arms_array.size(); ++i)
    {
        const auto arm_ctx = xml::makeContext(context, "arms[" + std::to_string(i) + "]");
        ArmConfig cfg = parseArmConfig(arms_array[i], arm_ctx);
        auto lower_name = toLower(cfg.name);
        if (!names.insert(lower_name).second)
        {
            throw std::runtime_error("重复的机械臂名称: " + cfg.name);
        }
        result.push_back(std::move(cfg));
    }
    if (result.empty())
    {
        throw std::runtime_error("config.arms 至少需要一个条目");
    }
    return result;
}

} // namespace mag_device_arm
