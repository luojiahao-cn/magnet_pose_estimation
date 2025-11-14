#include <mag_device_arm/arm_node.hpp>

#include <mag_core_utils/param_reader.hpp>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/Mesh.h>

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace mag_device_arm
{
namespace
{
geometry_msgs::Pose poseFromXyzRpy(const std::vector<double> &xyz, const std::vector<double> &rpy)
{
    // 将参数中的平移与欧拉角转换为 Pose 结构
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

ToolMountOption parseToolOption(const mag_core_utils::param::StructReader &tool_node,
                                const std::string &default_parent)
{
    ToolMountOption option;
    option.name = tool_node.requireString("name");
    option.child_frame = tool_node.optionalString("frame", option.name);
    option.parent_frame = tool_node.optionalString("parent_frame", default_parent);
    option.mesh_resource = tool_node.optionalString("mesh", std::string());
    option.pose.position.x = 0.0;
    option.pose.position.y = 0.0;
    option.pose.position.z = 0.0;
    option.pose.orientation.x = 0.0;
    option.pose.orientation.y = 0.0;
    option.pose.orientation.z = 0.0;
    option.pose.orientation.w = 1.0;

    if (tool_node.has("pose"))
    {
        const auto pose_node = tool_node.childStruct("pose");
        const auto xyz = pose_node.requireVector3("xyz");
        const auto rpy = pose_node.requireVector3("rpy");
        option.pose = poseFromXyzRpy(xyz, rpy);
    }

    if (tool_node.has("scale"))
    {
        const auto scale_vec = tool_node.requireVector3("scale");
        option.scale = {scale_vec[0], scale_vec[1], scale_vec[2]};
    }

    return option;
}

shape_msgs::Mesh meshToMsg(const shapes::Mesh &mesh)
{
    shape_msgs::Mesh msg;
    msg.vertices.resize(mesh.vertex_count);
    for (size_t i = 0; i < mesh.vertex_count; ++i)
    {
        geometry_msgs::Point p;
        p.x = mesh.vertices[3 * i + 0];
        p.y = mesh.vertices[3 * i + 1];
        p.z = mesh.vertices[3 * i + 2];
        msg.vertices[i] = p;
    }

    msg.triangles.resize(mesh.triangle_count);
    for (size_t i = 0; i < mesh.triangle_count; ++i)
    {
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = mesh.triangles[3 * i + 0];
        tri.vertex_indices[1] = mesh.triangles[3 * i + 1];
        tri.vertex_indices[2] = mesh.triangles[3 * i + 2];
        msg.triangles[i] = tri;
    }

    return msg;
}

std::optional<moveit_msgs::AttachedCollisionObject> makeAttachedCollisionObject(const ArmConfig &cfg,
                                                                                const ToolMountOption &tool)
{
    if (tool.mesh_resource.empty())
    {
        return std::nullopt;
    }

    Eigen::Vector3d scale(tool.scale[0], tool.scale[1], tool.scale[2]);
    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(tool.mesh_resource, scale));
    if (!mesh)
    {
        ROS_WARN_STREAM("[mag_device_arm] 无法加载工具 mesh: " << tool.mesh_resource);
        return std::nullopt;
    }

    moveit_msgs::AttachedCollisionObject attached;
    attached.link_name = cfg.end_effector_link;
    attached.object.header.frame_id = tool.parent_frame.empty() ? cfg.end_effector_link : tool.parent_frame;
    attached.object.id = cfg.name + "_" + tool.name + "_tool";
    attached.object.operation = moveit_msgs::CollisionObject::ADD;
    attached.object.meshes.push_back(meshToMsg(*mesh));
    attached.object.mesh_poses.push_back(tool.pose);
    attached.touch_links.push_back(cfg.end_effector_link);
    return attached;
}

} // namespace

ArmConfig parseArmConfig(const mag_core_utils::param::StructReader &arm_node)
{
    // 解析单个机械臂的配置项，缺失关键字段时抛出异常
    ArmConfig cfg;
    cfg.name = arm_node.requireString("name");
    cfg.group_name = arm_node.requireString("group_name");
    cfg.end_effector_link = arm_node.requireString("end_effector_link");
    cfg.reference_frame = arm_node.optionalString("reference_frame", cfg.reference_frame);
    cfg.default_velocity = arm_node.optionalNumber("default_velocity", cfg.default_velocity);
    cfg.default_acceleration = arm_node.optionalNumber("default_acceleration", cfg.default_acceleration);
    cfg.planning_time = arm_node.optionalNumber("planning_time", cfg.planning_time);
    cfg.allow_replanning = arm_node.optionalBool("allow_replanning", cfg.allow_replanning);
    cfg.position_tolerance = arm_node.optionalNumber("goal_position_tolerance", cfg.position_tolerance);
    cfg.orientation_tolerance = arm_node.optionalNumber("goal_orientation_tolerance", cfg.orientation_tolerance);
    cfg.default_named_target = arm_node.optionalString("default_named_target", cfg.default_named_target);

    if (arm_node.has("base_tf"))
    {
        const auto base_tf = arm_node.childStruct("base_tf");
        cfg.base_tf.enabled = true;
        cfg.base_tf.parent_frame = base_tf.requireString("parent");
        cfg.base_tf.child_frame = base_tf.requireString("child");
        if (base_tf.has("pose"))
        {
            const auto pose_node = base_tf.childStruct("pose");
            const auto xyz = pose_node.requireVector3("xyz");
            const auto rpy = pose_node.requireVector3("rpy");
            cfg.base_tf.pose = poseFromXyzRpy(xyz, rpy);
        }
        else
        {
            cfg.base_tf.pose.orientation.w = 1.0;
        }
    }

    if (arm_node.has("named_targets"))
    {
        const auto named_targets = arm_node.childArray("named_targets");
        for (int i = 0; i < named_targets.size(); ++i)
        {
            const auto entry = named_targets.structAt(i);
            const auto alias = entry.requireString("name");
            const auto target = entry.requireString("target");
            cfg.named_targets.emplace(alias, target);
        }
    }

    if (arm_node.has("tool_mount"))
    {
        const auto tool_mount = arm_node.childStruct("tool_mount");
        if (!tool_mount.has("options"))
        {
            throw std::runtime_error("tool_mount 需要提供 options 数组");
        }

        const auto options = tool_mount.childArray("options");
        if (options.size() == 0)
        {
            throw std::runtime_error("tool_mount.options 不可为空");
        }

        cfg.tool_options.reserve(options.size());
        std::unordered_set<std::string> option_names;
        for (int i = 0; i < options.size(); ++i)
        {
            const auto entry = options.structAt(i);
            ToolMountOption option = parseToolOption(entry, cfg.end_effector_link);
            auto lower = toLower(option.name);
            if (!option_names.insert(lower).second)
            {
                throw std::runtime_error("重复的工具选项名称: " + option.name + " (arm=" + cfg.name + ")");
            }
            cfg.tool_options.push_back(option);
        }

        std::string selected = tool_mount.optionalString("selected", std::string());
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

std::vector<ArmConfig> loadArmConfigs(const mag_core_utils::param::StructReader &root)
{
    // 从根节点读取全局配置，保证至少存在一个 arm 定义
    std::vector<ArmConfig> result;
    if (!root.has("arms"))
    {
        throw std::runtime_error("缺少 config.arms 配置");
    }
    const auto arms_array = root.childArray("arms");
    result.reserve(arms_array.size());
    std::unordered_set<std::string> names;
    for (int i = 0; i < arms_array.size(); ++i)
    {
        const auto arm_node = arms_array.structAt(i);
        ArmConfig cfg = parseArmConfig(arm_node);
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

using moveit::planning_interface::MoveGroupInterface;

ArmNode::ArmNode(ros::NodeHandle nh,
                 ros::NodeHandle pnh,
                 std::vector<ArmConfig> arm_configs)
    : nh_(std::move(nh)), pnh_(std::move(pnh))
{
    // 预先构造哈希表槽位，确保互斥量等成员不会被复制或移动
    for (auto &cfg : arm_configs)
    {
        auto key = cfg.name;
        auto inserted = arms_.emplace(std::piecewise_construct,
                                      std::forward_as_tuple(std::move(key)),
                                      std::forward_as_tuple());
        if (!inserted.second)
        {
            throw std::runtime_error("重复的机械臂名称: " + cfg.name);
        }
        ArmHandle &handle = inserted.first->second;
        handle.config = std::move(cfg);
    }
}

void ArmNode::start()
{
    // 初始化 MoveGroupInterface 并按需发布静态 TF
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    std::vector<moveit_msgs::AttachedCollisionObject> attached_tools;

    for (auto &kv : arms_)
    {
        ArmHandle &handle = kv.second;
        try
        {
            handle.move_group = std::make_unique<MoveGroupInterface>(handle.config.group_name);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("MoveGroupInterface 初始化失败 (" + handle.config.group_name + "): " + e.what());
        }

        auto &group = *handle.move_group;
        group.setPoseReferenceFrame(handle.config.reference_frame);
        group.setEndEffectorLink(handle.config.end_effector_link);
        group.setPlanningTime(handle.config.planning_time);
        group.allowReplanning(handle.config.allow_replanning);
        group.setGoalPositionTolerance(handle.config.position_tolerance);
        group.setGoalOrientationTolerance(handle.config.orientation_tolerance);
        group.setMaxVelocityScalingFactor(handle.config.default_velocity);
        group.setMaxAccelerationScalingFactor(handle.config.default_acceleration);

        const auto stamp = ros::Time::now();
        if (handle.config.base_tf.enabled)
        {
            static_transforms.push_back(makeTransform(handle.config.base_tf, stamp));
        }
        if (handle.config.selected_tool)
        {
            static_transforms.push_back(makeTransform(handle.config.selected_tool.value(), stamp));
            ROS_INFO_STREAM("[mag_device_arm] " << handle.config.name << " 工具挂载: "
                                                 << handle.config.selected_tool->name << " -> "
                                                 << handle.config.selected_tool->parent_frame << " -> "
                                                 << handle.config.selected_tool->child_frame);

            if (auto attached = makeAttachedCollisionObject(handle.config, handle.config.selected_tool.value()))
            {
                attached_tools.push_back(*attached);
            }
        }
    }

    if (!static_transforms.empty())
    {
        static_broadcaster_.sendTransform(static_transforms);
    }

    for (const auto &obj : attached_tools)
    {
        planning_scene_interface_.applyAttachedCollisionObject(obj);
    }

    set_pose_srv_ = pnh_.advertiseService("set_end_effector_pose", &ArmNode::handleSetPose, this);
    execute_named_srv_ = pnh_.advertiseService("execute_named_target", &ArmNode::handleExecuteNamedTarget, this);

    ROS_INFO_STREAM("[mag_device_arm] 已初始化 " << arms_.size() << " 个机械臂接口");
}

bool ArmNode::handleSetPose(mag_device_arm::SetEndEffectorPose::Request &req,
                            mag_device_arm::SetEndEffectorPose::Response &res)
{
    // 使用 MoveIt 规划并可选执行末端位姿的直达运动
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        res.success = false;
        res.message = error;
        return true;
    }

    ArmHandle &handle = arms_.at(req.arm);
    std::lock_guard<std::mutex> lock(handle.mutex);

    group->setStartStateToCurrentState();
    group->setPoseReferenceFrame(handle.config.reference_frame);
    group->setEndEffectorLink(handle.config.end_effector_link);

    const double vel = req.velocity_scaling > 0.0 ? req.velocity_scaling : handle.config.default_velocity;
    const double acc = req.acceleration_scaling > 0.0 ? req.acceleration_scaling : handle.config.default_acceleration;
    group->setMaxVelocityScalingFactor(vel);
    group->setMaxAccelerationScalingFactor(acc);

    group->setPoseTarget(req.target, handle.config.end_effector_link);

    MoveGroupInterface::Plan plan;
    auto plan_result = group->plan(plan);
    if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        res.success = false;
        res.message = "规划失败，MoveIt 错误码=" + std::to_string(plan_result.val);
        group->clearPoseTargets();
        return true;
    }

    if (req.execute)
    {
        auto exec_result = group->execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            res.success = false;
            res.message = "执行失败，MoveIt 错误码=" + std::to_string(exec_result.val);
            group->clearPoseTargets();
            return true;
        }
    }

    res.success = true;
    res.message = req.execute ? "规划并执行成功" : "规划成功";
    group->clearPoseTargets();
    return true;
}

bool ArmNode::handleExecuteNamedTarget(mag_device_arm::ExecuteNamedTarget::Request &req,
                                       mag_device_arm::ExecuteNamedTarget::Response &res)
{
    // 将自定义别名映射到 MoveIt 的 named target 并执行
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        res.success = false;
        res.message = error;
        return true;
    }

    ArmHandle &handle = arms_.at(req.arm);
    auto it = handle.config.named_targets.find(req.target);
    if (it == handle.config.named_targets.end())
    {
        res.success = false;
        res.message = "未知的 named target: " + req.target;
        return true;
    }

    std::lock_guard<std::mutex> lock(handle.mutex);

    group->setStartStateToCurrentState();
    group->setPoseReferenceFrame(handle.config.reference_frame);
    group->setEndEffectorLink(handle.config.end_effector_link);

    const double vel = req.velocity_scaling > 0.0 ? req.velocity_scaling : handle.config.default_velocity;
    const double acc = req.acceleration_scaling > 0.0 ? req.acceleration_scaling : handle.config.default_acceleration;
    group->setMaxVelocityScalingFactor(vel);
    group->setMaxAccelerationScalingFactor(acc);

    const std::string &target = it->second;
    if (!group->setNamedTarget(target))
    {
        res.success = false;
        res.message = "MoveIt 无法识别 named target: " + target;
        return true;
    }

    MoveGroupInterface::Plan plan;
    auto plan_result = group->plan(plan);
    if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        res.success = false;
        res.message = "规划失败，MoveIt 错误码=" + std::to_string(plan_result.val);
        group->clearPoseTargets();
        return true;
    }

    if (req.execute)
    {
        auto exec_result = group->execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            res.success = false;
            res.message = "执行失败，MoveIt 错误码=" + std::to_string(exec_result.val);
            group->clearPoseTargets();
            return true;
        }
    }

    res.success = true;
    res.message = req.execute ? "规划并执行成功" : "规划成功";
    group->clearPoseTargets();
    return true;
}

moveit::planning_interface::MoveGroupInterface *ArmNode::getMoveGroup(const std::string &arm_name,
                                                                       std::string &error_message)
{
    // 根据名称检索 MoveGroupInterface，未初始化时返回错误
    auto it = arms_.find(arm_name);
    if (it == arms_.end())
    {
        error_message = "未知的机械臂: " + arm_name;
        return nullptr;
    }
    if (!it->second.move_group)
    {
        error_message = "机械臂未初始化: " + arm_name;
        return nullptr;
    }
    return it->second.move_group.get();
}

geometry_msgs::TransformStamped ArmNode::makeTransform(const BaseTransformConfig &cfg,
                                                        const ros::Time &stamp)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = cfg.parent_frame;
    tf.child_frame_id = cfg.child_frame;
    tf.transform.translation.x = cfg.pose.position.x;
    tf.transform.translation.y = cfg.pose.position.y;
    tf.transform.translation.z = cfg.pose.position.z;
    tf.transform.rotation = cfg.pose.orientation;
    return tf;
}

geometry_msgs::TransformStamped ArmNode::makeTransform(const ToolMountOption &cfg,
                                                       const ros::Time &stamp)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = cfg.parent_frame;
    tf.child_frame_id = cfg.child_frame;
    tf.transform.translation.x = cfg.pose.position.x;
    tf.transform.translation.y = cfg.pose.position.y;
    tf.transform.translation.z = cfg.pose.position.z;
    tf.transform.rotation = cfg.pose.orientation;
    return tf;
}

} // namespace mag_device_arm
