#include <mag_device_arm/arm_node.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <shape_msgs/Mesh.h>

#include <Eigen/Core>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace mag_device_arm
{
namespace
{

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
