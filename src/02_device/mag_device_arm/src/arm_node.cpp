#include <mag_device_arm/arm_node.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <locale>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
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

    return cfg;
}

} // namespace

using moveit::planning_interface::MoveGroupInterface;

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
        result.push_back(cfg);
    }
    if (result.empty())
    {
        throw std::runtime_error("config.arms 至少需要一个条目");
    }
    return result;
}

ArmNode::ArmNode(ros::NodeHandle nh,
                 ros::NodeHandle pnh,
                 std::vector<ArmConfig> arm_configs)
    : nh_(nh), pnh_(pnh)
{
    // 初始化机械臂配置
    for (size_t i = 0; i < arm_configs.size(); ++i)
    {
        const std::string& key = arm_configs[i].name;
        if (arms_.find(key) != arms_.end())
        {
            throw std::runtime_error("重复的机械臂名称: " + key);
        }
        // 使用 piecewise_construct 直接构造，避免复制不可复制的成员
        arms_.emplace(std::piecewise_construct,
                      std::forward_as_tuple(key),
                      std::forward_as_tuple());
        arms_[key].config = std::move(arm_configs[i]);
    }
}

void ArmNode::start()
{
    // 初始化 MoveGroupInterface 并按需发布静态 TF
    std::vector<geometry_msgs::TransformStamped> static_transforms;

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
    }

    if (!static_transforms.empty())
    {
        static_broadcaster_.sendTransform(static_transforms);
    }

    set_pose_srv_ = pnh_.advertiseService("set_end_effector_pose", &ArmNode::handleSetPose, this);
    execute_named_srv_ = pnh_.advertiseService("execute_named_target", &ArmNode::handleExecuteNamedTarget, this);
    execute_cartesian_path_srv_ = pnh_.advertiseService("execute_cartesian_path", &ArmNode::handleExecuteCartesianPath, this);

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

bool ArmNode::handleExecuteCartesianPath(mag_device_arm::ExecuteCartesianPath::Request &req,
                                         mag_device_arm::ExecuteCartesianPath::Response &res)
{
    // 使用 MoveIt 的 computeCartesianPath 规划并可选执行连续轨迹
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        res.success = false;
        res.message = error;
        res.fraction = 0.0;
        return true;
    }

    if (req.waypoints.empty())
    {
        res.success = false;
        res.message = "路径点序列为空";
        res.fraction = 0.0;
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

    // 设置默认步长（如果未指定）
    const double step_size = req.step_size > 0.0 ? req.step_size : 0.01;

    // 计算笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group->computeCartesianPath(
        req.waypoints,
        step_size,
        trajectory
    );

    res.fraction = fraction;
    res.trajectory = trajectory;

    if (fraction < 0.5)
    {
        res.success = false;
        res.message = "笛卡尔路径规划失败，完成度: " + std::to_string(fraction * 100.0) + "%";
        return true;
    }

    if (req.execute)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto exec_result = group->execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            res.success = false;
            res.message = "轨迹执行失败，MoveIt 错误码=" + std::to_string(exec_result.val);
            return true;
        }
    }

    res.success = true;
    res.message = req.execute ? "笛卡尔路径规划并执行成功" : "笛卡尔路径规划成功";
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

} // namespace mag_device_arm

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_arm");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    namespace rps = rosparam_shortcuts;
    const std::string ns = "mag_device_arm";

    try
    {
        XmlRpc::XmlRpcValue config;
        std::size_t error = 0;
        error += !rps::get(ns, pnh, "config", config);
        rps::shutdownIfError(ns, error);

        std::vector<mag_device_arm::ArmConfig> arm_configs = 
            mag_device_arm::loadArmConfigs(config, ns + ".config");

        mag_device_arm::ArmNode node(nh, pnh, arm_configs);
        node.start();

        ros::waitForShutdown();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_arm 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}
