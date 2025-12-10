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
#include <cmath>
#include <locale>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <ros/console.h>

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
    cfg.prefer_extended_config = xml::optionalBoolField(node, "prefer_extended_config", context, cfg.prefer_extended_config);
    
    // 加载关节权重配置（可选）
    if (xml::hasMember(node, "joint_weights"))
    {
        const auto &weights = xml::requireStructField(node, "joint_weights", context);
        for (const auto &entry : weights)
        {
            const std::string joint_name = entry.first;
            const auto joint_ctx = xml::makeContext(context, "joint_weights." + joint_name);
            cfg.joint_weights[joint_name] = xml::readNumber(entry.second, joint_ctx);
        }
    }

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
        for (size_t i = 0; i < static_cast<size_t>(named_targets.size()); ++i)
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
    : nh_(nh), pnh_(pnh), verbose_logging_(false)
{
    // 设置日志级别
    setLogLevel();
    // 读取日志级别参数
    pnh_.param("verbose_logging", verbose_logging_, false);
    
    for (auto &config : arm_configs)
    {
        const std::string &key = config.name;
        if (arms_.find(key) != arms_.end())
        {
            throw std::runtime_error("重复的机械臂名称: " + key);
        }
        arms_.emplace(std::piecewise_construct,
                      std::forward_as_tuple(key),
                      std::forward_as_tuple());
        arms_[key].config = std::move(config);
    }
}

void ArmNode::setLogLevel()
{
    std::string log_level_str = "INFO";
    pnh_.param("logging_level", log_level_str, log_level_str);
    
    // 转换为大写
    std::transform(log_level_str.begin(), log_level_str.end(), log_level_str.begin(), ::toupper);
    
    ros::console::Level level = ros::console::levels::Info;
    if (log_level_str == "DEBUG") {
        level = ros::console::levels::Debug;
    } else if (log_level_str == "INFO") {
        level = ros::console::levels::Info;
    } else if (log_level_str == "WARN") {
        level = ros::console::levels::Warn;
    } else if (log_level_str == "ERROR") {
        level = ros::console::levels::Error;
    } else if (log_level_str == "FATAL") {
        level = ros::console::levels::Fatal;
    }
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level);
}

void ArmNode::start()
{
    // 初始化 MoveGroupInterface 并按需发布静态 TF
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    const auto stamp = ros::Time::now();

    for (auto &kv : arms_)
    {
        ArmHandle &handle = kv.second;
        try
        {
            handle.move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(handle.config.group_name);
            if (verbose_logging_)
            {
                ROS_DEBUG_STREAM("[mag_device_arm] 初始化机械臂: " << handle.config.name 
                                 << " (group: " << handle.config.group_name << ")");
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[mag_device_arm] MoveGroupInterface 初始化失败 (%s): %s", 
                     handle.config.group_name.c_str(), e.what());
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

        if (handle.config.base_tf.enabled)
        {
            static_transforms.push_back(makeTransform(handle.config.base_tf, stamp));
            if (verbose_logging_)
            {
                ROS_DEBUG_STREAM("[mag_device_arm] 配置静态 TF: " << handle.config.base_tf.parent_frame 
                                 << " -> " << handle.config.base_tf.child_frame);
            }
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
    if (verbose_logging_)
    {
        ROS_INFO_STREAM("[mag_device_arm] 详细日志已启用");
    }
}

bool ArmNode::handleSetPose(mag_device_arm::SetEndEffectorPose::Request &req,
                            mag_device_arm::SetEndEffectorPose::Response &res)
{
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        ROS_ERROR("[mag_device_arm] set_end_effector_pose 失败: %s", error.c_str());
        res.success = false;
        res.message = error;
        return true;
    }

    ArmHandle &handle = arms_.at(req.arm);
    std::lock_guard<std::mutex> lock(handle.mutex);

    configureMoveGroup(group, handle, req.velocity_scaling, req.acceleration_scaling);

    // 尝试使用最优逆解选择（如果配置了优化选项）
    bool use_optimal_ik = !handle.config.joint_weights.empty() || handle.config.prefer_extended_config;
    if (use_optimal_ik)
    {
        if (selectOptimalIKSolution(group, handle.config, req.target, handle.config.end_effector_link))
        {
            if (verbose_logging_)
            {
                ROS_DEBUG_STREAM("[mag_device_arm] 使用加权最短行程准则选择了最优逆解");
            }
        }
        else
        {
            ROS_WARN_STREAM("[mag_device_arm] 最优逆解选择失败，回退到位姿目标");
            group->setPoseTarget(req.target, handle.config.end_effector_link);
        }
    }
    else
    {
        group->setPoseTarget(req.target, handle.config.end_effector_link);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!planAndExecute(group, plan, req.execute, res.message))
    {
        ROS_ERROR("[mag_device_arm] set_end_effector_pose 失败 (%s): %s", req.arm.c_str(), res.message.c_str());
        res.success = false;
        return true;
    }

    res.success = true;
    res.message = req.execute ? "规划并执行成功" : "规划成功";
    if (verbose_logging_)
    {
        ROS_INFO_STREAM("[mag_device_arm] set_end_effector_pose 成功 (" << req.arm << "): " << res.message);
    }
    return true;
}

bool ArmNode::handleExecuteNamedTarget(mag_device_arm::ExecuteNamedTarget::Request &req,
                                       mag_device_arm::ExecuteNamedTarget::Response &res)
{
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        ROS_ERROR("[mag_device_arm] execute_named_target 失败: %s", error.c_str());
        res.success = false;
        res.message = error;
        return true;
    }

    ArmHandle &handle = arms_.at(req.arm);
    auto it = handle.config.named_targets.find(req.target);
    if (it == handle.config.named_targets.end())
    {
        ROS_WARN_STREAM("[mag_device_arm] 未知的 named target: " << req.target << " (arm: " << req.arm << ")");
        res.success = false;
        res.message = "未知的 named target: " + req.target;
        return true;
    }

    std::lock_guard<std::mutex> lock(handle.mutex);
    configureMoveGroup(group, handle, req.velocity_scaling, req.acceleration_scaling);

    const std::string &target = it->second;
    if (!group->setNamedTarget(target))
    {
        ROS_ERROR("[mag_device_arm] MoveIt 无法识别 named target: %s (arm: %s)", target.c_str(), req.arm.c_str());
        res.success = false;
        res.message = "MoveIt 无法识别 named target: " + target;
        return true;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!planAndExecute(group, plan, req.execute, res.message))
    {
        ROS_ERROR("[mag_device_arm] execute_named_target 失败 (%s, target: %s): %s", 
                 req.arm.c_str(), req.target.c_str(), res.message.c_str());
        res.success = false;
        return true;
    }

    res.success = true;
    res.message = req.execute ? "规划并执行成功" : "规划成功";
    if (verbose_logging_)
    {
        ROS_INFO_STREAM("[mag_device_arm] execute_named_target 成功 (" << req.arm << ", target: " << req.target << ")");
    }
    return true;
}

bool ArmNode::handleExecuteCartesianPath(mag_device_arm::ExecuteCartesianPath::Request &req,
                                         mag_device_arm::ExecuteCartesianPath::Response &res)
{
    std::string error;
    auto *group = getMoveGroup(req.arm, error);
    if (!group)
    {
        ROS_ERROR("[mag_device_arm] execute_cartesian_path 失败: %s", error.c_str());
        res.success = false;
        res.message = error;
        res.fraction = 0.0;
        return true;
    }

    if (req.waypoints.empty())
    {
        ROS_WARN_STREAM("[mag_device_arm] 路径点序列为空 (arm: " << req.arm << ")");
        res.success = false;
        res.message = "路径点序列为空";
        res.fraction = 0.0;
        return true;
    }

    ArmHandle &handle = arms_.at(req.arm);
    std::lock_guard<std::mutex> lock(handle.mutex);
    configureMoveGroup(group, handle, req.velocity_scaling, req.acceleration_scaling);

    const double step_size = req.step_size > 0.0 ? req.step_size : 0.01;

    // 检查目标位姿是否可达（用于诊断）
    moveit::core::RobotStatePtr current_state = group->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group = 
        current_state->getJointModelGroup(handle.config.group_name);
    
    bool target_reachable = false;
    if (joint_model_group)
    {
        moveit::core::RobotState test_state(*current_state);
        target_reachable = test_state.setFromIK(joint_model_group, req.waypoints.back(), 
                                                 handle.config.end_effector_link, 0.1);
        if (!target_reachable && verbose_logging_)
        {
            ROS_DEBUG_STREAM("[mag_device_arm] 目标位姿不可达（无法求解逆运动学）");
        }
    }

    // 计算笛卡尔路径
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group->computeCartesianPath(req.waypoints, step_size, trajectory);

    res.fraction = fraction;
    res.trajectory = trajectory;

    if (fraction < 0.5)
    {
        res.success = false;
        std::string error_msg = "笛卡尔路径规划失败，完成度: " + std::to_string(fraction * 100.0) + "%";
        
        if (fraction == 0.0)
        {
            error_msg += target_reachable 
                ? " (路径中间点逆解求解失败，可能原因：碰撞、关节限制或步长过大)"
                : " (目标位姿不可达)";
        }
        else
        {
            error_msg += " (部分路径可达，但未达到目标)";
        }
        
        ROS_ERROR("[mag_device_arm] execute_cartesian_path 失败 (%s): %s", req.arm.c_str(), error_msg.c_str());
        res.message = error_msg;
        return true;
    }

    if (req.execute)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        auto exec_result = group->execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("[mag_device_arm] 轨迹执行失败 (%s)，MoveIt 错误码=%d", req.arm.c_str(), exec_result.val);
            res.success = false;
            res.message = "轨迹执行失败，MoveIt 错误码=" + std::to_string(exec_result.val);
            return true;
        }
    }

    res.success = true;
    res.message = req.execute ? "笛卡尔路径规划并执行成功" : "笛卡尔路径规划成功";
    if (verbose_logging_)
    {
        ROS_INFO_STREAM("[mag_device_arm] execute_cartesian_path 成功 (" << req.arm 
                        << ", 路径点: " << req.waypoints.size() << ", 完成度: " << fraction * 100.0 << "%)");
    }
    return true;
}

void ArmNode::configureMoveGroup(moveit::planning_interface::MoveGroupInterface *group, 
                                 const ArmHandle &handle,
                                 double velocity_scaling, double acceleration_scaling)
{
    group->setStartStateToCurrentState();
    group->setPoseReferenceFrame(handle.config.reference_frame);
    group->setEndEffectorLink(handle.config.end_effector_link);
    
    const double vel = velocity_scaling > 0.0 ? velocity_scaling : handle.config.default_velocity;
    const double acc = acceleration_scaling > 0.0 ? acceleration_scaling : handle.config.default_acceleration;
    group->setMaxVelocityScalingFactor(vel);
    group->setMaxAccelerationScalingFactor(acc);
}

bool ArmNode::planAndExecute(moveit::planning_interface::MoveGroupInterface *group, 
                             moveit::planning_interface::MoveGroupInterface::Plan &plan,
                             bool execute, std::string &error_msg)
{
    auto plan_result = group->plan(plan);
    if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        error_msg = "规划失败，MoveIt 错误码=" + std::to_string(plan_result.val);
        if (verbose_logging_)
        {
            ROS_DEBUG_STREAM("[mag_device_arm] 规划失败: " << error_msg);
        }
        group->clearPoseTargets();
        return false;
    }

    if (execute)
    {
        auto exec_result = group->execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            error_msg = "执行失败，MoveIt 错误码=" + std::to_string(exec_result.val);
            if (verbose_logging_)
            {
                ROS_DEBUG_STREAM("[mag_device_arm] 执行失败: " << error_msg);
            }
            group->clearPoseTargets();
            return false;
        }
    }

    group->clearPoseTargets();
    return true;
}

moveit::planning_interface::MoveGroupInterface *ArmNode::getMoveGroup(const std::string &arm_name,
                                                                       std::string &error_message)
{
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

double ArmNode::getJointWeight(const ArmConfig &config, const std::string &joint_name, size_t joint_index)
{
    // 如果配置了该关节的权重，使用配置值
    auto it = config.joint_weights.find(joint_name);
    if (it != config.joint_weights.end())
    {
        return it->second;
    }
    
    // 默认策略：前3个大关节权重高（少移动），后续关节权重低（多移动）
    return (joint_index < 3) ? 3.0 : 0.5;
}

bool ArmNode::selectOptimalIKSolution(moveit::planning_interface::MoveGroupInterface *group,
                                       const ArmConfig &config,
                                       const geometry_msgs::Pose &target_pose,
                                       const std::string &end_effector_link)
{
    moveit::core::RobotStatePtr current_state = group->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group = 
        current_state->getJointModelGroup(config.group_name);
    
    if (!joint_model_group)
    {
        if (verbose_logging_)
        {
            ROS_WARN_STREAM("[mag_device_arm] 无法获取关节模型组: " << config.group_name);
        }
        return false;
    }

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    std::vector<double> current_joint_values;
    current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

    // 收集多个逆解候选
    std::vector<moveit::core::RobotStatePtr> candidate_states;
    const int max_attempts = 10;
    const double duplicate_threshold = 0.01;
    
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        moveit::core::RobotStatePtr seed_state(new moveit::core::RobotState(*current_state));
        if (attempt > 0)
        {
            seed_state->setToRandomPositions(joint_model_group);
        }
        
        if (!seed_state->setFromIK(joint_model_group, target_pose, end_effector_link, 0.1))
        {
            continue;
        }
        
        // 检查是否重复
        std::vector<double> candidate_values;
        seed_state->copyJointGroupPositions(joint_model_group, candidate_values);
        
        bool is_duplicate = false;
        for (const auto &existing_state : candidate_states)
        {
            std::vector<double> existing_values;
            existing_state->copyJointGroupPositions(joint_model_group, existing_values);
            
            double diff_sq_sum = 0.0;
            for (size_t i = 0; i < candidate_values.size(); ++i)
            {
                double diff = candidate_values[i] - existing_values[i];
                diff_sq_sum += diff * diff;
            }
            
            if (std::sqrt(diff_sq_sum) < duplicate_threshold)
            {
                is_duplicate = true;
                break;
            }
        }
        
        if (!is_duplicate)
        {
            candidate_states.push_back(seed_state);
        }
    }

    if (candidate_states.empty())
    {
        if (verbose_logging_)
        {
            ROS_DEBUG_STREAM("[mag_device_arm] 无法找到有效的逆运动学解");
        }
        return false;
    }

    // 计算每个候选解的加权成本
    std::vector<double> candidate_costs;
    for (const auto &candidate_state : candidate_states)
    {
        std::vector<double> candidate_values;
        candidate_state->copyJointGroupPositions(joint_model_group, candidate_values);
        
        double weighted_cost = 0.0;
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            double joint_diff = candidate_values[i] - current_joint_values[i];
            
            // 处理角度环绕
            const moveit::core::JointModel *joint_model = 
                current_state->getJointModel(joint_names[i]);
            if (joint_model && joint_model->getType() == moveit::core::JointModel::REVOLUTE)
            {
                while (joint_diff > M_PI) joint_diff -= 2.0 * M_PI;
                while (joint_diff < -M_PI) joint_diff += 2.0 * M_PI;
            }
            
            double weight = getJointWeight(config, joint_names[i], i);
            weighted_cost += weight * joint_diff * joint_diff;
        }
        
        candidate_costs.push_back(weighted_cost);
    }

    // 选择成本最小的解
    size_t best_index = 0;
    for (size_t i = 1; i < candidate_costs.size(); ++i)
    {
        if (candidate_costs[i] < candidate_costs[best_index])
        {
            best_index = i;
        }
    }

    std::vector<double> best_joint_values;
    candidate_states[best_index]->copyJointGroupPositions(joint_model_group, best_joint_values);
    group->setJointValueTarget(best_joint_values);

    if (verbose_logging_)
    {
        ROS_DEBUG_STREAM("[mag_device_arm] 从 " << candidate_states.size() 
                         << " 个候选解中选择最优解（加权成本: " << candidate_costs[best_index] << "）");
    }

    return true;
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
