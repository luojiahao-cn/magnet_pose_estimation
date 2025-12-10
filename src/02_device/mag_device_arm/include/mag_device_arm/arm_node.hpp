#pragma once

#include <mag_device_arm/ExecuteNamedTarget.h>
#include <mag_device_arm/SetEndEffectorPose.h>
#include <mag_device_arm/ExecuteCartesianPath.h>

#include <XmlRpcValue.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace mag_device_arm
{

// 基座 TF 发布配置
struct BaseTransformConfig
{
    bool enabled = false;
    std::string parent_frame;
    std::string child_frame;
    geometry_msgs::Pose pose;
};

// 单个机械臂的 MoveIt 参数
struct ArmConfig
{
    std::string name;
    std::string group_name;
    std::string end_effector_link;
    std::string reference_frame = "world";
    double default_velocity = 0.2;
    double default_acceleration = 0.2;
    double planning_time = 5.0;
    bool allow_replanning = false;
    double position_tolerance = 0.005;
    double orientation_tolerance = 0.01;
    BaseTransformConfig base_tf;
    std::unordered_map<std::string, std::string> named_targets;
    std::string default_named_target;
    // 优化目标配置：是否偏好展开姿态（避免折叠）
    bool prefer_extended_config = false;
    // 关节权重：用于优化时优先选择展开的关节配置
    // 如果为空，则使用默认权重；否则按关节名称设置权重
    std::unordered_map<std::string, double> joint_weights;
};

struct ArmHandle
{
    ArmConfig config;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::mutex mutex;
};

std::vector<ArmConfig> loadArmConfigs(const XmlRpc::XmlRpcValue &root,
                                      const std::string &context);

class ArmNode
{
public:
    ArmNode(ros::NodeHandle nh,
            ros::NodeHandle pnh,
            std::vector<ArmConfig> arm_configs);

    void start();

private:
    void setLogLevel();
    bool handleSetPose(mag_device_arm::SetEndEffectorPose::Request &req,
                       mag_device_arm::SetEndEffectorPose::Response &res);

    bool handleExecuteNamedTarget(mag_device_arm::ExecuteNamedTarget::Request &req,
                                  mag_device_arm::ExecuteNamedTarget::Response &res);

    bool handleExecuteCartesianPath(mag_device_arm::ExecuteCartesianPath::Request &req,
                                    mag_device_arm::ExecuteCartesianPath::Response &res);

    moveit::planning_interface::MoveGroupInterface *getMoveGroup(const std::string &arm_name,
                                                                  std::string &error_message);

    // 配置 MoveGroup 的基础设置
    void configureMoveGroup(moveit::planning_interface::MoveGroupInterface *group, 
                           const ArmHandle &handle,
                           double velocity_scaling, double acceleration_scaling);

    // 执行规划并可选执行
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface *group, 
                       moveit::planning_interface::MoveGroupInterface::Plan &plan,
                       bool execute, std::string &error_msg);

    // 基于加权最短行程选择最优逆解
    // 返回 true 如果成功找到并设置了最优关节目标，false 否则
    bool selectOptimalIKSolution(moveit::planning_interface::MoveGroupInterface *group,
                                  const ArmConfig &config,
                                  const geometry_msgs::Pose &target_pose,
                                  const std::string &end_effector_link);

    // 计算关节权重（如果未配置则返回默认值）
    double getJointWeight(const ArmConfig &config, const std::string &joint_name, size_t joint_index);

    static geometry_msgs::TransformStamped makeTransform(const BaseTransformConfig &cfg,
                                                         const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unordered_map<std::string, ArmHandle> arms_;

    ros::ServiceServer set_pose_srv_;
    ros::ServiceServer execute_named_srv_;
    ros::ServiceServer execute_cartesian_path_srv_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    
    // 日志级别控制
    bool verbose_logging_;
};

} // namespace mag_device_arm
