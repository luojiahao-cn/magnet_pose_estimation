#pragma once

#include <mag_device_arm/ExecuteNamedTarget.h>
#include <mag_device_arm/SetEndEffectorPose.h>

#include <XmlRpcValue.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <array>
#include <memory>
#include <mutex>
#include <optional>
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

// 工具安装配置
struct ToolMountOption
{
    std::string name;
    std::string parent_frame;
    std::string child_frame;
    geometry_msgs::Pose pose;
    std::string mesh_resource;
    std::array<double, 3> scale{ {1.0, 1.0, 1.0} };
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
    std::vector<ToolMountOption> tool_options;
    std::optional<ToolMountOption> selected_tool;
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
    bool handleSetPose(mag_device_arm::SetEndEffectorPose::Request &req,
                       mag_device_arm::SetEndEffectorPose::Response &res);

    bool handleExecuteNamedTarget(mag_device_arm::ExecuteNamedTarget::Request &req,
                                  mag_device_arm::ExecuteNamedTarget::Response &res);

    moveit::planning_interface::MoveGroupInterface *getMoveGroup(const std::string &arm_name,
                                                                  std::string &error_message);

    static geometry_msgs::TransformStamped makeTransform(const BaseTransformConfig &cfg,
                                                         const ros::Time &stamp);
    static geometry_msgs::TransformStamped makeTransform(const ToolMountOption &cfg,
                                                         const ros::Time &stamp);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unordered_map<std::string, ArmHandle> arms_;

    ros::ServiceServer set_pose_srv_;
    ros::ServiceServer execute_named_srv_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

} // namespace mag_device_arm
