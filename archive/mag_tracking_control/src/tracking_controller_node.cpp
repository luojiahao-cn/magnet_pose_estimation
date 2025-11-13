#include <mag_tracking_control/tracking_controller.hpp>
#include <ros/ros.h>
#include <moveit_msgs/MoveGroupGoal.h>
#include <moveit_msgs/MoveGroupResult.h>
#include <moveit_msgs/PlanningOptions.h>

namespace mag_tracking_control {

TrackingController::TrackingController(ros::NodeHandle &nh)
    : nh_(nh),
      tracking_server_(nh_, "track_magnet", boost::bind(&TrackingController::executeTracking, this, _1), false),
      move_group_client_("move_group", true),
      tf_listener_(tf_buffer_),
      tracking_active_(false)
{
    // 参数加载
    nh_.param<std::string>("sensor_array_frame", sensor_array_frame_, "sensor_array_frame");
    nh_.param<std::string>("magnet_frame", magnet_frame_, "magnet_frame");
    nh_.param<std::string>("magnet_arm_planning_group", magnet_arm_planning_group_, "magnet_arm");
    nh_.param<std::string>("world_frame", world_frame_, "world");

    // 等待MoveIt action server
    ROS_INFO("[tracking_controller] 等待MoveIt action server...");
    move_group_client_.waitForServer();
    ROS_INFO("[tracking_controller] MoveIt action server已连接");

    tracking_server_.registerPreemptCallback(boost::bind(&TrackingController::preemptCallback, this));
    tracking_server_.start();
    ROS_INFO("[tracking_controller] 跟踪控制器已启动");
}

void TrackingController::run()
{
    ros::Rate rate(10); // 10Hz控制循环
    while (ros::ok()) {
        if (tracking_active_) {
            geometry_msgs::PoseStamped target_pose;
            if (computeTargetPose(target_pose)) {
                // 发送MoveIt目标
                moveit_msgs::MoveGroupGoal move_goal;
                move_goal.request.group_name = magnet_arm_planning_group_;
                move_goal.request.goal_constraints.resize(1);
                move_goal.request.goal_constraints[0].position_constraints.resize(1);
                move_goal.request.goal_constraints[0].orientation_constraints.resize(1);

                // 位置约束
                move_goal.request.goal_constraints[0].position_constraints[0].header = target_pose.header;
                move_goal.request.goal_constraints[0].position_constraints[0].link_name = "magnet_arm_ee";
                move_goal.request.goal_constraints[0].position_constraints[0].target_point_offset.x = 0;
                move_goal.request.goal_constraints[0].position_constraints[0].target_point_offset.y = 0;
                move_goal.request.goal_constraints[0].position_constraints[0].target_point_offset.z = 0;
                move_goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
                move_goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
                move_goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.resize(1);
                move_goal.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions[0] = 0.01; // 1cm tolerance
                move_goal.request.goal_constraints[0].position_constraints[0].weight = 1.0;

                // 姿态约束
                move_goal.request.goal_constraints[0].orientation_constraints[0].header = target_pose.header;
                move_goal.request.goal_constraints[0].orientation_constraints[0].link_name = "magnet_arm_ee";
                move_goal.request.goal_constraints[0].orientation_constraints[0].orientation = target_pose.pose.orientation;
                move_goal.request.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = 0.1;
                move_goal.request.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
                move_goal.request.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = 0.1;
                move_goal.request.goal_constraints[0].orientation_constraints[0].weight = 1.0;

                move_goal.request.num_planning_attempts = 5;
                move_goal.request.allowed_planning_time = 2.0;

                move_group_client_.sendGoal(move_goal);
            } else {
                ROS_WARN_THROTTLE(1.0, "[tracking_controller] 无法计算目标姿态");
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void TrackingController::executeTracking(const magnet_msgs::TrackMagnetGoalConstPtr &goal)
{
    ROS_INFO("[tracking_controller] 收到跟踪请求: enable=%d", goal->enable_tracking);

    if (goal->enable_tracking) {
        tracking_active_ = true;
        target_offset_ = goal->target_offset;

        magnet_msgs::TrackMagnetFeedback feedback;
        magnet_msgs::TrackMagnetResult result;

        ros::Rate rate(5);
        while (tracking_active_ && ros::ok()) {
            if (tracking_server_.isPreemptRequested()) {
                tracking_server_.setPreempted();
                break;
            }

            // 提供反馈
            geometry_msgs::PoseStamped current_magnet_pose;
            if (computeTargetPose(current_magnet_pose)) {
                feedback.current_magnet_pose = current_magnet_pose.pose;
                tracking_server_.publishFeedback(feedback);
            }

            rate.sleep();
        }

        if (tracking_active_) {
            result.success = true;
            result.message = "跟踪完成";
            tracking_server_.setSucceeded(result);
        }
    } else {
        tracking_active_ = false;
        magnet_msgs::TrackMagnetResult result;
        result.success = true;
        result.message = "跟踪已停止";
        tracking_server_.setSucceeded(result);
    }
}

bool TrackingController::computeTargetPose(geometry_msgs::PoseStamped &target_pose)
{
    try {
        // 获取磁铁在传感器阵列坐标系下的姿态
        geometry_msgs::TransformStamped magnet_transform = tf_buffer_.lookupTransform(
            sensor_array_frame_, magnet_frame_, ros::Time(0), ros::Duration(0.1));

        // 应用偏移
        geometry_msgs::PoseStamped magnet_pose;
        magnet_pose.header = magnet_transform.header;
        magnet_pose.pose.position.x = magnet_transform.transform.translation.x + target_offset_.position.x;
        magnet_pose.pose.position.y = magnet_transform.transform.translation.y + target_offset_.position.y;
        magnet_pose.pose.position.z = magnet_transform.transform.translation.z + target_offset_.position.z;
        magnet_pose.pose.orientation = magnet_transform.transform.rotation;

        // 转换到世界坐标系
        geometry_msgs::TransformStamped array_to_world = tf_buffer_.lookupTransform(
            world_frame_, sensor_array_frame_, ros::Time(0), ros::Duration(0.1));

        tf2::doTransform(magnet_pose, target_pose, array_to_world);
        return true;
    } catch (const tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[tracking_controller] TF转换失败: %s", ex.what());
        return false;
    }
}

void TrackingController::preemptCallback()
{
    ROS_INFO("[tracking_controller] 跟踪被抢占");
    tracking_active_ = false;
    move_group_client_.cancelAllGoals();
}

} // namespace mag_tracking_control