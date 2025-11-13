#ifndef MAG_TRACKING_CONTROL_TRACKING_CONTROLLER_HPP
#define MAG_TRACKING_CONTROL_TRACKING_CONTROLLER_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <magnet_msgs/TrackMagnetAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

namespace mag_tracking_control {

class TrackingController {
public:
    explicit TrackingController(ros::NodeHandle &nh);
    ~TrackingController() = default;

    void run();

private:
    void executeTracking(const magnet_msgs::TrackMagnetGoalConstPtr &goal);
    bool computeTargetPose(geometry_msgs::PoseStamped &target_pose);
    void preemptCallback();

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<magnet_msgs::TrackMagnetAction> tracking_server_;
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> move_group_client_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor_array_frame_;
    std::string magnet_frame_;
    std::string magnet_arm_planning_group_;
    std::string world_frame_;

    bool tracking_active_;
    geometry_msgs::Pose target_offset_;
};

} // namespace mag_tracking_control

#endif // MAG_TRACKING_CONTROL_TRACKING_CONTROLLER_HPP