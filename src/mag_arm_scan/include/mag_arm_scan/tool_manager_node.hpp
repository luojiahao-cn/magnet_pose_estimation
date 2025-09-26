#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>

class ToolManagerNode {
public:
  ToolManagerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  bool attachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool detachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

private:
  bool attachWithParams(std_srvs::Trigger::Response &res);
  void publishToolTF(const ros::TimerEvent&);

  ros::NodeHandle nh_, pnh_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string parent_link_;
  std::string object_name_;
  std::string mesh_path_;
  std::vector<double> xyz_;
  std::vector<double> rpy_;
  std::vector<double> scale_;
  std::vector<std::string> touch_links_;
  bool auto_attach_ {true};
  ros::Timer auto_attach_timer_;

  // 支架末端参数
  std::vector<double> tip_xyz_;
  std::vector<double> tip_rpy_;

  ros::Timer tf_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::ServiceServer srv_attach_;
  ros::ServiceServer srv_detach_;
};
