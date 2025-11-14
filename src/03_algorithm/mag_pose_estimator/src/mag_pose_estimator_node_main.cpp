#include <ros/ros.h>

#include "mag_pose_estimator/mag_pose_estimator_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_pose_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mag_pose_estimator::MagPoseEstimatorNode node(nh, pnh);
  ros::spin();
  return 0;
}
