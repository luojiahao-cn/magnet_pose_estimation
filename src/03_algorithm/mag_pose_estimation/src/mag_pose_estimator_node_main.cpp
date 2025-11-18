#include <ros/ros.h>

#include "mag_pose_estimator/mag_pose_estimator_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_pose_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 将运行期逻辑保持最小化，方便复用的节点类独立测试。
  mag_pose_estimator::MagPoseEstimatorNode node(nh, pnh);
  ros::spin();
  return 0;
}
