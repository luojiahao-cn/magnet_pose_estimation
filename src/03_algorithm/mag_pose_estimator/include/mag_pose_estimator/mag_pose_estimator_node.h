#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"
#include "mag_pose_estimator/estimator_factory.h"
#include "mag_pose_estimator/mag_preprocessor.h"

namespace mag_pose_estimator {

class MagPoseEstimatorNode {
public:
  MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  void loadParameters();
  void initializeEstimator();
  EstimatorConfig buildConfigFromParameters() const;
  void magCallback(const sensor_msgs::MagneticFieldConstPtr &msg);
  void publishPose(const geometry_msgs::Pose &pose, const ros::Time &stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber mag_sub_;
  ros::Publisher pose_pub_;

  MagPreprocessor preprocessor_;
  std::unique_ptr<EstimatorBase> estimator_;

  std::string estimator_type_;
  std::string pose_topic_;
  std::string output_frame_;

  double position_gain_;
  double process_noise_position_;
  double process_noise_orientation_;
  double measurement_noise_;
  int optimizer_iterations_;
  double optimizer_damping_;
  Eigen::Vector3d world_field_vector_;
};

}  // namespace mag_pose_estimator
