#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mag_core_msgs/MagSensorData.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"
#include "mag_pose_estimator/estimator_factory.h"
#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/optimizer_estimator.h"

namespace mag_pose_estimator {

class MagPoseEstimatorNode {
public:
  MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  struct CachedMeasurement {
    ros::Time stamp;
    std::string frame_id;
    Eigen::Vector3d field;
  };

  void loadParameters();
  void initializeEstimator();
  EstimatorConfig buildConfigFromParameters() const;
  void magCallback(const mag_core_msgs::MagSensorDataConstPtr &msg);
  void publishPose(const geometry_msgs::Pose &pose, const ros::Time &stamp);
  void cacheMeasurement(uint32_t sensor_id, const sensor_msgs::MagneticField &processed);
  bool buildBatch(std::vector<OptimizerMeasurement> &out_batch);
  bool fillOptimizerMeasurement(const ros::Time &stamp,
                                const std::string &frame_id,
                                const Eigen::Vector3d &field,
                                OptimizerMeasurement &out_meas) const;
  void runBatchSolver();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber mag_sub_;
  ros::Publisher pose_pub_;

  MagPreprocessor preprocessor_;
  std::unique_ptr<EstimatorBase> estimator_;
  OptimizerEstimator *optimizer_backend_ = nullptr;

  std::string estimator_type_;
  std::string mag_topic_;
  std::string pose_topic_;
  std::string output_frame_;

  double position_gain_;
  double process_noise_position_;
  double process_noise_orientation_;
  double measurement_noise_;
  int optimizer_iterations_;
  double optimizer_damping_;
  Eigen::Vector3d world_field_vector_;
  OptimizerParameters optimizer_params_;

  size_t min_sensors_;
  double tf_timeout_;
  bool use_batch_optimizer_ = false;

  std::map<int, CachedMeasurement> measurement_cache_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // 命名空间 mag_pose_estimator
