#include "mag_pose_estimator/mag_pose_estimator_node.h"

namespace mag_pose_estimator {

MagPoseEstimatorNode::MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  loadParameters();

  preprocessor_.configure(pnh_);
  initializeEstimator();

  mag_sub_ = nh_.subscribe("/mag", 10, &MagPoseEstimatorNode::magCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
}

void MagPoseEstimatorNode::loadParameters() {
  pnh_.param<std::string>("estimator_type", estimator_type_, "ekf");
  pnh_.param<std::string>("pose_topic", pose_topic_, "mag_pose");
  pnh_.param<std::string>("output_frame", output_frame_, "map");

  pnh_.param("position_gain", position_gain_, 0.01);
  pnh_.param("process_noise_position", process_noise_position_, 1e-4);
  pnh_.param("process_noise_orientation", process_noise_orientation_, 1e-5);
  pnh_.param("measurement_noise", measurement_noise_, 1e-3);
  pnh_.param("optimizer_iterations", optimizer_iterations_, 15);
  pnh_.param("optimizer_damping", optimizer_damping_, 1e-3);

  std::vector<double> world_field_vec = {2.0e-5, 0.0, 4.8e-5};
  pnh_.param("world_field", world_field_vec, world_field_vec);
  if (world_field_vec.size() == 3) {
    world_field_vector_ = Eigen::Map<Eigen::Vector3d>(world_field_vec.data());
  } else {
    ROS_WARN("world_field parameter must contain 3 elements, using fallback");
    world_field_vector_ = Eigen::Vector3d(2.0e-5, 0.0, 4.8e-5);
  }
}

void MagPoseEstimatorNode::initializeEstimator() {
  estimator_ = createEstimator(estimator_type_);
  EstimatorConfig cfg = buildConfigFromParameters();
  estimator_->setConfig(cfg);
  estimator_->initialize();
}

EstimatorConfig MagPoseEstimatorNode::buildConfigFromParameters() const {
  EstimatorConfig cfg;
  cfg.world_field = world_field_vector_;
  cfg.process_noise_position = process_noise_position_;
  cfg.process_noise_orientation = process_noise_orientation_;
  cfg.measurement_noise = measurement_noise_;
  cfg.position_gain = position_gain_;
  cfg.optimizer_iterations = optimizer_iterations_;
  cfg.optimizer_damping = optimizer_damping_;
  return cfg;
}

void MagPoseEstimatorNode::magCallback(const sensor_msgs::MagneticFieldConstPtr &msg) {
  sensor_msgs::MagneticField processed = preprocessor_.process(*msg);
  estimator_->update(processed);
  geometry_msgs::Pose pose = estimator_->getPose();
  publishPose(pose, msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
}

void MagPoseEstimatorNode::publishPose(const geometry_msgs::Pose &pose, const ros::Time &stamp) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = output_frame_;
  msg.pose = pose;
  pose_pub_.publish(msg);
}

}  // namespace mag_pose_estimator
