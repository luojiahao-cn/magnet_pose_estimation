#include "mag_pose_estimator/mag_pose_estimator_node.h"
#include "mag_pose_estimator/mag_pose_estimator_config_loader.hpp"

#include <algorithm>
#include <vector>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace mag_pose_estimator {

MagPoseEstimatorNode::MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  // 构造流程：读取参数、配置预处理、创建估计器并建立 ROS 通信。
  loadParameters();

  initializeEstimator();

  mag_sub_ = nh_.subscribe(mag_topic_, 10, &MagPoseEstimatorNode::magCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
}

void MagPoseEstimatorNode::loadParameters() {
  const std::string ns = "mag_pose_estimator";

  XmlRpc::XmlRpcValue config;
  if (!pnh_.getParam("config", config)) {
    ROS_ERROR_STREAM(ns << ": Failed to get config parameter");
    return;
  }

  auto cfg = loadMagPoseEstimatorConfig(config, ns + "/config");

  // Set member variables
  estimator_type_ = cfg.estimator_type;
  mag_topic_ = cfg.mag_topic;
  pose_topic_ = cfg.pose_topic;
  output_frame_ = cfg.output_frame;
  min_sensors_ = static_cast<size_t>(cfg.min_sensors);
  tf_timeout_ = cfg.tf_timeout;
  position_gain_ = cfg.position_gain;
  process_noise_position_ = cfg.process_noise_position;
  process_noise_orientation_ = cfg.process_noise_orientation;
  measurement_noise_ = cfg.measurement_noise;
  optimizer_iterations_ = cfg.optimizer_iterations;
  optimizer_damping_ = cfg.optimizer_damping;
  world_field_vector_ = cfg.world_field;

  optimizer_params_.initial_position = cfg.optimizer.initial_position;
  optimizer_params_.initial_direction = cfg.optimizer.initial_direction;
  optimizer_params_.initial_strength = cfg.optimizer.initial_strength;
  optimizer_params_.strength_delta = cfg.optimizer.strength_delta;
  optimizer_params_.optimize_strength = cfg.optimizer.optimize_strength;
  optimizer_params_.max_iterations = cfg.optimizer.max_iterations;
  optimizer_params_.function_tolerance = cfg.optimizer.function_tolerance;
  optimizer_params_.gradient_tolerance = cfg.optimizer.gradient_tolerance;
  optimizer_params_.parameter_tolerance = cfg.optimizer.parameter_tolerance;
  optimizer_params_.num_threads = cfg.optimizer.num_threads;
  optimizer_params_.minimizer_progress = cfg.optimizer.minimizer_progress;
  optimizer_params_.linear_solver = cfg.optimizer.linear_solver;

  // Configure preprocessor
  preprocessor_.configure(cfg);
}

void MagPoseEstimatorNode::initializeEstimator() {
  // 构建估计器实例、推送配置，并按需初始化批量优化辅助组件。
  estimator_ = createEstimator(estimator_type_);
  EstimatorConfig cfg = buildConfigFromParameters();
  estimator_->setConfig(cfg);
  estimator_->initialize();
  optimizer_backend_ = dynamic_cast<OptimizerEstimator *>(estimator_.get());
  use_batch_optimizer_ = (optimizer_backend_ != nullptr) && (estimator_type_ == "optimizer");
  if (use_batch_optimizer_) {
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    measurement_cache_.clear();
  }
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
  cfg.optimizer = optimizer_params_;
  return cfg;
}

void MagPoseEstimatorNode::magCallback(const mag_core_msgs::MagSensorDataConstPtr &msg) {
  // 将自定义消息转换为 sensor_msgs，经过预处理后交给估计算法。
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header = msg->header;
  constexpr double kMilliTeslaToTesla = 1e-3;
  mag_msg.magnetic_field.x = msg->mag_x * kMilliTeslaToTesla;
  mag_msg.magnetic_field.y = msg->mag_y * kMilliTeslaToTesla;
  mag_msg.magnetic_field.z = msg->mag_z * kMilliTeslaToTesla;

  sensor_msgs::MagneticField processed = preprocessor_.process(mag_msg);
  if (use_batch_optimizer_) {
    cacheMeasurement(msg->sensor_id, processed);
    runBatchSolver();
    return;
  }

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

void MagPoseEstimatorNode::cacheMeasurement(uint32_t sensor_id, const sensor_msgs::MagneticField &processed) {
  // 每个传感器只保留最新一次读数，方便批量优化器同步使用。
  CachedMeasurement entry;
  entry.stamp = processed.header.stamp.isZero() ? ros::Time::now() : processed.header.stamp;
  entry.frame_id = processed.header.frame_id;
  entry.field = Eigen::Vector3d(processed.magnetic_field.x, processed.magnetic_field.y, processed.magnetic_field.z);
  measurement_cache_[static_cast<int>(sensor_id)] = entry;
}

bool MagPoseEstimatorNode::buildBatch(std::vector<OptimizerMeasurement> &out_batch) {
  if (measurement_cache_.size() < min_sensors_) {
    return false;
  }

  std::vector<int> stale_keys;
  out_batch.clear();
  out_batch.reserve(measurement_cache_.size());

  for (const auto &kv : measurement_cache_) {
    OptimizerMeasurement meas;
    if (fillOptimizerMeasurement(kv.second.stamp, kv.second.frame_id, kv.second.field, meas)) {
      meas.sensor_id = kv.first;
      out_batch.push_back(meas);
    } else {
      stale_keys.push_back(kv.first);
    }
  }

  for (int key : stale_keys) {
    measurement_cache_.erase(key);
  }

  return out_batch.size() >= min_sensors_;
}

bool MagPoseEstimatorNode::fillOptimizerMeasurement(const ros::Time &stamp,
                                                   const std::string &frame_id,
                                                   const Eigen::Vector3d &field,
                                                   OptimizerMeasurement &out_meas) const {
  if (frame_id.empty()) {
    return false;
  }

  try {
    // 查询传感器相对于 output_frame_ 的位姿，并把测量转换到该坐标系下。
    geometry_msgs::TransformStamped tf_sensor = tf_buffer_.lookupTransform(
        output_frame_, frame_id, stamp, ros::Duration(tf_timeout_));

    geometry_msgs::Vector3Stamped v_sensor;
    v_sensor.header.frame_id = frame_id;
    v_sensor.header.stamp = stamp;
    v_sensor.vector.x = field.x();
    v_sensor.vector.y = field.y();
    v_sensor.vector.z = field.z();

    geometry_msgs::Vector3Stamped v_world;
    tf2::doTransform(v_sensor, v_world, tf_sensor);

    out_meas.magnetic_field = Eigen::Vector3d(v_world.vector.x, v_world.vector.y, v_world.vector.z);
    out_meas.sensor_position = Eigen::Vector3d(tf_sensor.transform.translation.x,
                                               tf_sensor.transform.translation.y,
                                               tf_sensor.transform.translation.z);
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "mag_pose_estimator: TF lookup failed for %s -> %s: %s",
                      frame_id.c_str(), output_frame_.c_str(), ex.what());
    return false;
  }
}

void MagPoseEstimatorNode::runBatchSolver() {
  if (!use_batch_optimizer_ || !optimizer_backend_) {
    return;
  }

  std::vector<OptimizerMeasurement> batch;
  if (!buildBatch(batch)) {
    return;
  }

  // 传感器数量满足要求后调用 Ceres 后端，成功即发布结果。
  geometry_msgs::Pose pose;
  double error = 0.0;
  if (optimizer_backend_->estimateFromBatch(batch, pose, &error)) {
    publishPose(pose, ros::Time::now());
    measurement_cache_.clear();
  } else {
    ROS_WARN_THROTTLE(1.0, "mag_pose_estimator: optimizer batch solve failed");
  }
}

}  // 命名空间 mag_pose_estimator
