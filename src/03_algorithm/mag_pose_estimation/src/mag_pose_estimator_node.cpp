#include "mag_pose_estimator/mag_pose_estimator_node.h"

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

  preprocessor_.configure(pnh_);
  initializeEstimator();

  mag_sub_ = nh_.subscribe(mag_topic_, 10, &MagPoseEstimatorNode::magCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
}

void MagPoseEstimatorNode::loadParameters() {
  namespace rps = rosparam_shortcuts;
  const std::string ns = "mag_pose_estimator";
  std::size_t error = 0;
  int min_sensors_param = 0;
  std::vector<double> initial_pos;
  std::vector<double> initial_dir;
  std::vector<double> world_field_vec;

  error += !rps::get(ns, pnh_, "estimator_type", estimator_type_);
  error += !rps::get(ns, pnh_, "mag_topic", mag_topic_);
  error += !rps::get(ns, pnh_, "pose_topic", pose_topic_);
  error += !rps::get(ns, pnh_, "output_frame", output_frame_);
  error += !rps::get(ns, pnh_, "min_sensors", min_sensors_param);
  error += !rps::get(ns, pnh_, "tf_timeout", tf_timeout_);
  error += !rps::get(ns, pnh_, "position_gain", position_gain_);
  error += !rps::get(ns, pnh_, "process_noise_position", process_noise_position_);
  error += !rps::get(ns, pnh_, "process_noise_orientation", process_noise_orientation_);
  error += !rps::get(ns, pnh_, "measurement_noise", measurement_noise_);
  error += !rps::get(ns, pnh_, "optimizer_iterations", optimizer_iterations_);
  error += !rps::get(ns, pnh_, "optimizer_damping", optimizer_damping_);
  error += !rps::get(ns, pnh_, "world_field", world_field_vec);

  error += !rps::get(ns, pnh_, "optimizer/initial_position", initial_pos);
  error += !rps::get(ns, pnh_, "optimizer/initial_direction", initial_dir);
  error += !rps::get(ns, pnh_, "optimizer/initial_strength", optimizer_params_.initial_strength);
  error += !rps::get(ns, pnh_, "optimizer/strength_delta", optimizer_params_.strength_delta);
  error += !rps::get(ns, pnh_, "optimizer/optimize_strength", optimizer_params_.optimize_strength);
  error += !rps::get(ns, pnh_, "optimizer/max_iterations", optimizer_params_.max_iterations);
  error += !rps::get(ns, pnh_, "optimizer/function_tolerance", optimizer_params_.function_tolerance);
  error += !rps::get(ns, pnh_, "optimizer/gradient_tolerance", optimizer_params_.gradient_tolerance);
  error += !rps::get(ns, pnh_, "optimizer/parameter_tolerance", optimizer_params_.parameter_tolerance);
  error += !rps::get(ns, pnh_, "optimizer/num_threads", optimizer_params_.num_threads);
  error += !rps::get(ns, pnh_, "optimizer/minimizer_progress", optimizer_params_.minimizer_progress);
  error += !rps::get(ns, pnh_, "optimizer/linear_solver", optimizer_params_.linear_solver);

  if (min_sensors_param < 1) {
    ROS_ERROR("mag_pose_estimator: min_sensors 必须大于 0");
    ++error;
  } else {
    min_sensors_ = static_cast<size_t>(min_sensors_param);
  }

  if (world_field_vec.size() != 3) {
    ROS_ERROR("mag_pose_estimator: world_field 必须是长度为 3 的数组");
    ++error;
  } else {
    world_field_vector_ = Eigen::Vector3d(world_field_vec[0], world_field_vec[1], world_field_vec[2]);
  }

  if (initial_pos.size() != 3) {
    ROS_ERROR("mag_pose_estimator: optimizer/initial_position 必须是长度为 3 的数组");
    ++error;
  } else {
    optimizer_params_.initial_position = Eigen::Vector3d(initial_pos[0], initial_pos[1], initial_pos[2]);
  }

  if (initial_dir.size() != 3) {
    ROS_ERROR("mag_pose_estimator: optimizer/initial_direction 必须是长度为 3 的数组");
    ++error;
  } else {
    Eigen::Vector3d dir(initial_dir[0], initial_dir[1], initial_dir[2]);
    if (dir.norm() < 1e-9) {
      ROS_ERROR("mag_pose_estimator: optimizer/initial_direction 不能为零向量");
      ++error;
    } else {
      optimizer_params_.initial_direction = dir.normalized();
    }
  }

  rps::shutdownIfError(ns, error);
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
