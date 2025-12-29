#include "mag_sensor_calibration/calibration_data_collector.h"

#include <mag_core_utils/xmlrpc_utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <cmath>
#include <algorithm>

namespace mag_sensor_calibration {
namespace xml = mag_core_utils::xmlrpc;

CalibrationDataCollector::CalibrationDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
  setLogLevel();
}

void CalibrationDataCollector::setLogLevel() {
  std::string log_level;
  if (pnh_.getParam("logging_level", log_level)) {
    if (log_level == "DEBUG") {
      ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Debug);
    } else if (log_level == "INFO") {
      ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Info);
    } else if (log_level == "WARN") {
      ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Warn);
    } else if (log_level == "ERROR") {
      ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Error);
    }
  }
}

bool CalibrationDataCollector::loadConfig() {
  XmlRpc::XmlRpcValue root;
  if (!pnh_.getParam("config", root)) {
    ROS_ERROR("[calibration_collector] 无法加载配置参数 'config'");
    return false;
  }
  
  try {
    const auto config_ctx = "config";
    const auto &config = xml::requireStructField(root, "config", config_ctx);
    
    // 机械臂配置
    const auto arm_ctx = xml::makeContext(config_ctx, "arm");
    const auto &arm = xml::requireStructField(config, "arm", config_ctx);
    config_.arm_name = xml::requireStringField(arm, "name", arm_ctx);
    {
      const auto &pos_array = xml::requireArrayField(arm, "base_position", arm_ctx);
      config_.base_position.clear();
      for (int i = 0; i < pos_array.size(); ++i) {
        config_.base_position.push_back(xml::readNumber(pos_array[i], xml::makeContext(arm_ctx, "base_position[" + std::to_string(i) + "]")));
      }
    }
    {
      const auto &ori_array = xml::requireArrayField(arm, "base_orientation", arm_ctx);
      config_.base_orientation.clear();
      for (int i = 0; i < ori_array.size(); ++i) {
        config_.base_orientation.push_back(xml::readNumber(ori_array[i], xml::makeContext(arm_ctx, "base_orientation[" + std::to_string(i) + "]")));
      }
    }
    
    // 旋转配置
    const auto rot_ctx = xml::makeContext(config_ctx, "rotation");
    const auto &rotation = xml::requireStructField(config, "rotation", config_ctx);
    {
      const auto &roll_array = xml::requireArrayField(rotation, "roll_range", rot_ctx);
      config_.roll_range.clear();
      for (int i = 0; i < roll_array.size(); ++i) {
        config_.roll_range.push_back(xml::readNumber(roll_array[i], xml::makeContext(rot_ctx, "roll_range[" + std::to_string(i) + "]")));
      }
    }
    {
      const auto &pitch_array = xml::requireArrayField(rotation, "pitch_range", rot_ctx);
      config_.pitch_range.clear();
      for (int i = 0; i < pitch_array.size(); ++i) {
        config_.pitch_range.push_back(xml::readNumber(pitch_array[i], xml::makeContext(rot_ctx, "pitch_range[" + std::to_string(i) + "]")));
      }
    }
    {
      const auto &yaw_array = xml::requireArrayField(rotation, "yaw_range", rot_ctx);
      config_.yaw_range.clear();
      for (int i = 0; i < yaw_array.size(); ++i) {
        config_.yaw_range.push_back(xml::readNumber(yaw_array[i], xml::makeContext(rot_ctx, "yaw_range[" + std::to_string(i) + "]")));
      }
    }
    config_.step_size = xml::readNumber(rotation, "step_size", rot_ctx, 0.2);
    config_.stable_wait_time = xml::readNumber(rotation, "stable_wait_time", rot_ctx, 2.0);
    config_.sample_duration = xml::readNumber(rotation, "sample_duration", rot_ctx, 1.0);
    
    // 话题配置
    const auto topics_ctx = xml::makeContext(config_ctx, "topics");
    const auto &topics = xml::requireStructField(config, "topics", config_ctx);
    config_.sensor_batch_topic = xml::requireStringField(topics, "sensor_batch", topics_ctx);
    config_.arm_service_name = xml::requireStringField(topics, "arm_service", topics_ctx);
    
    // 输出配置
    const auto output_ctx = xml::makeContext(config_ctx, "output");
    const auto &output = xml::requireStructField(config, "output", config_ctx);
    config_.data_file = xml::requireStringField(output, "data_file", output_ctx);
    config_.params_file = xml::requireStringField(output, "params_file", output_ctx);
    
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[calibration_collector] 配置加载失败: " << e.what());
    return false;
  }
}

bool CalibrationDataCollector::collectData() {
  // 设置订阅者和服务客户端
  sensor_batch_sub_ = nh_.subscribe<mag_core_msgs::MagSensorBatch>(
      config_.sensor_batch_topic, 10,
      &CalibrationDataCollector::sensorBatchCallback, this);
  
  arm_pose_client_ = nh_.serviceClient<mag_device_arm::SetEndEffectorPose>(
      config_.arm_service_name);
  
  if (!arm_pose_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("[calibration_collector] 机械臂服务不可用: %s", config_.arm_service_name.c_str());
    return false;
  }
  
  // 生成旋转轨迹
  std::vector<geometry_msgs::Pose> trajectory = generateRotationTrajectory();
  ROS_INFO("[calibration_collector] 生成了 %zu 个姿态点", trajectory.size());
  
  collected_data_.clear();
  collected_data_.reserve(trajectory.size());
  
  // 遍历每个姿态，采集数据
  for (size_t i = 0; i < trajectory.size(); ++i) {
    ROS_INFO("[calibration_collector] 采集进度: %zu/%zu", i + 1, trajectory.size());
    
    // 移动到目标姿态
    if (!moveArmToPose(trajectory[i])) {
      ROS_WARN("[calibration_collector] 无法移动到姿态 %zu，跳过", i);
      continue;
    }
    
    // 等待稳定
    waitForStability(config_.stable_wait_time);
    
    // 采集数据
    PoseSensorData pose_data = sampleSensorData(config_.sample_duration, trajectory[i]);
    collected_data_.push_back(pose_data);
    
    ROS_INFO("[calibration_collector] 姿态 %zu 采集完成，获得 %zu 个传感器的数据",
             i, pose_data.sensor_measurements.size());
  }
  
  ROS_INFO("[calibration_collector] 数据采集完成，共采集 %zu 个姿态的数据", collected_data_.size());
  
  return !collected_data_.empty();
}

std::vector<geometry_msgs::Pose> CalibrationDataCollector::generateRotationTrajectory() {
  std::vector<geometry_msgs::Pose> trajectory;
  
  // 生成角度网格
  std::vector<double> roll_angles, pitch_angles, yaw_angles;
  
  for (double r = config_.roll_range[0]; r <= config_.roll_range[1]; r += config_.step_size) {
    roll_angles.push_back(r);
  }
  for (double p = config_.pitch_range[0]; p <= config_.pitch_range[1]; p += config_.step_size) {
    pitch_angles.push_back(p);
  }
  for (double y = config_.yaw_range[0]; y <= config_.yaw_range[1]; y += config_.step_size) {
    yaw_angles.push_back(y);
  }
  
  // 生成所有组合
  for (double roll : roll_angles) {
    for (double pitch : pitch_angles) {
      for (double yaw : yaw_angles) {
        geometry_msgs::Pose pose = poseFromRpy(roll, pitch, yaw);
        pose.position.x = config_.base_position[0];
        pose.position.y = config_.base_position[1];
        pose.position.z = config_.base_position[2];
        trajectory.push_back(pose);
      }
    }
  }
  
  return trajectory;
}

bool CalibrationDataCollector::moveArmToPose(const geometry_msgs::Pose &pose) {
  mag_device_arm::SetEndEffectorPose srv;
  srv.request.arm = config_.arm_name;
  srv.request.target = pose;
  srv.request.velocity_scaling = 0.3;
  srv.request.acceleration_scaling = 0.3;
  srv.request.execute = true;
  
  if (!arm_pose_client_.call(srv)) {
    ROS_ERROR("[calibration_collector] 调用机械臂服务失败");
    return false;
  }
  
  if (!srv.response.success) {
    ROS_WARN_STREAM("[calibration_collector] 机械臂移动失败: " << srv.response.message);
    return false;
  }
  
  return true;
}

void CalibrationDataCollector::waitForStability(double wait_time) {
  ros::Duration(wait_time).sleep();
}

PoseSensorData CalibrationDataCollector::sampleSensorData(double duration, const geometry_msgs::Pose &current_pose) {
  PoseSensorData pose_data;
  pose_data.pose = current_pose;
  pose_data.timestamp = ros::Time::now();
  
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = start_time + ros::Duration(duration);
  
  std::map<uint32_t, std::vector<Eigen::Vector3d>> measurements;
  
  while (ros::Time::now() < end_time) {
    ros::spinOnce();
    
    std::lock_guard<std::mutex> lock(batch_mutex_);
    if (latest_batch_) {
      for (const auto &sensor : latest_batch_->measurements) {
        Eigen::Vector3d field(sensor.mag_x, sensor.mag_y, sensor.mag_z);
        measurements[sensor.sensor_id].push_back(field);
      }
    }
    
    ros::Duration(0.01).sleep();  // 10ms采样间隔
  }
  
  pose_data.sensor_measurements = measurements;
  return pose_data;
}

void CalibrationDataCollector::sensorBatchCallback(const mag_core_msgs::MagSensorBatchConstPtr &msg) {
  std::lock_guard<std::mutex> lock(batch_mutex_);
  latest_batch_ = msg;
}

geometry_msgs::Pose CalibrationDataCollector::poseFromRpy(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  
  geometry_msgs::Pose pose;
  pose.orientation = tf2::toMsg(q);
  return pose;
}

}  // namespace mag_sensor_calibration

