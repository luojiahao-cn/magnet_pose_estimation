#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/mag_pose_estimator_node.h"

#include <Eigen/Geometry>
#include <string>

namespace mag_pose_estimator {

/**
 * @brief 构造函数
 */
MagPreprocessor::MagPreprocessor()
    : low_pass_alpha_(0.3),
      enable_filter_(true),
      filter_initialized_(false) {
}

/**
 * @brief 配置预处理器参数
 */
void MagPreprocessor::configure(const MagPoseEstimatorConfig &config) {
  enable_filter_ = config.enable_filter;
  enable_background_removal_ = config.enable_background_removal;
  background_duration_ = config.background_removal_duration;
  low_pass_alpha_ = config.low_pass_alpha;

  // 如果禁用了背景滤除，直接标记为已采集完成（不进行任何减偏置操作）
  if (!enable_background_removal_) {
    background_collected_ = true;
  }
}

/**
 * @brief 处理磁场测量数据
 */
sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg, uint32_t sensor_id) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  // 1. 初始背景磁场采样
  if (!background_collected_) {
    if (sampling_start_time_.isZero()) {
      sampling_start_time_ = ros::Time::now();
      ROS_INFO("[mag_preprocessor] 开始 %.1f 秒静默背景采样，请远离磁铁...", background_duration_);
    }

    double elapsed = (ros::Time::now() - sampling_start_time_).toSec();
    if (elapsed < background_duration_) {
      if (bias_count_.find(sensor_id) == bias_count_.end()) {
        bias_sum_[sensor_id] = field;
        bias_count_[sensor_id] = 1;
      } else {
        bias_sum_[sensor_id] += field;
        bias_count_[sensor_id]++;
      }

      // 采样期间返回 0，不参与计算
      sensor_msgs::MagneticField zero_msg = msg;
      zero_msg.magnetic_field.x = 0; zero_msg.magnetic_field.y = 0; zero_msg.magnetic_field.z = 0;
      return zero_msg;
    } else {
      // 计算每个传感器的平均背景（地磁等静态场）
      for (auto const& [id, sum] : bias_sum_) {
        if (bias_count_[id] > 0) {
          background_bias_[id] = sum / bias_count_[id];
        }
      }
      background_collected_ = true;
      ROS_INFO("[mag_preprocessor] 背景采样完成，共捕获 %zu 个传感器的静态偏置。", background_bias_.size());
    }
  }

  // 2. 减去采集到的背景
  if (enable_background_removal_ && background_bias_.count(sensor_id)) {
    field -= background_bias_[sensor_id];
  }

  // 3. 低通滤波
  if (enable_filter_) {
    if (!filter_initialized_) {
      filtered_field_ = field;
      filter_initialized_ = true;
    } else {
      filtered_field_ = low_pass_alpha_ * field + (1.0 - low_pass_alpha_) * filtered_field_;
    }
    field = filtered_field_;
  }

  sensor_msgs::MagneticField out_msg = msg;
  out_msg.magnetic_field.x = field.x();
  out_msg.magnetic_field.y = field.y();
  out_msg.magnetic_field.z = field.z();
  return out_msg;
}

}  // 命名空间 mag_pose_estimator
