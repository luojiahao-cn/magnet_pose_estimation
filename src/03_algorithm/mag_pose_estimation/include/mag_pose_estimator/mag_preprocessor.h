#pragma once

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>

namespace mag_pose_estimator
{
  struct MagPoseEstimatorConfig;

  /**
   * @brief 磁场数据预处理器 - 专注于背景采集和基础滤波
   */
  class MagPreprocessor
  {
  public:
    MagPreprocessor();
    void configure(const MagPoseEstimatorConfig &config);

    /**
     * @brief 处理传感器数据：采样背景或减去背景
     * @param msg 原始磁场消息
     * @param sensor_id 传感器ID
     */
    sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg, uint32_t sensor_id);

    /**
     * @brief 背景采样是否完成
     */
    bool isBackgroundCollected() const { return background_collected_; }

  private:
    bool enable_background_removal_ = true;
    double background_duration_ = 3.0;
    bool enable_filter_ = false;
    double low_pass_alpha_ = 0.3;

    bool background_collected_ = false;
    ros::Time sampling_start_time_;

    std::map<uint32_t, Eigen::Vector3d> bias_sum_;
    std::map<uint32_t, int> bias_count_;
    std::map<uint32_t, Eigen::Vector3d> background_bias_;

    bool filter_initialized_ = false;
    Eigen::Vector3d filtered_field_;
  };

} // 命名空间 mag_pose_estimator
