#pragma once

#include <mag_core_msgs/MagSensorBatch.h>
#include <mag_device_arm/SetEndEffectorPose.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <vector>
#include <string>
#include <mutex>

namespace mag_sensor_calibration {

/**
 * @brief 校正数据采集配置
 */
struct CalibrationCollectConfig {
  // 机械臂配置
  std::string arm_name;
  std::vector<double> base_position;  // [x, y, z]
  std::vector<double> base_orientation;  // [roll, pitch, yaw]
  
  // 旋转配置
  std::vector<double> roll_range;     // [min, max]
  std::vector<double> pitch_range;    // [min, max]
  std::vector<double> yaw_range;      // [min, max]
  double step_size;                    // 角度步长（弧度）
  double stable_wait_time;             // 姿态稳定等待时间（秒）
  double sample_duration;              // 每个姿态采样时长（秒）
  
  // 话题配置
  std::string sensor_batch_topic;
  std::string arm_service_name;
  
  // 输出配置
  std::string data_file;
  std::string params_file;
};

/**
 * @brief 单个姿态的传感器数据
 */
struct PoseSensorData {
  geometry_msgs::Pose pose;  // 机械臂姿态
  std::map<uint32_t, std::vector<Eigen::Vector3d>> sensor_measurements;  // sensor_id -> 测量值列表
  ros::Time timestamp;
};

/**
 * @brief 校正数据采集器
 * 
 * 功能：
 * - 控制机械臂旋转传感器阵列，覆盖整个球面
 * - 在每个姿态下采集所有25个传感器的数据
 * - 保存原始数据用于后续分析
 */
class CalibrationDataCollector {
public:
  CalibrationDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh);
  
  /**
   * @brief 加载配置
   */
  bool loadConfig();
  
  /**
   * @brief 执行数据采集
   * @return 是否成功
   */
  bool collectData();
  
  /**
   * @brief 获取采集的数据
   */
  const std::vector<PoseSensorData>& getCollectedData() const { return collected_data_; }
  
private:
  /**
   * @brief 设置ROS日志级别
   */
  void setLogLevel();
  
  /**
   * @brief 生成旋转轨迹（球面均匀采样）
   * @return 姿态列表
   */
  std::vector<geometry_msgs::Pose> generateRotationTrajectory();
  
  /**
   * @brief 控制机械臂移动到指定姿态
   * @param pose 目标姿态
   * @return 是否成功
   */
  bool moveArmToPose(const geometry_msgs::Pose &pose);
  
  /**
   * @brief 等待机械臂稳定
   * @param wait_time 等待时间（秒）
   */
  void waitForStability(double wait_time);
  
  /**
   * @brief 采集传感器数据
   * @param duration 采集时长（秒）
   * @return 采集的数据
   */
  PoseSensorData sampleSensorData(double duration, const geometry_msgs::Pose &current_pose);
  
  /**
   * @brief 传感器批量数据回调
   */
  void sensorBatchCallback(const mag_core_msgs::MagSensorBatchConstPtr &msg);
  
  /**
   * @brief 从RPY创建姿态
   */
  geometry_msgs::Pose poseFromRpy(double roll, double pitch, double yaw);
  
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  
  CalibrationCollectConfig config_;
  
  // ROS接口
  ros::Subscriber sensor_batch_sub_;
  ros::ServiceClient arm_pose_client_;
  
  // 数据存储
  std::vector<PoseSensorData> collected_data_;
  std::mutex data_mutex_;
  
  // 临时数据缓存（用于采样）
  mag_core_msgs::MagSensorBatchConstPtr latest_batch_;
  std::mutex batch_mutex_;
  
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace mag_sensor_calibration

