#pragma once

#include <mag_core_msgs/MagSensorArray.h>
#include <mag_core_msgs/MagnetPose.h>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"
#include "mag_pose_estimator/estimator_ekf.h"
#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/estimator_optimization.h"

namespace mag_pose_estimator {

/**
 * @brief 节点配置结构体
 */
struct MagPoseEstimatorConfig {
  std::string output_frame;  ///< 输出姿态的参考坐标系
  std::string magnet_frame;  ///< 磁铁坐标系名称（用于发布 TF）
  bool enable_tf;  ///< 是否发布 TF
  std::string array_topic;  ///< 传感器数组数据输入话题 (MagSensorArray)
  std::string pose_topic;  ///< 姿态估计结果输出话题
  std::string estimator_type;  ///< 估计器类型（"ekf", "optimizer", "window_optimizer"）
  double tf_timeout;  ///< TF 查询超时时间（秒）
  bool enable_background_removal; ///< 是否启用初始背景磁场滤除（地磁滤除）
  double background_removal_duration; ///< 背景采样时长（秒）
  bool enable_filter;  ///< 是否启用低通滤波器
  double low_pass_alpha;  ///< 低通滤波器系数（0-1）
  double position_gain;  ///< 位置增益系数（EKF）
  double process_noise_position;  ///< 位置过程噪声方差（EKF）
  double process_noise_orientation;  ///< 姿态过程噪声方差（EKF）
  double measurement_noise;  ///< 测量噪声方差（EKF）
  Eigen::Vector3d world_field;  ///< 世界坐标系磁场向量 [x, y, z] (mT，EKF)
  Eigen::Vector3d initial_position;  ///< 初始位置估计 [x, y, z] (米，optimizer)
  Eigen::Vector3d initial_direction;  ///< 初始磁矩方向向量（归一化，optimizer）
  Eigen::Vector3d position_min;       ///< 位置优化下界 [x, y, z] (米，optimizer)
  Eigen::Vector3d position_max;       ///< 位置优化上界 [x, y, z] (米，optimizer)
  double initial_strength;  ///< 初始磁矩强度 (Am²，optimizer)
  double strength_delta;  ///< 磁矩强度优化范围 (±delta，optimizer)
  bool optimize_strength;  ///< 是否优化磁矩强度（optimizer）
  bool use_strength_limit; ///< 是否限制磁矩强度范围（optimizer）
  bool use_position_limit; ///< 是否限制位置范围（optimizer）
  int max_iterations;  ///< 最大迭代次数（optimizer）
  int num_threads;  ///< 并行计算线程数（optimizer）
  std::string linear_solver;  ///< 线性求解器类型（optimizer）
  int window_size;  ///< 滑动窗口长度 L（window_optimizer）
  double dt;  ///< 采样周期（秒，window_optimizer）
  double m0;  ///< 磁偶极矩模长 (Am²，window_optimizer)
  double mu0;  ///< 真空磁导率（window_optimizer）
  double sigma_meas;  ///< 测量噪声标准差（window_optimizer）
  double sigma_proc_p;  ///< 位置过程噪声标准差（window_optimizer）
  double sigma_proc_v;  ///< 速度过程噪声标准差（window_optimizer）
  double sigma_proc_u;  ///< 磁化方向过程噪声标准差（window_optimizer）
  double sigma_unit;  ///< 单位向量约束残差标准差（window_optimizer）
  int max_iters;  ///< 最大迭代次数（window_optimizer）
  double lambda_init;  ///< LM 初始阻尼参数（window_optimizer）
  bool verbose;  ///< 是否输出调试信息（window_optimizer）
};

/**
 * @brief 磁铁姿态估计 ROS 节点
 */
class MagPoseEstimatorNode {
public:
  /**
   * @brief 构造函数
   * @param nh 全局节点句柄
   * @param pnh 私有节点句柄
   *
   * 初始化流程：
   * 1. 从参数服务器加载配置
   * 2. 初始化预处理器
   * 3. 创建估计器实例
   * 4. 设置 ROS 订阅者和发布者
   */
  MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh);

  /**
   * @brief 工厂方法：创建估计器实例
   * @param type 估计器类型（"ekf" 或 "optimizer"）
   * @return 估计器实例指针
   */
  static std::unique_ptr<EstimatorBase> createEstimator(const std::string &type);

private:
  /**
   * @brief 设置 ROS 日志级别
   */
  void setLogLevel();

  /**
   * @brief 从参数服务器加载配置
   */
  void loadParameters();

  /**
   * @brief 解析 XML-RPC 配置为配置结构体
   * @param root XML-RPC 根节点
   * @param context 上下文路径（用于错误信息）
   * @return 配置结构体
   */
  MagPoseEstimatorConfig loadMagPoseEstimatorConfig(const XmlRpc::XmlRpcValue &root,
                                                     const std::string &context);

  /**
   * @brief 初始化估计器
   */
  void initializeEstimator();

  /**
   * @brief 从参数构建估计器配置
   * @return 估计器配置结构体
   */
  EstimatorConfig buildConfigFromParameters() const;

  /**
   * @brief 传感器数组数据回调函数 (MagSensorArray 类型)
   * @param msg 传感器数组消息
   */
  void arrayCallback(const mag_core_msgs::MagSensorArrayConstPtr &msg);

  /**
   * @brief 发布姿态估计结果
   * @param pose 估计的姿态
   * @param stamp 时间戳
   */
  void publishPose(const geometry_msgs::Pose &pose, const ros::Time &stamp);

  /**
   * @brief 查询传感器位置和变换
   * @param frame_id 传感器坐标系名称
   * @param stamp 时间戳
   * @param position 输出的传感器位置
   * @param transform 输出的完整 TF 变换（用于磁场向量转换）
   * @return 是否成功查询
   */
  bool querySensorTransform(const std::string &frame_id,
                             const ros::Time &stamp,
                             Eigen::Vector3d &position,
                             geometry_msgs::TransformStamped &transform) const;

  /**
   * @brief 转换并处理传感器数据
   * @param header 原始消息头
   * @param sensor_id 传感器 ID
   * @param field 磁场向量 [x, y, z] (mT)
   * @return 处理后的磁场数据
   */
  sensor_msgs::MagneticField convertAndProcess(const std_msgs::Header &header,
                                               int sensor_id,
                                               const Eigen::Vector3d &field);

  /**
   * @brief 处理测量数据并估计姿态
   * @param measurements 测量数据列表
   * @param pose_out 输出的姿态估计结果
   * @param error_out 输出的估计误差（可选）
   * @return 是否成功估计
   */
  bool processMeasurements(const std::vector<sensor_msgs::MagneticField> &measurements,
                           geometry_msgs::Pose &pose_out,
                           double *error_out = nullptr);

  ros::NodeHandle nh_;  // 全局节点句柄
  ros::NodeHandle pnh_;  // 私有节点句柄
  ros::Subscriber array_sub_;  // 传感器数组数据订阅者
  ros::Publisher pose_pub_;  // 姿态估计结果发布者

  MagPreprocessor preprocessor_;  ///< 数据预处理器
  std::unique_ptr<EstimatorBase> estimator_;  ///< 估计器实例

  std::string estimator_type_;  ///< 估计器类型
  std::string array_topic_;  ///< 传感器数组数据话题
  std::string pose_topic_;  ///< 姿态估计结果话题
  std::string output_frame_;  ///< 输出坐标系
  std::string magnet_frame_;  ///< 磁铁坐标系名称
  bool enable_tf_;  ///< 是否发布 TF

  EKFParameters ekf_params_;  ///< EKF 估计器参数
  OptimizerParameters optimizer_params_;  ///< 优化器参数
  WindowOptimizerParameters window_optimizer_params_;  ///< 窗口优化器参数

  double tf_timeout_;  ///< TF 查询超时时间

  tf2_ros::Buffer tf_buffer_;  ///< TF 缓冲区
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< TF 监听器
  tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< TF 发布器
};

}  // 命名空间 mag_pose_estimator
