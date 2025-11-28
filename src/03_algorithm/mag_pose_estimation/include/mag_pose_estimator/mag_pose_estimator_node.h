#pragma once

#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_msgs/MagSensorBatch.h>
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
#include "mag_pose_estimator/mag_pose_estimator_ekf.h"
#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/mag_pose_estimator_optimization.h"

namespace mag_pose_estimator {

/**
 * @brief 节点配置结构体
 */
struct MagPoseEstimatorConfig {
  std::string output_frame;  ///< 输出姿态的参考坐标系
  std::string magnet_frame;  ///< 磁铁坐标系名称（用于发布 TF）
  bool enable_tf;  ///< 是否发布 TF
  std::string mag_topic;  ///< 单个传感器数据输入话题
  std::string batch_topic;  ///< 批量传感器数据输入话题
  std::string pose_topic;  ///< 姿态估计结果输出话题
  std::string estimator_type;  ///< 估计器类型（"ekf" 或 "optimizer"）
  double tf_timeout;  ///< TF 查询超时时间（秒）
  bool enable_calibration;  ///< 是否启用软/硬铁校准
  Eigen::Matrix3d soft_iron_matrix;  ///< 软铁校准矩阵（3×3）
  Eigen::Vector3d hard_iron_offset;  ///< 硬铁偏移向量 [x, y, z] (mT)
  bool enable_filter;  ///< 是否启用低通滤波器
  double low_pass_alpha;  ///< 低通滤波器系数（0-1）
  double position_gain;  ///< 位置增益系数（EKF）
  double process_noise_position;  ///< 位置过程噪声方差（EKF）
  double process_noise_orientation;  ///< 姿态过程噪声方差（EKF）
  double measurement_noise;  ///< 测量噪声方差（EKF）
  Eigen::Vector3d world_field;  ///< 世界坐标系磁场向量 [x, y, z] (mT，EKF)
  int min_sensors;  ///< 最小有效传感器数量（optimizer）
  Eigen::Vector3d initial_position;  ///< 初始位置估计 [x, y, z] (米，optimizer)
  Eigen::Vector3d initial_direction;  ///< 初始磁矩方向向量（归一化，optimizer）
  double initial_strength;  ///< 初始磁矩强度 (Am²，optimizer)
  double strength_delta;  ///< 磁矩强度优化范围 (±delta，optimizer)
  bool optimize_strength;  ///< 是否优化磁矩强度（optimizer）
  int max_iterations;  ///< 最大迭代次数（optimizer）
  double function_tolerance;  ///< 函数值收敛容差（optimizer）
  double gradient_tolerance;  ///< 梯度收敛容差（optimizer）
  double parameter_tolerance;  ///< 参数收敛容差（optimizer）
  int num_threads;  ///< 并行计算线程数（optimizer）
  bool minimizer_progress;  ///< 是否输出优化进度（optimizer）
  std::string linear_solver;  ///< 线性求解器类型（optimizer）
  double max_acceptable_residual;  ///< 可接受的最大平均残差 (mT)，超过此值即使优化收敛也认为失败（optimizer）
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
   * @brief 批量传感器数据回调函数
   * @param msg 批量传感器测量消息
   */
  void batchCallback(const mag_core_msgs::MagSensorBatchConstPtr &msg);

  /**
   * @brief 单个磁传感器数据回调函数
   * @param msg 磁传感器测量消息
   */
  void magCallback(const mag_core_msgs::MagSensorDataConstPtr &msg);

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
   * @param sensor_data 传感器原始数据
   * @return 处理后的磁场数据
   */
  sensor_msgs::MagneticField convertAndProcess(const mag_core_msgs::MagSensorData &sensor_data);

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
  ros::Subscriber batch_sub_;  // 批量传感器数据订阅者（主要接口）
  ros::Subscriber mag_sub_;  // 单个磁传感器数据订阅者
  ros::Publisher pose_pub_;  // 姿态估计结果发布者

  MagPreprocessor preprocessor_;  ///< 数据预处理器
  std::unique_ptr<EstimatorBase> estimator_;  ///< 估计器实例

  std::string estimator_type_;  ///< 估计器类型
  std::string mag_topic_;  ///< 单个磁传感器数据话题
  std::string batch_topic_;  ///< 批量传感器数据话题
  std::string pose_topic_;  ///< 姿态估计结果话题
  std::string output_frame_;  ///< 输出坐标系
  std::string magnet_frame_;  ///< 磁铁坐标系名称
  bool enable_tf_;  ///< 是否发布 TF

  EKFParameters ekf_params_;  ///< EKF 估计器参数
  OptimizerParameters optimizer_params_;  ///< 优化器参数

  double tf_timeout_;  ///< TF 查询超时时间

  tf2_ros::Buffer tf_buffer_;  ///< TF 缓冲区
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< TF 监听器
  tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< TF 发布器
};

}  // 命名空间 mag_pose_estimator
