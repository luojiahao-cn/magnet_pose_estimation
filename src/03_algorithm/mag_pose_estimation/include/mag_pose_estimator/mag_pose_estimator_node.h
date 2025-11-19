#pragma once

#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_msgs/MagSensorBatch.h>
#include <mag_core_msgs/MagnetPose.h>
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
#include "mag_pose_estimator/ekf_estimator.h"
#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/optimizer_estimator.h"

namespace mag_pose_estimator {

/**
 * @brief 节点配置结构体
 * 
 * 包含从 ROS 参数服务器加载的所有配置参数。
 */
struct MagPoseEstimatorConfig
{
    // 坐标系配置
    std::string output_frame;  // 输出姿态的参考坐标系

    // 话题配置
    std::string mag_topic;  // 单个磁传感器测量数据输入话题（向后兼容）
    std::string batch_topic;  // 批量传感器数据输入话题（优先使用）
    std::string pose_topic;  // 姿态估计结果输出话题

    // 估计器参数
    std::string estimator_type;  // 估计器类型（"ekf" 或 "optimizer"）
    int min_sensors;  // 批量优化器所需的最小传感器数量
    double tf_timeout;  // TF 变换查询超时时间（秒）

    // 预处理器参数
    bool enable_calibration;  // 是否启用软/硬铁校准
    Eigen::Matrix3d soft_iron_matrix;  // 软铁校准矩阵（3×3）
    Eigen::Vector3d hard_iron_offset;  // 硬铁偏移向量 [x, y, z] (mT)
    bool enable_filter;  // 是否启用低通滤波器
    double low_pass_alpha;  // 低通滤波器系数（0-1）

    // 估计器核心参数
    double position_gain;  // 位置增益系数
    double process_noise_position;  // 位置过程噪声方差
    double process_noise_orientation;  // 姿态过程噪声方差
    double measurement_noise;  // 测量噪声方差
    int optimizer_iterations;  // 传统优化器迭代次数
    double optimizer_damping;  // 传统优化器阻尼系数
    Eigen::Vector3d world_field;  // 世界坐标系下的磁场向量 [x, y, z] (mT)

    // 优化器参数
    Eigen::Vector3d initial_position;  // 初始位置估计 [x, y, z] (米)
    Eigen::Vector3d initial_direction;  // 初始磁矩方向向量 [x, y, z]（归一化）
    double initial_strength;  // 初始磁矩强度 (Am²)
    double strength_delta;  // 磁矩强度优化范围 (±delta)
    bool optimize_strength;  // 是否优化磁矩强度
    int max_iterations;  // Ceres 优化器最大迭代次数
    double function_tolerance;  // 函数值收敛容差
    double gradient_tolerance;  // 梯度收敛容差
    double parameter_tolerance;  // 参数收敛容差
    int num_threads;  // 并行计算线程数
    bool minimizer_progress;  // 是否输出优化进度
    std::string linear_solver;  // 线性求解器类型
};

/**
 * @brief 磁铁姿态估计 ROS 节点
 * 
 * 主要功能：
 * 1. 订阅磁传感器测量数据（MagSensorData，单位：mT）
 * 2. 对测量数据进行预处理（校准、滤波）
 * 3. 使用估计器（EKF 或优化器）估计磁铁姿态
 * 4. 发布估计结果（MagnetPose，包含位置、姿态和磁矩强度）
 * 
 * 支持两种工作模式：
 * - EKF 模式：使用扩展卡尔曼滤波器，适合实时单次测量更新
 * - 优化器模式：使用批量优化器，需要多个传感器数据，能够同时估计位置和姿态
 * 
 * 注意：所有磁场数据的单位均为 mT（毫特斯拉）。
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

  /**
   * @brief 主函数
   * @param argc 命令行参数数量
   * @param argv 命令行参数数组
   * @return 退出码
   */
  static int main(int argc, char **argv);

private:
  /**
   * @brief 缓存的测量数据结构
   * 
   * 用于批量优化器，缓存每个传感器的最新测量数据。
   */
  struct CachedMeasurement {
    ros::Time stamp;  // 时间戳
    std::string frame_id;  // 传感器坐标系名称
    Eigen::Vector3d field;  // 磁场测量值 [Bx, By, Bz] (mT)
  };

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
   * @brief 解析 3D 向量
   * @param node XML-RPC 节点
   * @param context 上下文路径
   * @return 3D 向量
   */
  Eigen::Vector3d parseVector3(const XmlRpc::XmlRpcValue &node, const std::string &context);

  /**
   * @brief 解析 3×3 矩阵
   * @param node XML-RPC 节点
   * @param context 上下文路径
   * @return 3×3 矩阵
   */
  Eigen::Matrix3d parseMatrix3x3(const XmlRpc::XmlRpcValue &node, const std::string &context);

  /**
   * @brief 初始化估计器
   * 创建估计器实例，设置配置，初始化批量优化器相关组件
   */
  void initializeEstimator();

  /**
   * @brief 从参数构建估计器配置
   * @return 估计器配置结构体
   */
  EstimatorConfig buildConfigFromParameters() const;

  /**
   * @brief 批量传感器数据回调函数（主要接口）
   * @param msg 批量传感器测量消息（单位：mT）
   * 
   * 处理流程：
   * 1. 从批量数据中提取所有传感器数据
   * 2. 通过预处理器处理数据
   * 3. 如果是批量优化器模式：直接构建批量数据并运行批量求解器
   * 4. 否则：逐个更新估计器并发布结果
   */
  void batchCallback(const mag_core_msgs::MagSensorBatchConstPtr &msg);

  /**
   * @brief 单个磁传感器数据回调函数（向后兼容）
   * @param msg 磁传感器测量消息（单位：mT）
   * 
   * 处理流程：
   * 1. 将 MagSensorData 转换为 sensor_msgs::MagneticField（保持 mT 单位）
   * 2. 通过预处理器处理数据
   * 3. 如果是批量优化器模式：缓存测量数据并运行批量求解器
   * 4. 否则：直接更新估计器并发布结果
   */
  void magCallback(const mag_core_msgs::MagSensorDataConstPtr &msg);

  /**
   * @brief 发布姿态估计结果
   * @param pose 估计的姿态
   * @param stamp 时间戳
   */
  void publishPose(const geometry_msgs::Pose &pose, const ros::Time &stamp);

  /**
   * @brief 缓存测量数据（用于批量优化器）
   * @param sensor_id 传感器 ID
   * @param processed 预处理后的磁场数据（单位：mT）
   * 
   * 每个传感器只保留最新一次读数，用于批量优化器同步使用。
   */
  void cacheMeasurement(uint32_t sensor_id, const sensor_msgs::MagneticField &processed);

  /**
   * @brief 构建批量测量数据
   * @param out_batch 输出的批量测量数据
   * @param out_latest_stamp 输出的最新时间戳（用于发布结果）
   * @return 是否成功构建（需要满足最小传感器数量要求）
   * 
   * 从缓存中提取所有有效测量数据，并通过 TF 转换到统一坐标系。
   * 返回批量数据中的最新时间戳，用于确保发布结果的时间戳准确。
   */
  bool buildBatch(std::vector<OptimizerMeasurement> &out_batch,
                  ros::Time &out_latest_stamp);

  /**
   * @brief 填充优化器测量数据
   * @param stamp 时间戳
   * @param frame_id 传感器坐标系名称
   * @param field 磁场向量 (mT)
   * @param out_meas 输出的优化器测量数据
   * @return 是否成功填充
   * 
   * 通过 TF 查询传感器位置，并将磁场向量转换到输出坐标系。
   */
  bool fillOptimizerMeasurement(const ros::Time &stamp,
                                const std::string &frame_id,
                                const Eigen::Vector3d &field,
                                OptimizerMeasurement &out_meas) const;

  /**
   * @brief 运行批量求解器
   * 
   * 当传感器数量满足要求时，调用批量优化器估计姿态并发布结果。
   */
  void runBatchSolver();

  ros::NodeHandle nh_;  // 全局节点句柄
  ros::NodeHandle pnh_;  // 私有节点句柄
  ros::Subscriber batch_sub_;  // 批量传感器数据订阅者（主要接口）
  ros::Subscriber mag_sub_;  // 单个磁传感器数据订阅者（向后兼容）
  ros::Publisher pose_pub_;  // 姿态估计结果发布者

  MagPreprocessor preprocessor_;  // 数据预处理器
  std::unique_ptr<EstimatorBase> estimator_;  // 估计器实例
  OptimizerEstimator *optimizer_backend_ = nullptr;  // 优化器后端指针（用于批量优化）

  std::string estimator_type_;  // 估计器类型
  std::string mag_topic_;  // 单个磁传感器数据话题（向后兼容）
  std::string batch_topic_;  // 批量传感器数据话题（优先使用）
  std::string pose_topic_;  // 姿态估计结果话题
  std::string output_frame_;  // 输出坐标系

  // 估计器参数
  double position_gain_;
  double process_noise_position_;
  double process_noise_orientation_;
  double measurement_noise_;
  int optimizer_iterations_;
  double optimizer_damping_;
  Eigen::Vector3d world_field_vector_;  // 世界磁场向量 (mT)
  OptimizerParameters optimizer_params_;  // 优化器参数

  size_t min_sensors_;  // 批量优化器所需的最小传感器数量
  double tf_timeout_;  // TF 查询超时时间
  bool use_batch_optimizer_ = false;  // 是否使用批量优化器模式

  std::map<int, CachedMeasurement> measurement_cache_;  // 测量数据缓存（传感器 ID -> 测量数据）

  tf2_ros::Buffer tf_buffer_;  // TF 缓冲区
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // TF 监听器
};

}  // 命名空间 mag_pose_estimator
