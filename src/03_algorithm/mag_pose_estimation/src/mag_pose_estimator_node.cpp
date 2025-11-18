/**
 * @file mag_pose_estimator_node.cpp
 * @brief 磁体姿态估计节点实现
 * 
 * 实现磁体姿态估计 ROS 节点的核心功能，包括配置加载、估计器初始化、
 * 传感器数据处理和姿态估计结果发布。
 */

// 项目头文件
#include "mag_pose_estimator/mag_pose_estimator_node.h"
#include "mag_pose_estimator/mag_preprocessor.h"

// 标准库
#include <algorithm>
#include <vector>

// ROS 相关
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// 工具库
#include <mag_core_utils/xmlrpc_utils.hpp>

// XML-RPC
#include <XmlRpcValue.h>

namespace mag_pose_estimator {

// ============================================================================
// 构造函数和初始化
// ============================================================================

/**
 * @brief 构造函数
 * @param nh 全局节点句柄
 * @param pnh 私有节点句柄
 * 
 * 初始化流程：
 * 1. 从参数服务器加载配置
 * 2. 初始化估计器和预处理器
 * 3. 设置 ROS 订阅者和发布者
 */
MagPoseEstimatorNode::MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  loadParameters();
  initializeEstimator();

  mag_sub_ = nh_.subscribe(mag_topic_, 10, &MagPoseEstimatorNode::magCallback, this);
  pose_pub_ = nh_.advertise<mag_core_msgs::MagnetPose>(pose_topic_, 10);
}

/**
 * @brief 从参数服务器加载配置
 * 
 * 从私有命名空间的 "config" 参数加载所有配置，包括：
 * - 坐标系和话题配置
 * - 估计器参数
 * - 预处理器参数
 * - 优化器参数
 */
void MagPoseEstimatorNode::loadParameters() {
  const std::string ns = "mag_pose_estimator";

  XmlRpc::XmlRpcValue config;
  if (!pnh_.getParam("config", config)) {
    ROS_ERROR_STREAM(ns << ": Failed to get config parameter");
    return;
  }

  auto cfg = loadMagPoseEstimatorConfig(config, ns + "/config");

  // 设置成员变量
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
  world_field_vector_ = cfg.world_field;  // 单位：mT

  // 优化器参数
  optimizer_params_.initial_position = cfg.initial_position;
  optimizer_params_.initial_direction = cfg.initial_direction;
  optimizer_params_.initial_strength = cfg.initial_strength;
  optimizer_params_.strength_delta = cfg.strength_delta;
  optimizer_params_.optimize_strength = cfg.optimize_strength;
  optimizer_params_.max_iterations = cfg.max_iterations;
  optimizer_params_.function_tolerance = cfg.function_tolerance;
  optimizer_params_.gradient_tolerance = cfg.gradient_tolerance;
  optimizer_params_.parameter_tolerance = cfg.parameter_tolerance;
  optimizer_params_.num_threads = cfg.num_threads;
  optimizer_params_.minimizer_progress = cfg.minimizer_progress;
  optimizer_params_.linear_solver = cfg.linear_solver;

  // 配置预处理器
  preprocessor_.configure(cfg);
}

/**
 * @brief 初始化估计器
 * 
 * 流程：
 * 1. 创建估计器实例（通过工厂方法）
 * 2. 构建配置结构体并设置到估计器
 * 3. 初始化估计器
 * 4. 如果是批量优化器模式，初始化 TF 监听器和测量缓存
 */
void MagPoseEstimatorNode::initializeEstimator() {
  estimator_ = createEstimator(estimator_type_);
  EstimatorConfig cfg = buildConfigFromParameters();
  estimator_->setConfig(cfg);
  estimator_->initialize();
  
  // 检查是否为批量优化器模式
  optimizer_backend_ = dynamic_cast<OptimizerEstimator *>(estimator_.get());
  use_batch_optimizer_ = (optimizer_backend_ != nullptr) && (estimator_type_ == "optimizer");
  
  if (use_batch_optimizer_) {
    // 批量优化器需要 TF 来查询传感器位置
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    measurement_cache_.clear();
  }
}

// ============================================================================
// 配置相关函数
// ============================================================================

/**
 * @brief 从参数构建估计器配置
 * @return 估计器配置结构体
 */
EstimatorConfig MagPoseEstimatorNode::buildConfigFromParameters() const {
  EstimatorConfig cfg;
  cfg.world_field = world_field_vector_;  // 单位：mT
  cfg.process_noise_position = process_noise_position_;
  cfg.process_noise_orientation = process_noise_orientation_;
  cfg.measurement_noise = measurement_noise_;
  cfg.position_gain = position_gain_;
  cfg.optimizer_iterations = optimizer_iterations_;
  cfg.optimizer_damping = optimizer_damping_;
  cfg.optimizer = optimizer_params_;
  return cfg;
}

/**
 * @brief 解析 XML-RPC 配置为配置结构体
 * @param root XML-RPC 根节点
 * @param context 上下文路径（用于错误信息）
 * @return 配置结构体
 * 
 * 从 XML-RPC 配置中解析所有参数，包括：
 * - 坐标系和话题配置
 * - 估计器参数
 * - 预处理器参数（校准矩阵、偏移量、滤波参数）
 * - 优化器参数（初始值、迭代次数、收敛容差等）
 */
MagPoseEstimatorConfig MagPoseEstimatorNode::loadMagPoseEstimatorConfig(
    const XmlRpc::XmlRpcValue &root, const std::string &context) {
  namespace xml = mag_core_utils::xmlrpc;
  const auto &node = xml::asStruct(root, context);

  MagPoseEstimatorConfig cfg;

  // 解析坐标系配置
  const auto frames_ctx = xml::makeContext(context, "frames");
  const auto &frames = xml::requireStructField(node, "frames", context);
  cfg.output_frame = xml::requireStringField(frames, "output_frame", frames_ctx);

  // 解析话题配置
  const auto topics_ctx = xml::makeContext(context, "topics");
  const auto &topics = xml::requireStructField(node, "topics", context);
  cfg.mag_topic = xml::requireStringField(topics, "mag_field", topics_ctx);
  cfg.pose_topic = xml::requireStringField(topics, "pose_estimate", topics_ctx);

  // 解析估计器参数
  const auto estimator_ctx = xml::makeContext(context, "params/estimator");
  const auto &estimator = xml::requireStructField(
      xml::requireStructField(node, "params", context), "estimator", context);
  cfg.estimator_type = xml::requireStringField(estimator, "type", estimator_ctx);
  cfg.min_sensors = xml::readNumber(
      xml::requireMember(estimator, "min_sensors", estimator_ctx),
      estimator_ctx + "/min_sensors");
  cfg.tf_timeout = xml::readNumber(
      xml::requireMember(estimator, "tf_timeout", estimator_ctx),
      estimator_ctx + "/tf_timeout");
  cfg.position_gain = xml::readNumber(
      xml::requireMember(estimator, "position_gain", estimator_ctx),
      estimator_ctx + "/position_gain");
  cfg.process_noise_position = xml::readNumber(
      xml::requireMember(estimator, "process_noise_position", estimator_ctx),
      estimator_ctx + "/process_noise_position");
  cfg.process_noise_orientation = xml::readNumber(
      xml::requireMember(estimator, "process_noise_orientation", estimator_ctx),
      estimator_ctx + "/process_noise_orientation");
  cfg.measurement_noise = xml::readNumber(
      xml::requireMember(estimator, "measurement_noise", estimator_ctx),
      estimator_ctx + "/measurement_noise");
  cfg.optimizer_iterations = xml::readNumber(
      xml::requireMember(estimator, "optimizer_iterations", estimator_ctx),
      estimator_ctx + "/optimizer_iterations");
  cfg.optimizer_damping = xml::readNumber(
      xml::requireMember(estimator, "optimizer_damping", estimator_ctx),
      estimator_ctx + "/optimizer_damping");
  cfg.world_field = parseVector3(
      xml::requireMember(estimator, "world_field", estimator_ctx),
      estimator_ctx + "/world_field");  // 单位：mT

  // 解析预处理器参数
  const auto preprocessor_ctx = xml::makeContext(context, "params/preprocessor");
  const auto &preprocessor = xml::requireStructField(
      xml::requireStructField(node, "params", context), "preprocessor", context);
  cfg.enable_calibration = xml::requireBoolField(
      preprocessor, "enable_calibration", preprocessor_ctx);
  cfg.soft_iron_matrix = parseMatrix3x3(
      xml::requireMember(preprocessor, "soft_iron_matrix", preprocessor_ctx),
      preprocessor_ctx + "/soft_iron_matrix");
  cfg.hard_iron_offset = parseVector3(
      xml::requireMember(preprocessor, "hard_iron_offset", preprocessor_ctx),
      preprocessor_ctx + "/hard_iron_offset");  // 单位：mT
  cfg.enable_filter = xml::requireBoolField(
      preprocessor, "enable_filter", preprocessor_ctx);
  cfg.low_pass_alpha = xml::readNumber(
      xml::requireMember(preprocessor, "low_pass_alpha", preprocessor_ctx),
      preprocessor_ctx + "/low_pass_alpha");

  // 解析优化器参数
  const auto optimizer_ctx = xml::makeContext(context, "params/optimizer");
  const auto &optimizer = xml::requireStructField(
      xml::requireStructField(node, "params", context), "optimizer", context);
  cfg.initial_position = parseVector3(
      xml::requireMember(optimizer, "initial_position", optimizer_ctx),
      optimizer_ctx + "/initial_position");
  cfg.initial_direction = parseVector3(
      xml::requireMember(optimizer, "initial_direction", optimizer_ctx),
      optimizer_ctx + "/initial_direction");
  cfg.initial_strength = xml::readNumber(
      xml::requireMember(optimizer, "initial_strength", optimizer_ctx),
      optimizer_ctx + "/initial_strength");
  cfg.strength_delta = xml::readNumber(
      xml::requireMember(optimizer, "strength_delta", optimizer_ctx),
      optimizer_ctx + "/strength_delta");
  cfg.optimize_strength = xml::requireBoolField(
      optimizer, "optimize_strength", optimizer_ctx);
  cfg.max_iterations = xml::readNumber(
      xml::requireMember(optimizer, "max_iterations", optimizer_ctx),
      optimizer_ctx + "/max_iterations");
  cfg.function_tolerance = xml::readNumber(
      xml::requireMember(optimizer, "function_tolerance", optimizer_ctx),
      optimizer_ctx + "/function_tolerance");
  cfg.gradient_tolerance = xml::readNumber(
      xml::requireMember(optimizer, "gradient_tolerance", optimizer_ctx),
      optimizer_ctx + "/gradient_tolerance");
  cfg.parameter_tolerance = xml::readNumber(
      xml::requireMember(optimizer, "parameter_tolerance", optimizer_ctx),
      optimizer_ctx + "/parameter_tolerance");
  cfg.num_threads = xml::readNumber(
      xml::requireMember(optimizer, "num_threads", optimizer_ctx),
      optimizer_ctx + "/num_threads");
  cfg.minimizer_progress = xml::requireBoolField(
      optimizer, "minimizer_progress", optimizer_ctx);
  cfg.linear_solver = xml::requireStringField(
      optimizer, "linear_solver", optimizer_ctx);

  return cfg;
}

// ============================================================================
// 回调函数和数据发布
// ============================================================================

/**
 * @brief 磁传感器数据回调函数
 * @param msg 磁传感器测量消息（单位：mT，毫特斯拉）
 * 
 * 处理流程：
 * 1. 将 MagSensorData 转换为 sensor_msgs::MagneticField（保持 mT 单位）
 * 2. 通过预处理器处理数据（校准、滤波）
 * 3. 如果是批量优化器模式：
 *    - 缓存测量数据
 *    - 当传感器数量满足要求时，运行批量求解器
 * 4. 否则（EKF 模式）：
 *    - 直接更新估计器
 *    - 发布估计结果
 */
void MagPoseEstimatorNode::magCallback(
    const mag_core_msgs::MagSensorDataConstPtr &msg) {
  // 将自定义消息转换为 sensor_msgs，统一使用 mT 单位
  // 注意：sensor_msgs::MagneticField 的标准单位是 Tesla，但这里我们使用 mT 以保持一致性
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header = msg->header;
  // 直接使用 mT，不进行单位转换
  mag_msg.magnetic_field.x = msg->mag_x;
  mag_msg.magnetic_field.y = msg->mag_y;
  mag_msg.magnetic_field.z = msg->mag_z;

  // 预处理（校准、滤波）
  sensor_msgs::MagneticField processed = preprocessor_.process(mag_msg);
  
  if (use_batch_optimizer_) {
    // 批量优化器模式：缓存数据并运行批量求解器
    cacheMeasurement(msg->sensor_id, processed);
    runBatchSolver();
    return;
  }

  // EKF 模式：直接更新并发布
  estimator_->update(processed);
  geometry_msgs::Pose pose = estimator_->getPose();
  publishPose(pose, msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp);
}

/**
 * @brief 发布姿态估计结果
 * @param pose 估计的姿态（包含位置和方向）
 * @param stamp 时间戳
 */
void MagPoseEstimatorNode::publishPose(const geometry_msgs::Pose &pose,
                                        const ros::Time &stamp) {
  mag_core_msgs::MagnetPose msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = output_frame_;
  msg.position = pose.position;
  msg.orientation = pose.orientation;
  msg.magnetic_strength = estimator_->getMagneticStrength();  // 获取估计的磁矩强度 (Am²)
  pose_pub_.publish(msg);
}

// ============================================================================
// 批量优化器相关函数
// ============================================================================

/**
 * @brief 缓存测量数据（用于批量优化器）
 * @param sensor_id 传感器 ID
 * @param processed 预处理后的磁场数据（单位：mT）
 * 
 * 每个传感器只保留最新一次读数，用于批量优化器同步使用。
 * 当传感器数量满足要求时，批量优化器会使用这些数据同时估计位置和姿态。
 */
void MagPoseEstimatorNode::cacheMeasurement(
    uint32_t sensor_id, const sensor_msgs::MagneticField &processed) {
  CachedMeasurement entry;
  entry.stamp = processed.header.stamp.isZero() ? ros::Time::now() : processed.header.stamp;
  entry.frame_id = processed.header.frame_id;
  // 注意：processed 的单位已经是 mT（与输入数据一致），直接使用即可
  entry.field = Eigen::Vector3d(processed.magnetic_field.x,
                                 processed.magnetic_field.y,
                                 processed.magnetic_field.z);
  measurement_cache_[static_cast<int>(sensor_id)] = entry;
}

/**
 * @brief 构建批量测量数据
 * @param out_batch 输出的批量测量数据
 * @return 是否成功构建（需要满足最小传感器数量要求）
 * 
 * 从缓存中提取所有有效测量数据，并通过 TF 转换到统一坐标系。
 * 如果某个传感器的 TF 查询失败，会将其从缓存中移除。
 */
bool MagPoseEstimatorNode::buildBatch(std::vector<OptimizerMeasurement> &out_batch) {
  if (measurement_cache_.size() < min_sensors_) {
    return false;
  }

  std::vector<int> stale_keys;  // 失效的传感器 ID
  out_batch.clear();
  out_batch.reserve(measurement_cache_.size());

  // 遍历所有缓存的测量数据
  for (const auto &kv : measurement_cache_) {
    OptimizerMeasurement meas;
    // 通过 TF 查询传感器位置并转换磁场向量
    if (fillOptimizerMeasurement(kv.second.stamp, kv.second.frame_id, kv.second.field, meas)) {
      meas.sensor_id = kv.first;
      out_batch.push_back(meas);
    } else {
      // TF 查询失败，标记为失效
      stale_keys.push_back(kv.first);
    }
  }

  // 移除失效的缓存项
  for (int key : stale_keys) {
    measurement_cache_.erase(key);
  }

  return out_batch.size() >= min_sensors_;
}

/**
 * @brief 填充优化器测量数据
 * @param stamp 时间戳
 * @param frame_id 传感器坐标系名称
 * @param field 磁场向量 (mT)
 * @param out_meas 输出的优化器测量数据
 * @return 是否成功填充
 * 
 * 通过 TF 查询传感器位置，并将磁场向量从传感器坐标系转换到输出坐标系。
 * 这是批量优化器所需的关键步骤，确保所有测量数据在统一坐标系下。
 */
bool MagPoseEstimatorNode::fillOptimizerMeasurement(
    const ros::Time &stamp,
    const std::string &frame_id,
    const Eigen::Vector3d &field,
    OptimizerMeasurement &out_meas) const {
  if (frame_id.empty()) {
    return false;
  }

  try {
    // 查询传感器相对于 output_frame_ 的位姿
    geometry_msgs::TransformStamped tf_sensor = tf_buffer_.lookupTransform(
        output_frame_, frame_id, stamp, ros::Duration(tf_timeout_));

    // 将磁场向量从传感器坐标系转换到输出坐标系
    geometry_msgs::Vector3Stamped v_sensor;
    v_sensor.header.frame_id = frame_id;
    v_sensor.header.stamp = stamp;
    v_sensor.vector.x = field.x();
    v_sensor.vector.y = field.y();
    v_sensor.vector.z = field.z();

    geometry_msgs::Vector3Stamped v_world;
    tf2::doTransform(v_sensor, v_world, tf_sensor);

    // 填充输出结构
    out_meas.magnetic_field = Eigen::Vector3d(
        v_world.vector.x, v_world.vector.y, v_world.vector.z);  // 单位：mT
    out_meas.sensor_position = Eigen::Vector3d(
        tf_sensor.transform.translation.x,
        tf_sensor.transform.translation.y,
        tf_sensor.transform.translation.z);  // 单位：米
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "mag_pose_estimator: TF lookup failed for %s -> %s: %s",
                      frame_id.c_str(), output_frame_.c_str(), ex.what());
    return false;
  }
}

/**
 * @brief 运行批量求解器
 * 
 * 当传感器数量满足要求时，调用批量优化器估计姿态并发布结果。
 * 成功后会清空缓存，准备下一批数据。
 * 失败时保留缓存数据，以便下次重试。
 */
void MagPoseEstimatorNode::runBatchSolver() {
  if (!use_batch_optimizer_ || !optimizer_backend_) {
    return;
  }

  std::vector<OptimizerMeasurement> batch;
  if (!buildBatch(batch)) {
    ROS_DEBUG_THROTTLE(2.0,
                      "mag_pose_estimator: insufficient sensors for batch optimization "
                      "(have %zu, need %zu)",
                      measurement_cache_.size(), min_sensors_);
    return;
  }

  // 传感器数量满足要求，调用 Ceres 批量优化器
  geometry_msgs::Pose pose;
  double error = 0.0;
  if (optimizer_backend_->estimateFromBatch(batch, pose, &error)) {
    publishPose(pose, ros::Time::now());
    // 清空缓存，准备下一批数据
    measurement_cache_.clear();
    ROS_DEBUG_THROTTLE(1.0,
                      "mag_pose_estimator: batch optimization succeeded with %zu sensors, "
                      "error=%.6e",
                      batch.size(), error);
  } else {
    ROS_WARN_THROTTLE(1.0,
                     "mag_pose_estimator: optimizer batch solve failed with %zu sensors",
                     batch.size());
    // 优化失败时不清空缓存，保留数据以便下次重试
  }
}

// ============================================================================
// 工具函数
// ============================================================================

/**
 * @brief 解析 3D 向量
 * @param node XML-RPC 节点
 * @param context 上下文路径（用于错误信息）
 * @return 3D 向量
 */
Eigen::Vector3d MagPoseEstimatorNode::parseVector3(
    const XmlRpc::XmlRpcValue &node, const std::string &context) {
  namespace xml = mag_core_utils::xmlrpc;
  const auto vec = xml::readVector3(node, context);
  return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

/**
 * @brief 解析 3×3 矩阵
 * @param node XML-RPC 节点
 * @param context 上下文路径（用于错误信息）
 * @return 3×3 矩阵
 */
Eigen::Matrix3d MagPoseEstimatorNode::parseMatrix3x3(
    const XmlRpc::XmlRpcValue &node, const std::string &context) {
  namespace xml = mag_core_utils::xmlrpc;
  const auto mat = xml::readVector9(node, context);
  Eigen::Matrix3d m;
  m << mat[0], mat[1], mat[2],
       mat[3], mat[4], mat[5],
       mat[6], mat[7], mat[8];
  return m;
}

// ============================================================================
// 工厂方法
// ============================================================================

/**
 * @brief 工厂方法：创建估计器实例
 * @param type 估计器类型（"ekf" 或 "optimizer"）
 * @return 估计器实例指针
 * 
 * 支持的估计器类型：
 * - "ekf": 扩展卡尔曼滤波器
 * - "optimizer": 批量优化器
 * 
 * 如果类型未知，默认使用 EKF。
 */
std::unique_ptr<EstimatorBase> MagPoseEstimatorNode::createEstimator(
    const std::string &type) {
  std::string lower = type;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "ekf") {
    ROS_INFO("mag_pose_estimator: launching EKF estimator");
    return std::make_unique<EKFEstimator>();
  }

  if (lower == "optimizer") {
    ROS_INFO("mag_pose_estimator: launching optimizer estimator");
    return std::make_unique<OptimizerEstimator>();
  }

  // 未知的类型统一回退到 EKF，保证节点仍能以合理默认值启动
  ROS_WARN("mag_pose_estimator: unknown estimator type '%s', defaulting to EKF",
           type.c_str());
  return std::make_unique<EKFEstimator>();
}

// ============================================================================
// 主函数
// ============================================================================

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 退出码
 * 
 * ROS 节点入口点，初始化节点并进入事件循环。
 */
int MagPoseEstimatorNode::main(int argc, char **argv) {
  ros::init(argc, argv, "mag_pose_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mag_pose_estimator::MagPoseEstimatorNode node(nh, pnh);
  ros::spin();
  return 0;
}

}  // namespace mag_pose_estimator
