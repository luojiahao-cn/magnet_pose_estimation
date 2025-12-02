/**
 * @file mag_pose_estimator_node.cpp
 * @brief 磁铁姿态估计节点实现
 */

// 项目头文件
#include "mag_pose_estimator/mag_pose_estimator_node.h"
#include "mag_pose_estimator/mag_preprocessor.h"

// 标准库
#include <algorithm>
#include <locale>
#include <vector>

// ROS 相关
#include <ros/console.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// 工具库
#include <mag_core_utils/xmlrpc_utils.hpp>
#include <mag_core_utils/logger_utils.hpp>

// XML-RPC
#include <XmlRpcValue.h>

namespace mag_pose_estimator {

/**
 * @brief 构造函数
 * @param nh 全局节点句柄
 * @param pnh 私有节点句柄
 */
MagPoseEstimatorNode::MagPoseEstimatorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  loadParameters();
  initializeEstimator();

  if (!batch_topic_.empty()) {
    batch_sub_ = nh_.subscribe(batch_topic_, 10, &MagPoseEstimatorNode::batchCallback, this);
  } else {
    mag_sub_ = nh_.subscribe(mag_topic_, 25, &MagPoseEstimatorNode::magCallback, this);
  }
  pose_pub_ = nh_.advertise<mag_core_msgs::MagnetPose>(pose_topic_, 10);
}

/**
 * @brief 从参数服务器加载配置
 */
void MagPoseEstimatorNode::loadParameters() {
  const std::string ns = "mag_pose_estimator";

  XmlRpc::XmlRpcValue config;
  if (!pnh_.getParam("config", config)) {
    ROS_ERROR("[mag_pose_estimator] 无法获取配置参数");
    return;
  }

  auto cfg = loadMagPoseEstimatorConfig(config, ns + "/config");

  // 设置成员变量
  estimator_type_ = cfg.estimator_type;
  mag_topic_ = cfg.mag_topic;
  batch_topic_ = cfg.batch_topic;
  pose_topic_ = cfg.pose_topic;
  output_frame_ = cfg.output_frame;
  magnet_frame_ = cfg.magnet_frame;
  enable_tf_ = cfg.enable_tf;
  tf_timeout_ = cfg.tf_timeout;
  
  // 使用统一的日志格式化工具
  namespace logger = mag_core_utils::logger;
  std::vector<std::pair<std::string, std::string>> config_items;
  config_items.emplace_back("估计器类型", estimator_type_);
  config_items.emplace_back("输出坐标系", cfg.output_frame);
  config_items.emplace_back("磁铁坐标系", magnet_frame_);
  config_items.emplace_back("TF发布", logger::boolToString(enable_tf_));
  config_items.emplace_back("数据话题", !batch_topic_.empty() ? batch_topic_ : mag_topic_);
  config_items.emplace_back("结果话题", pose_topic_);
  
  ROS_INFO("[mag_pose_estimator] %s", logger::formatConfig(config_items).c_str());

  // EKF 参数
  ekf_params_.world_field = cfg.world_field;
  ekf_params_.process_noise_position = cfg.process_noise_position;
  ekf_params_.process_noise_orientation = cfg.process_noise_orientation;
  ekf_params_.measurement_noise = cfg.measurement_noise;
  ekf_params_.position_gain = cfg.position_gain;

  // 优化器参数
  optimizer_params_.min_sensors = cfg.min_sensors;
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
  optimizer_params_.max_acceptable_residual = cfg.max_acceptable_residual;

  // 配置预处理器
  preprocessor_.configure(cfg);
}

/**
 * @brief 初始化估计器
 */
void MagPoseEstimatorNode::initializeEstimator() {
  estimator_ = createEstimator(estimator_type_);
  EstimatorConfig cfg = buildConfigFromParameters();
  estimator_->setConfig(cfg);
  estimator_->initialize();
  
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  
  // 使用统一的日志格式化工具
  namespace logger = mag_core_utils::logger;
  std::vector<std::pair<std::string, std::string>> init_items;
  init_items.emplace_back("估计器类型", estimator_type_);
  ROS_INFO("[mag_pose_estimator] %s", logger::formatInit(init_items).c_str());
}

/**
 * @brief 从参数构建估计器配置
 * @return 估计器配置结构体
 */
EstimatorConfig MagPoseEstimatorNode::buildConfigFromParameters() const {
  EstimatorConfig cfg;
  cfg.ekf = ekf_params_;
  cfg.optimizer = optimizer_params_;
  return cfg;
}

/**
 * @brief 解析 XML-RPC 配置为配置结构体
 * @param root XML-RPC 根节点
 * @param context 上下文路径
 * @return 配置结构体
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
  cfg.magnet_frame = xml::optionalStringField(frames, "magnet_frame", frames_ctx, "magnet_estimated");
  cfg.enable_tf = xml::optionalBoolField(frames, "enable_tf", frames_ctx, true);

  // 解析话题配置
  const auto topics_ctx = xml::makeContext(context, "topics");
  const auto &topics = xml::requireStructField(node, "topics", context);
  cfg.mag_topic = xml::requireStringField(topics, "mag_field", topics_ctx);
  cfg.pose_topic = xml::requireStringField(topics, "pose_estimate", topics_ctx);
  cfg.batch_topic = xml::optionalStringField(topics, "mag_batch", topics_ctx, "");

  const auto estimator_ctx = xml::makeContext(context, "params/estimator");
  const auto &estimator = xml::requireStructField(
      xml::requireStructField(node, "params", context), "estimator", context);
  cfg.estimator_type = xml::requireStringField(estimator, "type", estimator_ctx);
  cfg.tf_timeout = xml::readNumber(
      xml::requireMember(estimator, "tf_timeout", estimator_ctx),
      estimator_ctx + "/tf_timeout");

  std::string lower_type = cfg.estimator_type;
  std::transform(lower_type.begin(), lower_type.end(), lower_type.begin(), ::tolower);

  if (lower_type == "ekf") {
    const auto ekf_ctx = xml::makeContext(context, "params/ekf");
    const auto &ekf = xml::requireStructField(
        xml::requireStructField(node, "params", context), "ekf", context);
    cfg.position_gain = xml::readNumber(
        xml::requireMember(ekf, "position_gain", ekf_ctx),
        ekf_ctx + "/position_gain");
    cfg.process_noise_position = xml::readNumber(
        xml::requireMember(ekf, "process_noise_position", ekf_ctx),
        ekf_ctx + "/process_noise_position");
    cfg.process_noise_orientation = xml::readNumber(
        xml::requireMember(ekf, "process_noise_orientation", ekf_ctx),
        ekf_ctx + "/process_noise_orientation");
    cfg.measurement_noise = xml::readNumber(
        xml::requireMember(ekf, "measurement_noise", ekf_ctx),
        ekf_ctx + "/measurement_noise");
    cfg.world_field = xml::readEigenVector3(
        xml::requireMember(ekf, "world_field", ekf_ctx),
        ekf_ctx + "/world_field");
  } else if (lower_type == "optimizer") {
    const auto optimizer_ctx = xml::makeContext(context, "params/optimizer");
    const auto &optimizer = xml::requireStructField(
        xml::requireStructField(node, "params", context), "optimizer", context);
    cfg.min_sensors = static_cast<int>(xml::readNumber(
        xml::requireMember(optimizer, "min_sensors", optimizer_ctx),
        optimizer_ctx + "/min_sensors"));
    cfg.initial_position = xml::readEigenVector3(
        xml::requireMember(optimizer, "initial_position", optimizer_ctx),
        optimizer_ctx + "/initial_position");
    cfg.initial_direction = xml::readEigenVector3(
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
    cfg.max_iterations = static_cast<int>(xml::readNumber(
        xml::requireMember(optimizer, "max_iterations", optimizer_ctx),
        optimizer_ctx + "/max_iterations"));
    cfg.function_tolerance = xml::readNumber(
        xml::requireMember(optimizer, "function_tolerance", optimizer_ctx),
        optimizer_ctx + "/function_tolerance");
    cfg.gradient_tolerance = xml::readNumber(
        xml::requireMember(optimizer, "gradient_tolerance", optimizer_ctx),
        optimizer_ctx + "/gradient_tolerance");
    cfg.parameter_tolerance = xml::readNumber(
        xml::requireMember(optimizer, "parameter_tolerance", optimizer_ctx),
        optimizer_ctx + "/parameter_tolerance");
    cfg.num_threads = static_cast<int>(xml::readNumber(
        xml::requireMember(optimizer, "num_threads", optimizer_ctx),
        optimizer_ctx + "/num_threads"));
    cfg.minimizer_progress = xml::requireBoolField(
        optimizer, "minimizer_progress", optimizer_ctx);
    cfg.linear_solver = xml::requireStringField(
        optimizer, "linear_solver", optimizer_ctx);
    cfg.max_acceptable_residual = xml::optionalNumberField(
        optimizer, "max_acceptable_residual", optimizer_ctx, 1.0);  // 默认值 1.0 mT
  } else {
    throw std::runtime_error("未知的估计器类型: " + cfg.estimator_type + "，支持的类型: ekf, optimizer");
  }

  // 解析预处理器参数
  const auto preprocessor_ctx = xml::makeContext(context, "params/preprocessor");
  const auto &preprocessor = xml::requireStructField(
      xml::requireStructField(node, "params", context), "preprocessor", context);
  cfg.enable_calibration = xml::requireBoolField(
      preprocessor, "enable_calibration", preprocessor_ctx);
  cfg.soft_iron_matrix = xml::readEigenMatrix3x3(
      xml::requireMember(preprocessor, "soft_iron_matrix", preprocessor_ctx),
      preprocessor_ctx + "/soft_iron_matrix");
  cfg.hard_iron_offset = xml::readEigenVector3(
      xml::requireMember(preprocessor, "hard_iron_offset", preprocessor_ctx),
      preprocessor_ctx + "/hard_iron_offset");
  cfg.enable_filter = xml::requireBoolField(
      preprocessor, "enable_filter", preprocessor_ctx);
  cfg.low_pass_alpha = xml::readNumber(
      xml::requireMember(preprocessor, "low_pass_alpha", preprocessor_ctx),
      preprocessor_ctx + "/low_pass_alpha");


  return cfg;
}

/**
 * @brief 批量传感器数据回调函数
 * @param msg 批量传感器测量消息
 */
void MagPoseEstimatorNode::batchCallback(
    const mag_core_msgs::MagSensorBatchConstPtr &msg) {
  if (msg->measurements.empty()) {
    ROS_DEBUG_THROTTLE(2.0, "[mag_pose_estimator] 批量数据为空");
    return;
  }

  std::vector<sensor_msgs::MagneticField> processed_measurements;
  processed_measurements.reserve(msg->measurements.size());
  for (const auto &sensor_data : msg->measurements) {
    processed_measurements.push_back(convertAndProcess(sensor_data));
  }

  geometry_msgs::Pose pose;
  double error = 0.0;
  if (processMeasurements(processed_measurements, pose, &error)) {
    publishPose(pose, msg->header.stamp);
    double avg_residual = std::sqrt(error / (processed_measurements.size() * 3));
    ROS_INFO_THROTTLE(1.0, "[mag_pose_estimator] 姿态估计成功，总误差: %.9f，平均残差: %.9f mT，传感器数: %zu",
                      error, avg_residual, processed_measurements.size());
  } else {
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 姿态估计失败，传感器数量: %zu", processed_measurements.size());
  }
}

/**
 * @brief 单个磁传感器数据回调函数
 * @param msg 磁传感器测量消息
 */
void MagPoseEstimatorNode::magCallback(
    const mag_core_msgs::MagSensorDataConstPtr &msg) {
  std::vector<sensor_msgs::MagneticField> single_measurement = {convertAndProcess(*msg)};
  geometry_msgs::Pose pose;
  if (processMeasurements(single_measurement, pose)) {
    ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    publishPose(pose, stamp);
  }
}

/**
 * @brief 转换并处理传感器数据
 */
sensor_msgs::MagneticField MagPoseEstimatorNode::convertAndProcess(
    const mag_core_msgs::MagSensorData &sensor_data) {
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header = sensor_data.header;
  mag_msg.magnetic_field.x = sensor_data.mag_x;
  mag_msg.magnetic_field.y = sensor_data.mag_y;
  mag_msg.magnetic_field.z = sensor_data.mag_z;
  return preprocessor_.process(mag_msg);
}

/**
 * @brief 处理测量数据并估计姿态
 */
bool MagPoseEstimatorNode::processMeasurements(
    const std::vector<sensor_msgs::MagneticField> &measurements,
    geometry_msgs::Pose &pose_out,
    double *error_out) {
  auto tf_query = [this](const std::string &frame_id, const ros::Time &stamp,
                         Eigen::Vector3d &position,
                         geometry_msgs::TransformStamped &transform) -> bool {
    return querySensorTransform(frame_id, stamp, position, transform);
  };
  return estimator_->processBatch(measurements, tf_query, output_frame_, pose_out, error_out);
}

/**
 * @brief 发布姿态估计结果
 * @param pose 估计的姿态
 * @param stamp 时间戳
 */
void MagPoseEstimatorNode::publishPose(const geometry_msgs::Pose &pose,
                                        const ros::Time &stamp) {
  mag_core_msgs::MagnetPose msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = output_frame_;
  msg.position = pose.position;
  msg.orientation = pose.orientation;
  msg.magnetic_strength = estimator_->getMagneticStrength();
  pose_pub_.publish(msg);

  if (enable_tf_) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = output_frame_;
    transform.child_frame_id = magnet_frame_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }
}

/**
 * @brief 查询传感器位置和变换
 * @param frame_id 传感器坐标系名称
 * @param stamp 时间戳
 * @param position 输出的传感器位置
 * @param transform 输出的完整 TF 变换
 * @return 是否成功查询
 */
bool MagPoseEstimatorNode::querySensorTransform(const std::string &frame_id,
                                                 const ros::Time &stamp,
                                                 Eigen::Vector3d &position,
                                                 geometry_msgs::TransformStamped &transform) const {
  if (frame_id.empty()) {
    return false;
  }

  try {
    ros::Time query_time = ros::Time(0);
    if (!stamp.isZero()) {
      double age = (ros::Time::now() - stamp).toSec();
      if (age <= 0.5 && age >= -0.1) {
        query_time = stamp;
      }
    }
    
    try {
      transform = tf_buffer_.lookupTransform(
          output_frame_, frame_id, query_time, ros::Duration(tf_timeout_));
    } catch (const tf2::ExtrapolationException &) {
      if (query_time != ros::Time(0)) {
        transform = tf_buffer_.lookupTransform(
            output_frame_, frame_id, ros::Time(0), ros::Duration(tf_timeout_));
      } else {
        throw;
      }
    }

    position = Eigen::Vector3d(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
    return true;
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] TF 查询失败 (%s -> %s): %s",
                      frame_id.c_str(), output_frame_.c_str(), ex.what());
    return false;
  }
}


/**
 * @brief 工厂方法：创建估计器实例
 * @param type 估计器类型（"ekf" 或 "optimizer"）
 * @return 估计器实例指针
 */
std::unique_ptr<EstimatorBase> MagPoseEstimatorNode::createEstimator(
    const std::string &type) {
  std::string lower = type;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "ekf") {
    return std::make_unique<EKFEstimator>();
  }
  if (lower == "optimizer") {
    return std::make_unique<OptimizerEstimator>();
  }

  ROS_WARN("[mag_pose_estimator] 未知估计器类型 '%s'，默认使用 EKF", type.c_str());
  return std::make_unique<EKFEstimator>();
}

}

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 退出码
 */
int main(int argc, char **argv) {
  setlocale(LC_ALL, "zh_CN.UTF-8");
  ros::init(argc, argv, "mag_pose_estimator_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mag_pose_estimator::MagPoseEstimatorNode node(nh, pnh);
  ros::spin();
  return 0;
}
