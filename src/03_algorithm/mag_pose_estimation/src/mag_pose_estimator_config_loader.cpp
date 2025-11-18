#include <mag_pose_estimator/mag_pose_estimator_config_loader.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdexcept>
#include <string>

namespace mag_pose_estimator
{
namespace
{
namespace xml = mag_core_utils::xmlrpc;

Eigen::Vector3d parseVector3(const XmlRpc::XmlRpcValue &node, const std::string &context)
{
    const auto vec = xml::readVector3(node, context);
    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

Eigen::Matrix3d parseMatrix3x3(const XmlRpc::XmlRpcValue &node, const std::string &context)
{
    const auto mat = xml::readVector9(node, context);
    Eigen::Matrix3d m;
    m << mat[0], mat[1], mat[2],
         mat[3], mat[4], mat[5],
         mat[6], mat[7], mat[8];
    return m;
}

} // namespace

MagPoseEstimatorConfig loadMagPoseEstimatorConfig(const XmlRpc::XmlRpcValue &root,
                                                   const std::string &context)
{
    const auto &node = xml::asStruct(root, context);

    MagPoseEstimatorConfig cfg;

    // Frames
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(node, "frames", context);
    cfg.output_frame = xml::requireStringField(frames, "output_frame", frames_ctx);

    // Topics
    const auto topics_ctx = xml::makeContext(context, "topics");
    const auto &topics = xml::requireStructField(node, "topics", context);
    cfg.mag_topic = xml::requireStringField(topics, "mag_field", topics_ctx);
    cfg.pose_topic = xml::requireStringField(topics, "pose_estimate", topics_ctx);

    // Params
    const auto params_ctx = xml::makeContext(context, "params");
    const auto &params = xml::requireStructField(node, "params", context);

    // Estimator
    const auto estimator_ctx = xml::makeContext(params_ctx, "estimator");
    const auto &estimator = xml::requireStructField(params, "estimator", params_ctx);
    cfg.estimator_type = xml::requireStringField(estimator, "type", estimator_ctx);
    cfg.min_sensors = static_cast<int>(xml::readNumber(xml::requireMember(estimator, "min_sensors", estimator_ctx), estimator_ctx + "/min_sensors"));
    cfg.tf_timeout = xml::readNumber(xml::requireMember(estimator, "tf_timeout", estimator_ctx), estimator_ctx + "/tf_timeout");
    cfg.position_gain = xml::readNumber(xml::requireMember(estimator, "position_gain", estimator_ctx), estimator_ctx + "/position_gain");
    cfg.process_noise_position = xml::readNumber(xml::requireMember(estimator, "process_noise_position", estimator_ctx), estimator_ctx + "/process_noise_position");
    cfg.process_noise_orientation = xml::readNumber(xml::requireMember(estimator, "process_noise_orientation", estimator_ctx), estimator_ctx + "/process_noise_orientation");
    cfg.measurement_noise = xml::readNumber(xml::requireMember(estimator, "measurement_noise", estimator_ctx), estimator_ctx + "/measurement_noise");
    cfg.optimizer_iterations = static_cast<int>(xml::readNumber(xml::requireMember(estimator, "optimizer_iterations", estimator_ctx), estimator_ctx + "/optimizer_iterations"));
    cfg.optimizer_damping = xml::readNumber(xml::requireMember(estimator, "optimizer_damping", estimator_ctx), estimator_ctx + "/optimizer_damping");
    cfg.world_field = parseVector3(xml::requireMember(estimator, "world_field", estimator_ctx), estimator_ctx + "/world_field");

    // Preprocessor
    const auto preprocessor_ctx = xml::makeContext(params_ctx, "preprocessor");
    const auto &preprocessor = xml::requireStructField(params, "preprocessor", params_ctx);
    cfg.enable_calibration = xml::requireBoolField(preprocessor, "enable_calibration", preprocessor_ctx);
    cfg.soft_iron_matrix = parseMatrix3x3(xml::requireMember(preprocessor, "soft_iron_matrix", preprocessor_ctx), preprocessor_ctx + "/soft_iron_matrix");
    cfg.hard_iron_offset = parseVector3(xml::requireMember(preprocessor, "hard_iron_offset", preprocessor_ctx), preprocessor_ctx + "/hard_iron_offset");
    cfg.enable_filter = xml::requireBoolField(preprocessor, "enable_filter", preprocessor_ctx);
    cfg.low_pass_alpha = xml::readNumber(xml::requireMember(preprocessor, "low_pass_alpha", preprocessor_ctx), preprocessor_ctx + "/low_pass_alpha");

    // Optimizer
    const auto optimizer_ctx = xml::makeContext(params_ctx, "optimizer");
    const auto &optimizer = xml::requireStructField(params, "optimizer", params_ctx);
    cfg.optimizer.initial_position = parseVector3(xml::requireMember(optimizer, "initial_position", optimizer_ctx), optimizer_ctx + "/initial_position");
    cfg.optimizer.initial_direction = parseVector3(xml::requireMember(optimizer, "initial_direction", optimizer_ctx), optimizer_ctx + "/initial_direction");
    cfg.optimizer.initial_strength = xml::readNumber(xml::requireMember(optimizer, "initial_strength", optimizer_ctx), optimizer_ctx + "/initial_strength");
    cfg.optimizer.strength_delta = xml::readNumber(xml::requireMember(optimizer, "strength_delta", optimizer_ctx), optimizer_ctx + "/strength_delta");
    cfg.optimizer.optimize_strength = xml::requireBoolField(optimizer, "optimize_strength", optimizer_ctx);
    cfg.optimizer.max_iterations = static_cast<int>(xml::readNumber(xml::requireMember(optimizer, "max_iterations", optimizer_ctx), optimizer_ctx + "/max_iterations"));
    cfg.optimizer.function_tolerance = xml::readNumber(xml::requireMember(optimizer, "function_tolerance", optimizer_ctx), optimizer_ctx + "/function_tolerance");
    cfg.optimizer.gradient_tolerance = xml::readNumber(xml::requireMember(optimizer, "gradient_tolerance", optimizer_ctx), optimizer_ctx + "/gradient_tolerance");
    cfg.optimizer.parameter_tolerance = xml::readNumber(xml::requireMember(optimizer, "parameter_tolerance", optimizer_ctx), optimizer_ctx + "/parameter_tolerance");
    cfg.optimizer.num_threads = static_cast<int>(xml::readNumber(xml::requireMember(optimizer, "num_threads", optimizer_ctx), optimizer_ctx + "/num_threads"));
    cfg.optimizer.minimizer_progress = xml::requireBoolField(optimizer, "minimizer_progress", optimizer_ctx);
    cfg.optimizer.linear_solver = xml::requireStringField(optimizer, "linear_solver", optimizer_ctx);

    return cfg;
}

} // namespace mag_pose_estimator