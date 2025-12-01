#include "mag_tracking_control/tracking_control_config_loader.h"

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace mag_tracking_control {
namespace {
namespace xml = mag_core_utils::xmlrpc;

/**
 * @brief 将字符串转换为小写
 */
std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

}  // namespace

TrackingControlConfig loadTrackingControlConfig(const XmlRpc::XmlRpcValue &root,
                                                const std::string &context) {
    namespace xml = mag_core_utils::xmlrpc;
    
    const auto &node = xml::asStruct(root, context);
    TrackingControlConfig cfg;

    // 解析策略配置
    const auto strategy_ctx = xml::makeContext(context, "strategy");
    const auto &strategy = xml::requireStructField(node, "strategy", context);
    cfg.strategy_type = xml::requireStringField(strategy, "type", strategy_ctx);
    
    std::string lower_type = toLower(cfg.strategy_type);

    // 解析传感器机械臂名称
    cfg.sensor_arm_name = xml::requireStringField(node, "sensor_arm_name", context);

    // 解析话题配置
    const auto topics_ctx = xml::makeContext(context, "topics");
    const auto &topics = xml::requireStructField(node, "topics", context);
    cfg.magnet_pose_topic = xml::requireStringField(topics, "magnet_pose", topics_ctx);
    cfg.target_pose_topic = xml::requireStringField(topics, "target_pose", topics_ctx);
    
    // 解析服务配置
    const auto services_ctx = xml::makeContext(context, "services");
    const auto &services = xml::requireStructField(node, "services", context);
    cfg.arm_service_name = xml::requireStringField(services, "arm_service", services_ctx);
    cfg.cartesian_path_service_name = xml::optionalStringField(
        services, "cartesian_path_service", services_ctx, "/mag_device_arm/execute_cartesian_path");

    // 解析坐标系配置
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(node, "frames", context);
    cfg.sensor_frame = xml::requireStringField(frames, "sensor_frame", frames_ctx);
    cfg.magnet_frame = xml::requireStringField(frames, "magnet_frame", frames_ctx);

    // 解析控制参数
    const auto control_ctx = xml::makeContext(context, "control");
    const auto &control = xml::requireStructField(node, "control", context);
    cfg.update_rate = xml::readNumber(
        xml::requireMember(control, "update_rate", control_ctx),
        control_ctx + "/update_rate");
    cfg.enable_execution = xml::requireBoolField(control, "enable_execution", control_ctx);
    cfg.velocity_scaling = xml::readNumber(
        xml::requireMember(control, "velocity_scaling", control_ctx),
        control_ctx + "/velocity_scaling");
    cfg.acceleration_scaling = xml::readNumber(
        xml::requireMember(control, "acceleration_scaling", control_ctx),
        control_ctx + "/acceleration_scaling");
    
    // 解析连续轨迹参数（可选）
    cfg.use_continuous_trajectory = xml::optionalBoolField(control, "use_continuous_trajectory", control_ctx, false);
    if (cfg.use_continuous_trajectory) {
        cfg.trajectory_buffer_size = static_cast<size_t>(
            xml::optionalNumberField(control, "trajectory_buffer_size", control_ctx, 5.0));
        cfg.cartesian_path_step_size = xml::optionalNumberField(
            control, "cartesian_path_step_size", control_ctx, 0.01);
        cfg.cartesian_path_jump_threshold = xml::optionalNumberField(
            control, "cartesian_path_jump_threshold", control_ctx, 0.0);
    }

    // 根据策略类型解析相应参数
    if (lower_type == "fixed_offset") {
        const auto fixed_offset_ctx = xml::makeContext(strategy_ctx, "fixed_offset");
        const auto &fixed_offset = xml::requireStructField(strategy, "fixed_offset", strategy_ctx);
        
        const auto offset_ctx = xml::makeContext(fixed_offset_ctx, "offset");
        const auto &offset = xml::requireStructField(fixed_offset, "offset", fixed_offset_ctx);
        double offset_x = xml::readNumber(
            xml::requireMember(offset, "x", offset_ctx), offset_ctx + "/x");
        double offset_y = xml::readNumber(
            xml::requireMember(offset, "y", offset_ctx), offset_ctx + "/y");
        double offset_z = xml::readNumber(
            xml::requireMember(offset, "z", offset_ctx), offset_ctx + "/z");
        cfg.fixed_offset = Eigen::Vector3d(offset_x, offset_y, offset_z);
        
        cfg.max_movement_per_step = xml::readNumber(
            xml::requireMember(fixed_offset, "max_movement_per_step", fixed_offset_ctx),
            fixed_offset_ctx + "/max_movement_per_step");
            
    } else if (lower_type == "adaptive_distance") {
        const auto adaptive_ctx = xml::makeContext(strategy_ctx, "adaptive_distance");
        const auto &adaptive = xml::requireStructField(strategy, "adaptive_distance", strategy_ctx);
        
        cfg.target_field_strength = xml::readNumber(
            xml::requireMember(adaptive, "target_field_strength", adaptive_ctx),
            adaptive_ctx + "/target_field_strength");
        cfg.min_field_strength = xml::readNumber(
            xml::requireMember(adaptive, "min_field_strength", adaptive_ctx),
            adaptive_ctx + "/min_field_strength");
        cfg.max_field_strength = xml::readNumber(
            xml::requireMember(adaptive, "max_field_strength", adaptive_ctx),
            adaptive_ctx + "/max_field_strength");
        cfg.adjustment_gain = xml::readNumber(
            xml::requireMember(adaptive, "adjustment_gain", adaptive_ctx),
            adaptive_ctx + "/adjustment_gain");
        cfg.max_movement_per_step = xml::readNumber(
            xml::requireMember(adaptive, "max_movement_per_step", adaptive_ctx),
            adaptive_ctx + "/max_movement_per_step");
            
    } else {
        throw std::runtime_error("未知的策略类型: " + cfg.strategy_type + "，支持的类型: fixed_offset, adaptive_distance");
    }

    return cfg;
}

}  // namespace mag_tracking_control

