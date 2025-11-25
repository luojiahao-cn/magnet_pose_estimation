#pragma once

#include "mag_tracking_control/tracking_control_node.h"

#include <XmlRpcValue.h>
#include <string>

namespace mag_tracking_control {

/**
 * @brief 从 XML-RPC 配置中加载跟踪控制配置
 * 
 * @param root XML-RPC 配置根节点（应该是 "config" 节点）
 * @param context 配置上下文路径（用于错误提示），例如 "tracking_control_node.config"
 * @return TrackingControlConfig 配置结构体
 * @throws std::runtime_error 如果配置缺失必需字段或格式错误
 */
TrackingControlConfig loadTrackingControlConfig(const XmlRpc::XmlRpcValue &root,
                                                const std::string &context);

}  // namespace mag_tracking_control

