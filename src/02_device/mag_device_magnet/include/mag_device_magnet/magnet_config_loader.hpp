/**
 * @file magnet_config_loader.hpp
 * @brief 磁体设备配置加载器头文件
 * 
 * 提供从 XML-RPC 配置中加载磁体设备完整配置的功能
 */

#pragma once

#include <mag_device_magnet/magnet_node.hpp>

#include <XmlRpcValue.h>

#include <string>

namespace mag_device_magnet
{

/**
 * @brief 磁体配置包
 * 
 * 包含磁体设备运行所需的所有配置项
 */
struct MagnetConfigBundle
{
    FrameConfig frame;              ///< TF 坐标系配置
    MotionConfig motion;            ///< 运动参数配置
    TopicConfig topics;             ///< ROS 话题配置
    TfConfig tf;                    ///< TF 发布配置
    TrajectoryConfig trajectory;    ///< 轨迹配置
    OrientationConfig orientation;  ///< 姿态配置
};

/**
 * @brief 从 XML-RPC 配置中加载磁体设备配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径（用于错误提示）
 * @return MagnetConfigBundle 完整的配置包
 */
MagnetConfigBundle loadMagnetConfig(const XmlRpc::XmlRpcValue &root,
                                    const std::string &context);

} // namespace mag_device_magnet
