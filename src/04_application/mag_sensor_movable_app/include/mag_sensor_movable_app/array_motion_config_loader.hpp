/**
 * @file array_motion_config_loader.hpp
 * @brief 传感器阵列运动配置加载器头文件
 * 
 * 提供从 XML-RPC 配置中加载传感器阵列运动配置的功能
 */

#pragma once

#include <mag_sensor_movable_app/array_motion_node.hpp>

#include <XmlRpcValue.h>

#include <string>

namespace mag_sensor_movable_app
{

/**
 * @brief 传感器阵列运动配置包
 * 
 * 包含传感器阵列运动控制所需的所有配置项
 */
struct ArrayMotionConfigBundle
{
    FrameConfig frame;              ///< TF 坐标系配置
    MotionConfig motion;            ///< 运动参数配置
    TrajectoryConfig trajectory;    ///< 轨迹配置
    OrientationConfig orientation;  ///< 姿态配置
};

/**
 * @brief 从 XML-RPC 配置中加载传感器阵列运动配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径（用于错误提示）
 * @return ArrayMotionConfigBundle 完整的配置包
 */
ArrayMotionConfigBundle loadArrayMotionConfig(const XmlRpc::XmlRpcValue &root,
                                               const std::string &context);

} // namespace mag_sensor_movable_app

