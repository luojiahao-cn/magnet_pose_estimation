#pragma once

#include <mag_core_utils/xmlrpc_utils.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>

namespace mag_sensor_calibration {

/**
 * @brief 单个传感器的校正参数
 */
struct SensorCalibrationParams {
  uint32_t sensor_id;
  Eigen::Vector3d hard_iron_offset;  ///< 硬铁偏移 (mT)
  Eigen::Matrix3d soft_iron_matrix;  ///< 软铁矩阵 (3x3)
};

/**
 * @brief 校正参数配置
 */
struct CalibrationParams {
  std::map<uint32_t, SensorCalibrationParams> sensors;  ///< 传感器ID -> 校正参数
};

/**
 * @brief 校正参数配置加载器
 */
class CalibrationConfigLoader {
public:
  /**
   * @brief 从YAML文件加载校正参数
   * @param file_path 配置文件路径
   * @return 是否成功
   */
  bool loadFromFile(const std::string &file_path, CalibrationParams &params);
  
  /**
   * @brief 保存校正参数到YAML文件
   * @param file_path 配置文件路径
   * @param params 校正参数
   * @return 是否成功
   */
  bool saveToFile(const std::string &file_path, const CalibrationParams &params);
  
  /**
   * @brief 从ROS参数服务器加载校正参数
   * @param nh 节点句柄
   * @param param_name 参数名称
   * @return 是否成功
   */
  bool loadFromParam(ros::NodeHandle &nh, const std::string &param_name, CalibrationParams &params);
};

}  // namespace mag_sensor_calibration

