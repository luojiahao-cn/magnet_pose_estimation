#pragma once

#include <mag_core_msgs/MagSensorData.h>
#include <mag_sensor_calibration/calibration_config_loader.h>
#include <Eigen/Dense>

namespace mag_sensor_calibration {

/**
 * @brief 校正参数应用器
 * 
 * 功能：对传感器数据应用校正参数
 */
class CalibrationApplier {
public:
  CalibrationApplier();
  
  /**
   * @brief 加载校正参数
   * @param params 校正参数
   */
  void loadCalibrationParams(const CalibrationParams &params);
  
  /**
   * @brief 应用校正
   * @param sensor_id 传感器ID
   * @param raw_field 原始磁场向量 [x, y, z] (mT)
   * @return 校正后的磁场向量
   */
  Eigen::Vector3d applyCalibration(uint32_t sensor_id, const Eigen::Vector3d &raw_field);
  
  /**
   * @brief 检查传感器是否有校正参数
   */
  bool hasCalibration(uint32_t sensor_id) const;
  
private:
  CalibrationParams calibration_params_;
};

}  // namespace mag_sensor_calibration

