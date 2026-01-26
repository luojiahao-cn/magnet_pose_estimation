/**
 * @file calibration_applier.cpp
 * @brief 实现磁力计校正参数的应用功能，将原始观测值转换为校正后的磁场强度
 */

#include "mag_sensor_calibration/calibration_applier.h"
#include "mag_sensor_calibration/ellipsoid_fitter.h"

namespace mag_sensor_calibration {

CalibrationApplier::CalibrationApplier() {
}

void CalibrationApplier::loadCalibrationParams(const CalibrationParams &params) {
  calibration_params_ = params;
}

Eigen::Vector3d CalibrationApplier::applyCalibration(uint32_t sensor_id, const Eigen::Vector3d &raw_field) {
  auto it = calibration_params_.sensors.find(sensor_id);
  if (it == calibration_params_.sensors.end()) {
    // 没有校正参数，返回原始值
    return raw_field;
  }

  const auto &calib = it->second;
  return EllipsoidFitter::applyCalibration(raw_field, calib.hard_iron_offset, calib.soft_iron_matrix);
}

bool CalibrationApplier::hasCalibration(uint32_t sensor_id) const {
  return calibration_params_.sensors.find(sensor_id) != calibration_params_.sensors.end();
}

}  // namespace mag_sensor_calibration

