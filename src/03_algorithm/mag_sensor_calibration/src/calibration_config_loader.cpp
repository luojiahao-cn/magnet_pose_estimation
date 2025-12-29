#include "mag_sensor_calibration/calibration_config_loader.h"

#include <fstream>
#include <iomanip>
#include <ros/package.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace mag_sensor_calibration {

bool CalibrationConfigLoader::loadFromFile(const std::string &file_path, CalibrationParams &params) {
  // 使用ROS参数服务器加载YAML文件
  std::string command = "rosparam load " + file_path + " /calibration_temp_load";
  int ret = system(command.c_str());
  if (ret != 0) {
    ROS_ERROR("[calibration_config_loader] 无法加载配置文件: %s", file_path.c_str());
    return false;
  }
  
  ros::NodeHandle temp_nh("/calibration_temp_load");
  bool success = loadFromParam(temp_nh, "calibration", params);
  
  // 清理临时参数
  system("rosparam delete /calibration_temp_load");
  
  return success;
}

bool CalibrationConfigLoader::saveToFile(const std::string &file_path, const CalibrationParams &params) {
  try {
    std::ofstream file(file_path);
    if (!file.is_open()) {
      ROS_ERROR("[calibration_config_loader] 无法打开文件进行写入: %s", file_path.c_str());
      return false;
    }
    
    // 手动写入YAML格式
    file << "calibration:\n";
    file << "  sensors:\n";
    
    for (const auto &kv : params.sensors) {
      const auto &calib = kv.second;
      file << "    - id: " << calib.sensor_id << "\n";
      
      // 保存硬铁偏移
      file << "      hard_iron_offset: ["
           << std::fixed << std::setprecision(6)
           << calib.hard_iron_offset.x() << ", "
           << calib.hard_iron_offset.y() << ", "
           << calib.hard_iron_offset.z() << "]\n";
      
      // 保存软铁矩阵（按行存储）
      file << "      soft_iron_matrix: [";
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          file << std::fixed << std::setprecision(6) << calib.soft_iron_matrix(i, j);
          if (i < 2 || j < 2) {
            file << ", ";
          }
        }
      }
      file << "]\n";
    }
    
    file.close();
    
    ROS_INFO("[calibration_config_loader] 成功保存校正参数到: %s", file_path.c_str());
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[calibration_config_loader] 保存配置文件失败: " << e.what());
    return false;
  }
}

bool CalibrationConfigLoader::loadFromParam(ros::NodeHandle &nh, const std::string &param_name, CalibrationParams &params) {
  XmlRpc::XmlRpcValue root;
  if (!nh.getParam(param_name, root)) {
    ROS_ERROR("[calibration_config_loader] 无法从参数服务器加载参数: %s", param_name.c_str());
    return false;
  }
  
  try {
    namespace xml = mag_core_utils::xmlrpc;
    const auto &calibration = xml::requireStructField(root, "calibration", param_name);
    const auto &sensors = xml::requireArrayField(calibration, "sensors", xml::makeContext(param_name, "calibration"));
    
    params.sensors.clear();
    
    for (int i = 0; i < sensors.size(); ++i) {
      std::string sensor_ctx = xml::makeContext(param_name, "calibration.sensors[" + std::to_string(i) + "]");
      const auto &sensor = xml::requireStructField(sensors[i], "", sensor_ctx);
      
      const auto &id_member = xml::requireMember(sensor, "id", sensor_ctx);
      uint32_t sensor_id = static_cast<uint32_t>(xml::readNumber(id_member, xml::makeContext(sensor_ctx, "id")));
      
      SensorCalibrationParams calib_params;
      calib_params.sensor_id = sensor_id;
      
      // 加载硬铁偏移
      const auto &offset = xml::requireArrayField(sensor, "hard_iron_offset", sensor_ctx);
      if (offset.size() == 3) {
        std::string offset_ctx = xml::makeContext(sensor_ctx, "hard_iron_offset");
        calib_params.hard_iron_offset <<
          xml::readNumber(offset[0], xml::makeContext(offset_ctx, "[0]")),
          xml::readNumber(offset[1], xml::makeContext(offset_ctx, "[1]")),
          xml::readNumber(offset[2], xml::makeContext(offset_ctx, "[2]"));
      } else {
        calib_params.hard_iron_offset.setZero();
      }
      
      // 加载软铁矩阵
      const auto &matrix = xml::requireArrayField(sensor, "soft_iron_matrix", sensor_ctx);
      if (matrix.size() == 9) {
        std::string matrix_ctx = xml::makeContext(sensor_ctx, "soft_iron_matrix");
        calib_params.soft_iron_matrix <<
          xml::readNumber(matrix[0], xml::makeContext(matrix_ctx, "[0]")),
          xml::readNumber(matrix[1], xml::makeContext(matrix_ctx, "[1]")),
          xml::readNumber(matrix[2], xml::makeContext(matrix_ctx, "[2]")),
          xml::readNumber(matrix[3], xml::makeContext(matrix_ctx, "[3]")),
          xml::readNumber(matrix[4], xml::makeContext(matrix_ctx, "[4]")),
          xml::readNumber(matrix[5], xml::makeContext(matrix_ctx, "[5]")),
          xml::readNumber(matrix[6], xml::makeContext(matrix_ctx, "[6]")),
          xml::readNumber(matrix[7], xml::makeContext(matrix_ctx, "[7]")),
          xml::readNumber(matrix[8], xml::makeContext(matrix_ctx, "[8]"));
      } else {
        calib_params.soft_iron_matrix.setIdentity();
      }
      
      params.sensors[sensor_id] = calib_params;
    }
    
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[calibration_config_loader] 从参数服务器加载失败: " << e.what());
    return false;
  }
}

}  // namespace mag_sensor_calibration

