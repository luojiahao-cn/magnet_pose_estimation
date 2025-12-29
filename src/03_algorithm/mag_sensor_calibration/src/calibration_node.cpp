#include "mag_sensor_calibration/calibration_data_collector.h"
#include "mag_sensor_calibration/ellipsoid_fitter.h"
#include "mag_sensor_calibration/calibration_config_loader.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iomanip>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mag_sensor_calibration_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  // 创建数据采集器
  mag_sensor_calibration::CalibrationDataCollector collector(nh, pnh);
  
  // 加载配置
  if (!collector.loadConfig()) {
    ROS_ERROR("[calibration_node] 配置加载失败");
    return 1;
  }
  
  // 执行数据采集
  ROS_INFO("[calibration_node] 开始数据采集...");
  if (!collector.collectData()) {
    ROS_ERROR("[calibration_node] 数据采集失败");
    return 1;
  }
  
  // 获取采集的数据
  const auto &collected_data = collector.getCollectedData();
  if (collected_data.empty()) {
    ROS_ERROR("[calibration_node] 没有采集到数据");
    return 1;
  }
  
  // 按传感器ID组织数据
  std::map<uint32_t, std::vector<Eigen::Vector3d>> sensor_data;
  for (const auto &pose_data : collected_data) {
    for (const auto &kv : pose_data.sensor_measurements) {
      uint32_t sensor_id = kv.first;
      const auto &measurements = kv.second;
      sensor_data[sensor_id].insert(sensor_data[sensor_id].end(), 
                                     measurements.begin(), measurements.end());
    }
  }
  
  ROS_INFO("[calibration_node] 开始椭圆拟合...");
  
  // 对每个传感器进行椭圆拟合
  mag_sensor_calibration::EllipsoidFitter fitter;
  mag_sensor_calibration::CalibrationParams calibration_params;
  
  for (const auto &kv : sensor_data) {
    uint32_t sensor_id = kv.first;
    const auto &measurements = kv.second;
    
    if (measurements.size() < 9) {
      ROS_WARN("[calibration_node] 传感器 %u 数据点不足 (%zu < 9)，跳过", sensor_id, measurements.size());
      continue;
    }
    
    ROS_INFO("[calibration_node] 拟合传感器 %u，数据点: %zu", sensor_id, measurements.size());
    
    auto result = fitter.fit(measurements);
    
    if (!result.success) {
      ROS_WARN("[calibration_node] 传感器 %u 拟合失败", sensor_id);
      continue;
    }
    
    mag_sensor_calibration::SensorCalibrationParams calib_params;
    calib_params.sensor_id = sensor_id;
    calib_params.hard_iron_offset = result.hard_iron_offset;
    calib_params.soft_iron_matrix = result.soft_iron_matrix;
    
    calibration_params.sensors[sensor_id] = calib_params;
    
    ROS_INFO("[calibration_node] 传感器 %u 拟合成功，硬铁偏移: [%.4f, %.4f, %.4f] mT",
             sensor_id,
             result.hard_iron_offset.x(),
             result.hard_iron_offset.y(),
             result.hard_iron_offset.z());
  }
  
  ROS_INFO("[calibration_node] 椭圆拟合完成，成功拟合 %zu 个传感器", calibration_params.sensors.size());
  
  // 保存校正参数
  mag_sensor_calibration::CalibrationConfigLoader loader;
  
  // 获取输出文件路径
  std::string params_file;
  try {
    XmlRpc::XmlRpcValue root;
    if (pnh.getParam("config", root)) {
      namespace xml = mag_core_utils::xmlrpc;
      const auto &config = xml::requireStructField(root, "config", "config");
      const auto &output = xml::requireStructField(config, "output", "config");
      params_file = xml::requireStringField(output, "params_file", "config.output");
    } else {
      params_file = "calibration_params.yaml";
    }
  } catch (...) {
    params_file = "calibration_params.yaml";
  }
  
  // 解析相对路径
  if (params_file[0] != '/') {
    std::string package_path = ros::package::getPath("mag_sensor_calibration");
    if (!package_path.empty()) {
      params_file = package_path + "/" + params_file;
    }
  }
  
  if (!loader.saveToFile(params_file, calibration_params)) {
    ROS_ERROR("[calibration_node] 保存校正参数失败");
    return 1;
  }
  
  ROS_INFO("[calibration_node] 校正参数已保存到: %s", params_file.c_str());
  ROS_INFO("[calibration_node] 校正流程完成！");
  
  return 0;
}

