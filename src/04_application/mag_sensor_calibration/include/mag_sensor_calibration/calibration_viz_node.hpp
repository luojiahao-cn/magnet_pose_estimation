#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <mag_sensor_calibration/calibration_algorithm.hpp>

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>

namespace mag_sensor_calibration
{

/**
 * @brief 校正可视化节点
 * 
 * 功能：
 * 1. 可视化采集的数据点（在RViz中显示）
 * 2. 可视化校正前后的磁场向量
 * 3. 显示校正进度和状态
 */
class CalibrationVizNode
{
public:
    CalibrationVizNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~CalibrationVizNode();

    void start();

    /**
     * @brief 更新数据点可视化
     * @param sensor_id 传感器ID
     * @param measurements 测量数据点
     */
    void updateDataPoints(uint32_t sensor_id,
                         const std::vector<MeasurementPoint>& measurements);

    /**
     * @brief 更新校正结果可视化
     * @param sensor_id 传感器ID
     * @param params 校正参数
     * @param measurements 原始测量数据
     */
    void updateCalibrationResult(uint32_t sensor_id,
                                const CalibrationParams& params,
                                const std::vector<MeasurementPoint>& measurements);

    /**
     * @brief 清除所有可视化
     */
    void clearVisualization();

    /**
     * @brief 发布可视化标记
     */
    void publishMarkers();

private:
    void setupPublishers();
    
    // 创建数据点标记
    visualization_msgs::Marker createDataPointMarker(uint32_t sensor_id,
                                                     const MeasurementPoint& point,
                                                     int marker_id);
    
    // 创建校正前后对比标记
    visualization_msgs::Marker createCalibrationVectorMarker(uint32_t sensor_id,
                                                             const MeasurementPoint& raw,
                                                             const Eigen::Vector3d& corrected,
                                                             int marker_id);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher marker_pub_;
    std::string marker_topic_;

    // 标记管理
    std::map<uint32_t, std::vector<visualization_msgs::Marker>> sensor_markers_;
    int next_marker_id_;
};

} // namespace mag_sensor_calibration

