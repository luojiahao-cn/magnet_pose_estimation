#include <mag_sensor_calibration/calibration_viz_node.hpp>

#include <cmath>

namespace mag_sensor_calibration
{

CalibrationVizNode::CalibrationVizNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , next_marker_id_(0)
{
    pnh_.param("marker_topic", marker_topic_, std::string("/calibration_markers"));
    setupPublishers();
}

CalibrationVizNode::~CalibrationVizNode()
{
}

void CalibrationVizNode::setupPublishers()
{
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    ROS_INFO_STREAM("[calibration_viz] 发布可视化标记到: " << marker_topic_);
}

void CalibrationVizNode::start()
{
    ros::Rate rate(10);  // 10 Hz
    while (ros::ok())
    {
        publishMarkers();
        rate.sleep();
    }
}

void CalibrationVizNode::updateDataPoints(uint32_t sensor_id,
                                          const std::vector<MeasurementPoint>& measurements)
{
    std::vector<visualization_msgs::Marker> markers;
    
    for (size_t i = 0; i < measurements.size(); ++i)
    {
        visualization_msgs::Marker marker = createDataPointMarker(
            sensor_id, measurements[i], next_marker_id_++);
        markers.push_back(marker);
    }
    
    sensor_markers_[sensor_id] = markers;
}

void CalibrationVizNode::updateCalibrationResult(uint32_t sensor_id,
                                                const CalibrationParams& params,
                                                const std::vector<MeasurementPoint>& measurements)
{
    std::vector<visualization_msgs::Marker> markers;
    
    // 添加原始数据点（红色）
    for (const auto& m : measurements)
    {
        visualization_msgs::Marker marker = createDataPointMarker(
            sensor_id, m, next_marker_id_++);
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.push_back(marker);
    }
    
    // 添加校正后的向量（绿色）
    for (const auto& m : measurements)
    {
        Eigen::Vector3d corrected = CalibrationAlgorithm::applyCalibration(m.mag, params);
        visualization_msgs::Marker marker = createCalibrationVectorMarker(
            sensor_id, m, corrected, next_marker_id_++);
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markers.push_back(marker);
    }
    
    sensor_markers_[sensor_id] = markers;
}

void CalibrationVizNode::clearVisualization()
{
    sensor_markers_.clear();
    next_marker_id_ = 0;
}

void CalibrationVizNode::publishMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    
    // 添加所有传感器的标记
    for (const auto& kv : sensor_markers_)
    {
        marker_array.markers.insert(
            marker_array.markers.end(),
            kv.second.begin(),
            kv.second.end());
    }
    
    // 如果标记数组为空，发送删除所有标记的消息
    if (marker_array.markers.empty())
    {
        visualization_msgs::Marker delete_all;
        delete_all.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);
    }
    
    marker_pub_.publish(marker_array);
}

visualization_msgs::Marker CalibrationVizNode::createDataPointMarker(uint32_t sensor_id,
                                                                     const MeasurementPoint& point,
                                                                     int marker_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 可以根据需要修改
    marker.header.stamp = ros::Time::now();
    marker.ns = "calibration_data";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 位置（使用传感器位置或磁场向量）
    marker.pose.position.x = point.mag.x() * 0.01;  // 缩放显示
    marker.pose.position.y = point.mag.y() * 0.01;
    marker.pose.position.z = point.mag.z() * 0.01;
    marker.pose.orientation.w = 1.0;
    
    // 大小
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    
    // 颜色（默认蓝色）
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration(0);  // 永久显示
    
    return marker;
}

visualization_msgs::Marker CalibrationVizNode::createCalibrationVectorMarker(uint32_t sensor_id,
                                                                              const MeasurementPoint& raw,
                                                                              const Eigen::Vector3d& corrected,
                                                                              int marker_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "calibration_vectors";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 起点（原始测量值）
    geometry_msgs::Point start;
    start.x = raw.mag.x() * 0.01;
    start.y = raw.mag.y() * 0.01;
    start.z = raw.mag.z() * 0.01;
    
    // 终点（校正后的值）
    geometry_msgs::Point end;
    end.x = corrected.x() * 0.01;
    end.y = corrected.y() * 0.01;
    end.z = corrected.z() * 0.01;
    
    marker.points.push_back(start);
    marker.points.push_back(end);
    
    // 大小
    marker.scale.x = 0.003;  // 箭头轴直径
    marker.scale.y = 0.005;  // 箭头头直径
    marker.scale.z = 0.01;   // 箭头头长度
    
    // 颜色（默认绿色）
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration(0);
    
    return marker;
}

} // namespace mag_sensor_calibration

