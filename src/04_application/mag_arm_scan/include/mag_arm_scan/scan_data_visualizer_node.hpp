#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

namespace mag_arm_scan {

struct ScanDataPoint {
    double timestamp;
    double mag_x, mag_y, mag_z;
    double pos_x, pos_y, pos_z;
    double ori_x, ori_y, ori_z, ori_w;
    uint32_t sensor_id;
    std::string frame_id;
};

class ScanDataVisualizerNode {
public:
    ScanDataVisualizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void run();

private:
    void loadParams();
    bool loadCSVData(const std::string &file_path);
    void publishVisualization();
    void publishPointCloud();
    double interpolateColorComponent(double magnitude, double min_val, double max_val) const;
    
    ros::NodeHandle nh_, pnh_;
    
    // 数据存储
    std::vector<ScanDataPoint> scan_data_;
    
    // 发布者
    ros::Publisher marker_pub_;
    ros::Publisher pointcloud_pub_;
    
    // 配置参数
    std::string csv_file_path_;
    std::string frame_id_;
    std::string marker_topic_;
    std::string pointcloud_topic_;
    
    // 可视化参数
    double arrow_length_;
    double arrow_shaft_ratio_;
    double arrow_head_ratio_;
    double arrow_head_length_ratio_;
    double magnitude_min_;
    double magnitude_max_;
    double alpha_;
    std::vector<double> color_low_;
    std::vector<double> color_high_;
    
    // 发布频率
    double publish_rate_;
    bool publish_pointcloud_;
    
    // 数据过滤
    double min_magnitude_threshold_;
};

} // namespace mag_arm_scan

