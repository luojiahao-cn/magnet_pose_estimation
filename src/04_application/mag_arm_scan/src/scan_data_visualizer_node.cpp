/**
 * @file scan_data_visualizer_node.cpp
 * @brief 扫描数据可视化节点实现
 * 
 * 功能概述：
 * - 从CSV文件读取扫描数据
 * - 发布可视化标记到ROS话题
 * - 在RViz中显示磁场向量和采样点
 */

#include "mag_arm_scan/scan_data_visualizer_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <tuple>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace mag_arm_scan {

ScanDataVisualizerNode::ScanDataVisualizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
    loadParams();
    
    // 初始化发布者
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);
    if (publish_pointcloud_) {
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_, 1);
    }
    
    // 加载CSV数据
    if (!loadCSVData(csv_file_path_)) {
        ROS_ERROR("[scan_data_visualizer] Failed to load CSV data from: %s", csv_file_path_.c_str());
        return;
    }
    
    ROS_INFO("[scan_data_visualizer] Loaded %zu data points from CSV file", scan_data_.size());
}

void ScanDataVisualizerNode::loadParams() {
    // 文件路径
    pnh_.param<std::string>("csv_file", csv_file_path_, "");
    if (csv_file_path_.empty()) {
        // 尝试从参数服务器获取默认路径
        std::string default_dir;
        pnh_.param<std::string>("data_dir", default_dir, "");
        if (!default_dir.empty()) {
            // 查找最新的扫描目录
            // 这里简化处理，用户可以通过参数指定完整路径
            ROS_WARN("[scan_data_visualizer] csv_file not specified, please set ~csv_file parameter");
        }
    }
    
    // 坐标系
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    
    // 话题名称
    pnh_.param<std::string>("marker_topic", marker_topic_, "/scan_data/markers");
    pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/scan_data/pointcloud");
    
    // 可视化参数
    pnh_.param<double>("arrow_length", arrow_length_, 0.02);
    pnh_.param<double>("arrow_shaft_ratio", arrow_shaft_ratio_, 0.1);
    pnh_.param<double>("arrow_head_ratio", arrow_head_ratio_, 0.25);
    pnh_.param<double>("arrow_head_length_ratio", arrow_head_length_ratio_, 0.3);
    pnh_.param<double>("magnitude_min", magnitude_min_, 0.0);
    pnh_.param<double>("magnitude_max", magnitude_max_, 5.0);
    pnh_.param<double>("alpha", alpha_, 1.0);
    pnh_.param<double>("min_magnitude_threshold", min_magnitude_threshold_, 0.001);
    
    // 颜色配置
    color_low_.resize(3, 0.0);
    color_high_.resize(3, 1.0);
    if (pnh_.hasParam("color_low")) {
        XmlRpc::XmlRpcValue color_low;
        pnh_.getParam("color_low", color_low);
        if (color_low.getType() == XmlRpc::XmlRpcValue::TypeArray && color_low.size() >= 3) {
            color_low_[0] = static_cast<double>(color_low[0]);
            color_low_[1] = static_cast<double>(color_low[1]);
            color_low_[2] = static_cast<double>(color_low[2]);
        }
    } else {
        color_low_ = {0.0, 0.0, 1.0};  // 蓝色
    }
    
    if (pnh_.hasParam("color_high")) {
        XmlRpc::XmlRpcValue color_high;
        pnh_.getParam("color_high", color_high);
        if (color_high.getType() == XmlRpc::XmlRpcValue::TypeArray && color_high.size() >= 3) {
            color_high_[0] = static_cast<double>(color_high[0]);
            color_high_[1] = static_cast<double>(color_high[1]);
            color_high_[2] = static_cast<double>(color_high[2]);
        }
    } else {
        color_high_ = {1.0, 0.0, 0.0};  // 红色
    }
    
    // 发布参数
    pnh_.param<double>("publish_rate", publish_rate_, 1.0);
    pnh_.param<bool>("publish_pointcloud", publish_pointcloud_, false);
}

bool ScanDataVisualizerNode::loadCSVData(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("[scan_data_visualizer] Cannot open file: %s", file_path.c_str());
        return false;
    }
    
    scan_data_.clear();
    std::string line;
    
    // 跳过标题行
    if (!std::getline(file, line)) {
        ROS_ERROR("[scan_data_visualizer] CSV file is empty");
        return false;
    }
    
    // 读取数据行
    size_t line_num = 1;
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        ScanDataPoint point;
        
        // 解析CSV行：timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,sensor_id,frame_id
        try {
            if (!std::getline(iss, token, ',')) continue;
            point.timestamp = std::stod(token);
            
            if (!std::getline(iss, token, ',')) continue;
            point.mag_x = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.mag_y = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.mag_z = std::stod(token);
            
            if (!std::getline(iss, token, ',')) continue;
            point.pos_x = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.pos_y = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.pos_z = std::stod(token);
            
            if (!std::getline(iss, token, ',')) continue;
            point.ori_x = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.ori_y = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.ori_z = std::stod(token);
            if (!std::getline(iss, token, ',')) continue;
            point.ori_w = std::stod(token);
            
            if (!std::getline(iss, token, ',')) continue;
            point.sensor_id = static_cast<uint32_t>(std::stoul(token));
            
            if (std::getline(iss, token, ',')) {
                // 移除可能的换行符和空格
                token.erase(std::remove(token.begin(), token.end(), '\r'), token.end());
                token.erase(std::remove(token.begin(), token.end(), '\n'), token.end());
                point.frame_id = token;
            } else {
                point.frame_id = frame_id_;
            }
            
            scan_data_.push_back(point);
        } catch (const std::exception &e) {
            ROS_WARN("[scan_data_visualizer] Error parsing line %zu: %s", line_num, e.what());
            continue;
        }
    }
    
    file.close();
    return !scan_data_.empty();
}

void ScanDataVisualizerNode::publishVisualization() {
    if (scan_data_.empty()) {
        ROS_WARN("[scan_data_visualizer] No data to visualize");
        return;
    }
    
    visualization_msgs::MarkerArray markers;
    const ros::Time now = ros::Time::now();
    
    // 按位置分组，然后按传感器ID分组
    // 使用位置的三元组作为键（使用容差来合并相近位置）
    const double position_tolerance = 0.001;  // 位置容差（米）
    
    // 位置键：将位置坐标四舍五入到容差精度
    auto getPositionKey = [position_tolerance](double x, double y, double z) {
        int key_x = static_cast<int>(std::round(x / position_tolerance));
        int key_y = static_cast<int>(std::round(y / position_tolerance));
        int key_z = static_cast<int>(std::round(z / position_tolerance));
        return std::make_tuple(key_x, key_y, key_z);
    };
    
    // 按位置和传感器ID分组
    // 结构：position_sensor_groups[位置键][传感器ID] = 该位置该传感器的所有采样点
    std::map<std::tuple<int, int, int>, std::map<uint32_t, std::vector<ScanDataPoint>>> position_sensor_groups;
    for (const auto &point : scan_data_) {
        auto pos_key = getPositionKey(point.pos_x, point.pos_y, point.pos_z);
        position_sensor_groups[pos_key][point.sensor_id].push_back(point);
    }
    
    ROS_INFO("[scan_data_visualizer] Grouped data into %zu unique positions", position_sensor_groups.size());
    
    int marker_id = 0;
    int skipped_count = 0;
    size_t total_positions = 0;
    size_t total_sensors = 0;
    
    // 为每个位置和传感器创建标记
    // 外层循环：遍历每个位置
    for (const auto &pos_group : position_sensor_groups) {
        total_positions++;
        // 内层循环：遍历该位置的每个传感器
        for (const auto &sensor_group : pos_group.second) {
            total_sensors++;
            uint32_t sensor_id = sensor_group.first;
            const auto &points = sensor_group.second;
            
            if (points.empty()) continue;
            
            // 计算该位置该传感器的平均磁场向量（合并同一位置同一传感器的多个采样点）
            double sum_mag_x = 0.0, sum_mag_y = 0.0, sum_mag_z = 0.0;
            double sum_pos_x = 0.0, sum_pos_y = 0.0, sum_pos_z = 0.0;
            
            for (const auto &point : points) {
                sum_mag_x += point.mag_x;
                sum_mag_y += point.mag_y;
                sum_mag_z += point.mag_z;
                sum_pos_x += point.pos_x;
                sum_pos_y += point.pos_y;
                sum_pos_z += point.pos_z;
            }
            
            size_t count = points.size();
            geometry_msgs::Vector3 avg_mag;
            avg_mag.x = sum_mag_x / count;
            avg_mag.y = sum_mag_y / count;
            avg_mag.z = sum_mag_z / count;
            
            geometry_msgs::Point start_point;
            start_point.x = sum_pos_x / count;
            start_point.y = sum_pos_y / count;
            start_point.z = sum_pos_z / count;
            
            // 计算磁场强度
            double magnitude = std::sqrt(avg_mag.x * avg_mag.x + avg_mag.y * avg_mag.y + avg_mag.z * avg_mag.z);
            
            if (magnitude < min_magnitude_threshold_) {
                skipped_count++;
                continue;
            }
            
            // 计算箭头终点
            double arrow_length_current = std::max(magnitude * arrow_length_, 1e-6);
            geometry_msgs::Point end_point = start_point;
            end_point.x += avg_mag.x * arrow_length_;
            end_point.y += avg_mag.y * arrow_length_;
            end_point.z += avg_mag.z * arrow_length_;
            
            // 创建箭头标记
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = now;
            marker.ns = "scan_mag_field";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
            
            // 设置箭头尺寸
            double shaft_diameter = std::max(arrow_shaft_ratio_ * arrow_length_current, 1e-4);
            double head_diameter = std::max(arrow_head_ratio_ * arrow_length_current, shaft_diameter * 1.2);
            double head_length = std::max(arrow_head_length_ratio_ * arrow_length_current, head_diameter * 0.8);
            marker.scale.x = shaft_diameter;
            marker.scale.y = head_diameter;
            marker.scale.z = head_length;
            marker.pose.orientation.w = 1.0;
            
            // 根据磁场强度插值颜色
            double normalized_mag = std::clamp((magnitude - magnitude_min_) / (magnitude_max_ - magnitude_min_), 0.0, 1.0);
            marker.color.r = static_cast<float>(std::clamp(
                color_low_[0] + (color_high_[0] - color_low_[0]) * normalized_mag, 0.0, 1.0));
            marker.color.g = static_cast<float>(std::clamp(
                color_low_[1] + (color_high_[1] - color_low_[1]) * normalized_mag, 0.0, 1.0));
            marker.color.b = static_cast<float>(std::clamp(
                color_low_[2] + (color_high_[2] - color_low_[2]) * normalized_mag, 0.0, 1.0));
            marker.color.a = static_cast<float>(std::clamp(alpha_, 0.0, 1.0));
            marker.lifetime = ros::Duration(0.0);  // 永久显示
            
            markers.markers.push_back(marker);
        }
    }
    
    if (!markers.markers.empty()) {
        marker_pub_.publish(markers);
        ROS_INFO("[scan_data_visualizer] Published %zu markers: %zu positions × %zu sensors (avg %.1f sensors per position, skipped %d)", 
                 markers.markers.size(), total_positions, total_sensors, 
                 total_positions > 0 ? static_cast<double>(total_sensors) / total_positions : 0.0, skipped_count);
    } else {
        ROS_WARN("[scan_data_visualizer] No markers to publish (skipped %d)", skipped_count);
    }
}

void ScanDataVisualizerNode::publishPointCloud() {
    if (scan_data_.empty() || !publish_pointcloud_) {
        return;
    }
    
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = ros::Time::now();
    cloud.width = scan_data_.size();
    cloud.height = 1;
    cloud.is_dense = false;
    
    // 定义点云字段
    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
    
    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);
    
    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);
    
    field.name = "intensity";
    field.offset = 12;
    cloud.fields.push_back(field);
    
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
    
    // 填充点云数据
    float *data_ptr = reinterpret_cast<float*>(cloud.data.data());
    for (const auto &point : scan_data_) {
        *data_ptr++ = static_cast<float>(point.pos_x);
        *data_ptr++ = static_cast<float>(point.pos_y);
        *data_ptr++ = static_cast<float>(point.pos_z);
        
        // 使用磁场强度作为强度值
        double magnitude = std::sqrt(point.mag_x * point.mag_x + 
                                     point.mag_y * point.mag_y + 
                                     point.mag_z * point.mag_z);
        *data_ptr++ = static_cast<float>(magnitude);
    }
    
    pointcloud_pub_.publish(cloud);
    ROS_DEBUG("[scan_data_visualizer] Published point cloud with %zu points", scan_data_.size());
}

double ScanDataVisualizerNode::interpolateColorComponent(double magnitude, double min_val, double max_val) const {
    double normalized = std::clamp((magnitude - magnitude_min_) / (magnitude_max_ - magnitude_min_), 0.0, 1.0);
    return min_val + (max_val - min_val) * normalized;
}

void ScanDataVisualizerNode::run() {
    ros::Rate rate(publish_rate_);
    
    ROS_INFO("[scan_data_visualizer] Starting visualization loop at %.1f Hz", publish_rate_);
    
    while (ros::ok()) {
        publishVisualization();
        if (publish_pointcloud_) {
            publishPointCloud();
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

} // namespace mag_arm_scan

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_data_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    mag_arm_scan::ScanDataVisualizerNode node(nh, pnh);
    node.run();
    
    return 0;
}
