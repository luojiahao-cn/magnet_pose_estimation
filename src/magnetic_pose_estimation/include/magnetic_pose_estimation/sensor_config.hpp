#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <map>

namespace magnetic_pose_estimation {

struct SensorInfo {
    int id; 
    geometry_msgs::Pose pose;
};

class SensorConfig {
public:
    static SensorConfig& getInstance() {
        static SensorConfig instance;
        return instance;
    }

    bool loadConfig(const ros::NodeHandle& nh);
    
    // 获取所有传感器信息
    const std::vector<SensorInfo>& getAllSensors() const;
    
    // 根据ID获取单个传感器信息
    bool getSensorById(int id, SensorInfo& sensor) const;  
    
    // 获取坐标系信息
    std::string getParentFrame() const { return parent_frame_; }
    std::string getArrayFrame() const { return array_frame_; }
    geometry_msgs::Pose getArrayPose() const { return array_pose_; }

private:
    SensorConfig() = default;
    ~SensorConfig() = default;
    SensorConfig(const SensorConfig&) = delete;
    SensorConfig& operator=(const SensorConfig&) = delete;

    std::string parent_frame_;
    std::string array_frame_;
    geometry_msgs::Pose array_pose_;
    std::vector<SensorInfo> sensors_;
    std::map<int, size_t> id_index_map_;  
};

}  // namespace magnetic_pose_estimation