#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <vector>
#include <map>

namespace mag_sensor_node
{

    struct SensorInfo
    {
        int id;
        geometry_msgs::Pose pose;
    };

    class SensorConfig
    {
    public:
        static SensorConfig &getInstance()
        {
            static SensorConfig inst;
            return inst;
        }
        bool loadConfig(const ros::NodeHandle &nh);
        const std::vector<SensorInfo> &getAllSensors() const { return sensors_; }
        bool getSensorById(int id, SensorInfo &sensor) const;
        int getSensorCount() const { return sensors_.size(); }
    // 已移除 array_frame 与 array_pose 概念，保留接口最小化

    private:
        SensorConfig() = default;
        ~SensorConfig() = default;
        SensorConfig(const SensorConfig &) = delete;
        SensorConfig &operator=(const SensorConfig &) = delete;
    // array_frame_ 与 array_pose_ 已删除
        std::vector<SensorInfo> sensors_;
        std::map<int, size_t> id_index_map_;
    };

} // namespace mag_sensor_node
