#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "magnetic_pose_estimation/sensor_config.hpp"
#include "magnetic_pose_estimation/MagneticField.h"

class MagneticFieldVisualizer {
public:
    MagneticFieldVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 创建发布器
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("magnetic_field_markers", 1);

        // 创建订阅器
        real_sub_ = nh_.subscribe("/magnetic_field/real", 1, 
            &MagneticFieldVisualizer::realCallback, this);
        sim_sub_ = nh_.subscribe("/magnetic_field/simulation", 1, 
            &MagneticFieldVisualizer::simCallback, this);
        pred_sub_ = nh_.subscribe("/magnetic_field/predicted", 1, 
            &MagneticFieldVisualizer::predCallback, this);

        // 初始化标记
        initializeMarkers();
    }

private:
    void initializeMarkers() {
        const auto& sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();
        
        // 预分配空间
        sensor_markers_.markers.resize(sensors.size() * 3); // 每个传感器3种数据源
        
        // 初始化每个传感器的标记
        for (size_t i = 0; i < sensors.size(); ++i) {
            const auto& sensor = sensors[i];
            for (int j = 0; j < 3; ++j) {
                auto& marker = sensor_markers_.markers[i * 3 + j];
                marker.header.frame_id = magnetic_pose_estimation::SensorConfig::getInstance().getParentFrame();
                marker.ns = "magnetic_field_" + std::to_string(j);
                marker.id = sensor.id;  // 使用 id 而不是 label
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = sensor.pose;
                marker.scale.x = 0.05;  // 箭头长度
                marker.scale.y = 0.005; // 箭头宽度
                marker.scale.z = 0.005; // 箭头高度
                
                // 为不同数据源设置不同颜色
                switch(j) {
                    case 0: // 实际数据 - 红色
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        break;
                    case 1: // 仿真数据 - 绿色
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        break;
                    case 2: // 预测数据 - 蓝色
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        break;
                }
                marker.color.a = 1.0; // 不透明度
            }
        }
    }

    void updateMarkers(const magnetic_pose_estimation::MagneticField::ConstPtr& msg, int source_index) {
        magnetic_pose_estimation::SensorInfo sensor;
        // 使用 sensor_id 和 getSensorById
        if (magnetic_pose_estimation::SensorConfig::getInstance().getSensorById(msg->sensor_id, sensor)) {
            auto& marker = sensor_markers_.markers[sensor.id * 3 + source_index];
            marker.header.stamp = ros::Time::now();
            
            // 设置箭头方向和长度
            double magnitude = std::sqrt(
                std::pow(msg->mag_x, 2) + 
                std::pow(msg->mag_y, 2) + 
                std::pow(msg->mag_z, 2)
            );
            marker.scale.x = magnitude * 0.1; // 根据磁场强度调整箭头长度
            
            // 计算箭头方向
            tf2::Quaternion q;
            q.setRPY(
                std::atan2(msg->mag_z, msg->mag_y), 
                std::atan2(msg->mag_z, msg->mag_x), 
                std::atan2(msg->mag_y, msg->mag_x)
            );
            
            // 更新箭头方向
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();
            
            // 更新传感器位姿
            marker.pose.position = msg->sensor_pose.position;
            
            // 发布更新后的标记
            marker_pub_.publish(sensor_markers_);
        }
    }

    void realCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 0);
    }

    void simCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 1);
    }

    void predCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 2);
    }

    ros::NodeHandle& nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber real_sub_, sim_sub_, pred_sub_;
    visualization_msgs::MarkerArray sensor_markers_;
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnetic_field_visualizer");
    ros::NodeHandle nh;

    MagneticFieldVisualizer visualizer(nh);

    ros::spin();
    return 0;
}