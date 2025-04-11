#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "magnetic_pose_estimation/sensor_config.hpp"
#include "magnetic_pose_estimation/MagneticField.h"
#include "magnetic_pose_estimation/MagnetPose.h"

class MagneticFieldVisualizer {
public:
    // 定义数据源类型
    enum class DataSource {
        REAL,
        SIMULATION,
        PREDICTED
    };

    MagneticFieldVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        initializePublishersAndSubscribers();
        initializeMarkers();
    }

private:
    void initializePublishersAndSubscribers() {
        // 统一发布器和订阅器的初始化
        const std::vector<std::string> sources = {"real", "simulation", "predicted"};
        
        for (const auto& source : sources) {
            magnetic_field_pubs_[source] = nh_.advertise<visualization_msgs::MarkerArray>(
                "magnetic_field_markers/" + source, 100);

            magnet_marker_pubs_[source] = nh_.advertise<visualization_msgs::MarkerArray>(
                "magnet_marker/" + source, 10);
            
            magnetic_field_subs_[source] = nh_.subscribe<magnetic_pose_estimation::MagneticField>(
                "/magnetic_field/" + source, 100,
                boost::bind(&MagneticFieldVisualizer::magneticFieldCallback, this, _1, source));
            
            magnet_pose_subs_[source] = nh_.subscribe<magnetic_pose_estimation::MagnetPose>(
                "/magnet_pose/" + source, 10,
                boost::bind(&MagneticFieldVisualizer::magnetPoseCallback, this, _1, source));
        }
    }

    void initializeMarkers() {
        const auto& sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();
        sensor_markers_.markers.resize(sensors.size() * 3);
        
        for (size_t i = 0; i < sensors.size(); ++i) {
            id_to_index_[sensors[i].id] = i;
            initializeSensorMarkers(sensors[i], i);
        }

        initializeMagnetMarker();
    }

    void initializeSensorMarkers(const magnetic_pose_estimation::SensorInfo& sensor, size_t index) {
        // 从参数服务器读取颜色配置
        std::vector<std::vector<double>> colors;
        colors.resize(3);
        nh_.getParam("/visualization/magnetic_field_marker/colors/real", colors[0]);
        nh_.getParam("/visualization/magnetic_field_marker/colors/simulation", colors[1]);
        nh_.getParam("/visualization/magnetic_field_marker/colors/predicted", colors[2]);

        // 读取箭头参数
        double shaft_diameter, head_diameter;
        nh_.getParam("/visualization/magnetic_field_marker/arrow/shaft_diameter", shaft_diameter);
        nh_.getParam("/visualization/magnetic_field_marker/arrow/head_diameter", head_diameter);

        for (int j = 0; j < 3; ++j) {
            auto& marker = sensor_markers_.markers[index * 3 + j];
            marker.header.frame_id = magnetic_pose_estimation::SensorConfig::getInstance().getParentFrame();
            marker.ns = "magnetic_field_" + std::to_string(j);
            marker.id = sensor.id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = sensor.pose;
            
            // 使用箭头参数设置缩放
            marker.scale.x = shaft_diameter;
            marker.scale.y = head_diameter;
            marker.scale.z = head_diameter;
            
            marker.color.r = colors[j][0];
            marker.color.g = colors[j][1];
            marker.color.b = colors[j][2];
            marker.color.a = 1.0;
        }
    }

    void initializeMagnetMarker() {
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = "magnet";
        magnet_marker_.id = 0;
        magnet_marker_.type = visualization_msgs::Marker::CYLINDER;
        magnet_marker_.action = visualization_msgs::Marker::ADD;
        
        // 从参数服务器读取磁铁标记参数
        std::vector<double> scale, color;
        nh_.getParam("/visualization/magnet_marker/scale/x", magnet_marker_.scale.x);
        nh_.getParam("/visualization/magnet_marker/scale/y", magnet_marker_.scale.y);
        nh_.getParam("/visualization/magnet_marker/scale/z", magnet_marker_.scale.z);
        
        nh_.getParam("/visualization/magnet_marker/color", color);
        magnet_marker_.color.r = color[0];
        magnet_marker_.color.g = color[1];
        magnet_marker_.color.b = color[2];
        magnet_marker_.color.a = color[3];
    }

    void magneticFieldCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg,
                             const std::string& source) {
        int source_index = sourceToIndex(source);
        updateMarkers(msg, source_index);
    }

    void magnetPoseCallback(const magnetic_pose_estimation::MagnetPose::ConstPtr& msg,
                          const std::string& source) {
        updateMagnetMarker(msg, magnet_marker_pubs_[source], "magnet_" + source);
    }

    void updateMarkers(const magnetic_pose_estimation::MagneticField::ConstPtr& msg, int source_index) {
        magnetic_pose_estimation::SensorInfo sensor;

        if (magnetic_pose_estimation::SensorConfig::getInstance().getSensorById(msg->sensor_id, sensor)) {
            auto it = id_to_index_.find(msg->sensor_id);
            if (it == id_to_index_.end()) {
                ROS_WARN("未找到传感器ID %d 的索引映射", msg->sensor_id);
                return;
            }
            
            size_t marker_index = it->second * 3 + source_index;
            if (marker_index >= sensor_markers_.markers.size()) {
                ROS_ERROR("标记索引越界: %zu >= %zu", marker_index, sensor_markers_.markers.size());
                return;
            }
            
            auto& marker = sensor_markers_.markers[marker_index];
            marker.header.stamp = ros::Time::now();
            
            double magnitude = std::sqrt(
                std::pow(msg->mag_x, 2) + 
                std::pow(msg->mag_y, 2) + 
                std::pow(msg->mag_z, 2)
            );
            
            marker.type = visualization_msgs::Marker::ARROW;
            marker.points.resize(2);
            
            marker.points[0].x = 0.0;
            marker.points[0].y = 0.0;
            marker.points[0].z = 0.0;

            // 从参数服务器读取磁场缩放因子和标记存活时间
            double field_scale, lifetime;
            nh_.getParam("/visualization/magnetic_field_marker/field_scale", field_scale);
            nh_.getParam("/visualization/magnetic_field_marker/lifetime", lifetime);

            marker.points[1].x = msg->mag_x * field_scale;
            marker.points[1].y = msg->mag_y * field_scale;
            marker.points[1].z = msg->mag_z * field_scale;

            // 从参数服务器读取箭头参数
            double shaft_diameter, head_diameter;
            nh_.getParam("/visualization/magnetic_field_marker/arrow/shaft_diameter", shaft_diameter);
            nh_.getParam("/visualization/magnetic_field_marker/arrow/head_diameter", head_diameter);

            // 设置箭头的缩放参数
            marker.scale.x = shaft_diameter;  // 箭头轴的直径
            marker.scale.y = head_diameter;   // 箭头头部的直径
            marker.scale.z = head_diameter;   // 箭头头部的直径（通常与y相同）

            marker.pose.position.x = msg->sensor_pose.position.x;
            marker.pose.position.y = msg->sensor_pose.position.y;
            marker.pose.position.z = msg->sensor_pose.position.z;

            marker.lifetime = ros::Duration(lifetime);

            visualization_msgs::MarkerArray current_markers;
            current_markers.markers.push_back(marker);
            
            switch(source_index) {
                case 0:
                    magnetic_field_pubs_["real"].publish(current_markers);
                    break;
                case 1:
                    magnetic_field_pubs_["simulation"].publish(current_markers);
                    break;
                case 2:
                    magnetic_field_pubs_["predicted"].publish(current_markers);
                    break;
            }

            geometry_msgs::TransformStamped sensor_tf;
            sensor_tf.header.stamp = ros::Time::now();
            sensor_tf.header.frame_id = "world";
            sensor_tf.child_frame_id = "sensor_" + std::to_string(msg->sensor_id);
            
            sensor_tf.transform.translation.x = msg->sensor_pose.position.x;
            sensor_tf.transform.translation.y = msg->sensor_pose.position.y;
            sensor_tf.transform.translation.z = msg->sensor_pose.position.z;
            
            sensor_tf.transform.rotation = msg->sensor_pose.orientation;
            
            tf_broadcaster_.sendTransform(sensor_tf);
        }
    }

    void updateMagnetMarker(const magnetic_pose_estimation::MagnetPose::ConstPtr &msg,
                            const ros::Publisher &publisher,
                            const std::string &frame_id) {
        magnet_marker_.header.stamp = msg->header.stamp;
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = frame_id;
        magnet_marker_.pose.position = msg->position;
        magnet_marker_.pose.orientation = msg->orientation;

        visualization_msgs::MarkerArray magnet_markers;
        magnet_markers.markers.push_back(magnet_marker_);
        publisher.publish(magnet_markers);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "world";
        transform.child_frame_id = frame_id;

        transform.transform.translation.x = msg->position.x;
        transform.transform.translation.y = msg->position.y;
        transform.transform.translation.z = msg->position.z;
        transform.transform.rotation = msg->orientation;

        tf_broadcaster_.sendTransform(transform);
    }

    ros::NodeHandle& nh_;
    std::map<std::string, ros::Publisher> magnetic_field_pubs_;
    std::map<std::string, ros::Publisher> magnet_marker_pubs_;
    std::map<std::string, ros::Subscriber> magnetic_field_subs_;
    std::map<std::string, ros::Subscriber> magnet_pose_subs_;
    std::map<int, size_t> id_to_index_;
    
    visualization_msgs::MarkerArray sensor_markers_;
    visualization_msgs::Marker magnet_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    int sourceToIndex(const std::string& source) {
        static const std::map<std::string, int> source_map = {
            {"real", 0}, {"simulation", 1}, {"predicted", 2}
        };
        return source_map.at(source);
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnetic_field_visualizer");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(50);
    spinner.start();
    
    MagneticFieldVisualizer visualizer(nh);
    ros::waitForShutdown();
    return 0;
}