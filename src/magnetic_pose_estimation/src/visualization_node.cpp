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
    MagneticFieldVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 初始化标记
        initializeMarkerTemplates();

        // 订阅主题并设置回调
        subscribeToTopics();
    }

private:
    void subscribeToTopics() {
        // 获取所有 /magnetic_field/ 下的话题
        ros::master::V_TopicInfo topic_info;
        ros::master::getTopics(topic_info);

        for (const auto& topic : topic_info) {
            // 处理磁场话题
            if (topic.name.find("/magnetic_field/") == 0) {
                std::string topic_name = topic.name;
                std::string source = topic_name.substr(std::string("/magnetic_field/").length());
                
                // 为每个源创建发布器
                magnetic_field_pubs_[source] = nh_.advertise<visualization_msgs::MarkerArray>(
                    "magnetic_field_markers/" + source, 100);
                
                magnetic_field_subs_[source] = nh_.subscribe<magnetic_pose_estimation::MagneticField>(
                    topic_name, 100,
                    boost::bind(&MagneticFieldVisualizer::magneticFieldCallback, this, _1, source));
                
                ROS_INFO("订阅磁场话题: %s", topic_name.c_str());
            }
            // 处理磁铁位姿话题
            else if (topic.name.find("/magnet_pose/") == 0) {
                std::string topic_name = topic.name;
                std::string source = topic_name.substr(std::string("/magnet_pose/").length());
                
                magnet_marker_pubs_[source] = nh_.advertise<visualization_msgs::MarkerArray>(
                    "magnet_marker/" + source, 10);
                
                magnet_pose_subs_[source] = nh_.subscribe<magnetic_pose_estimation::MagnetPose>(
                    topic_name, 10,
                    boost::bind(&MagneticFieldVisualizer::magnetPoseCallback, this, _1, source));
                
                ROS_INFO("订阅磁铁位姿话题: %s", topic_name.c_str());
            }
        }
    }

    void initializeMarkerTemplates() {
        // 初始化传感器标记模板
        sensor_marker_template_.header.frame_id = magnetic_pose_estimation::SensorConfig::getInstance().getParentFrame();
        sensor_marker_template_.type = visualization_msgs::Marker::ARROW;
        sensor_marker_template_.action = visualization_msgs::Marker::ADD;

        // 从参数服务器读取箭头参数
        double shaft_diameter, head_diameter;
        nh_.getParam("/visualization_config/magnetic_field_marker/arrow/shaft_diameter", shaft_diameter);
        nh_.getParam("/visualization_config/magnetic_field_marker/arrow/head_diameter", head_diameter);

        sensor_marker_template_.scale.x = shaft_diameter;
        sensor_marker_template_.scale.y = head_diameter;
        sensor_marker_template_.scale.z = head_diameter;
        
        // 读取箭头颜色
        std::vector<double> color;
        nh_.getParam("/visualization_config/magnetic_field_marker/color", color);
        sensor_marker_template_.color.r = color[0];
        sensor_marker_template_.color.g = color[1];
        sensor_marker_template_.color.b = color[2];
        sensor_marker_template_.color.a = color[3];

        // 初始化磁铁标记模板
        initializeMagnetMarker();
    }

    void initializeMagnetMarker() {
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = "magnet";
        magnet_marker_.id = 0;
        magnet_marker_.type = visualization_msgs::Marker::CYLINDER;
        magnet_marker_.action = visualization_msgs::Marker::ADD;
        
        // 从参数服务器读取磁铁标记参数
        std::vector<double> scale, color;
        nh_.getParam("/visualization_config/magnet_marker/scale/x", magnet_marker_.scale.x);
        nh_.getParam("/visualization_config/magnet_marker/scale/y", magnet_marker_.scale.y);
        nh_.getParam("/visualization_config/magnet_marker/scale/z", magnet_marker_.scale.z);

        nh_.getParam("/visualization_config/magnet_marker/color", color);
        magnet_marker_.color.r = color[0];
        magnet_marker_.color.g = color[1];
        magnet_marker_.color.b = color[2];
        magnet_marker_.color.a = color[3];
    }

    void magneticFieldCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg,
                             const std::string& source) {
        updateMarkers(msg, source);
    }

    void magnetPoseCallback(const magnetic_pose_estimation::MagnetPose::ConstPtr& msg,
                          const std::string& source) {
        updateMagnetMarker(msg, magnet_marker_pubs_[source], "magnet_" + source);
    }

    void updateMarkers(const magnetic_pose_estimation::MagneticField::ConstPtr& msg, const std::string& source) {
        visualization_msgs::Marker marker = sensor_marker_template_;  // 使用模板创建新标记
        marker.header.stamp = ros::Time::now();
        marker.ns = source;
        marker.id = msg->sensor_id;
        
        // 从参数服务器读取磁场缩放因子和标记存活时间
        double field_scale, lifetime;
        nh_.getParam("/visualization_config/magnetic_field_marker/field_scale", field_scale);
        nh_.getParam("/visualization_config/magnetic_field_marker/lifetime", lifetime);

        marker.points.resize(2);
        marker.points[0].x = marker.points[0].y = marker.points[0].z = 0.0;
        marker.points[1].x = msg->mag_x * field_scale;
        marker.points[1].y = msg->mag_y * field_scale;
        marker.points[1].z = msg->mag_z * field_scale;

        marker.pose.position = msg->sensor_pose.position;
        marker.pose.orientation = msg->sensor_pose.orientation;
        marker.lifetime = ros::Duration(lifetime);

        visualization_msgs::MarkerArray current_markers;
        current_markers.markers.push_back(marker);
        magnetic_field_pubs_[source].publish(current_markers);

        // 发布传感器TF
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
    
    visualization_msgs::Marker sensor_marker_template_;  // 传感器标记模板
    visualization_msgs::Marker magnet_marker_;          // 磁铁标记模板
    tf2_ros::TransformBroadcaster tf_broadcaster_;
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