#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <map>
#include "magnetic_pose_estimation/sensor_config.hpp"
#include "magnetic_pose_estimation/MagneticField.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <magnetic_pose_estimation/MagnetPose.h>

class MagneticFieldVisualizer {
public:
/**
     * @brief 磁场可视化器的构造函数
     * @param nh ROS节点句柄
     * @details 初始化发布器和订阅器，加载传感器配置，并初始化各类标记
     */
    MagneticFieldVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 为每种类型创建单独的发布器
        real_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("magnetic_field_markers/real", 100);
        sim_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("magnetic_field_markers/simulation", 100);
        pred_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("magnetic_field_markers/predicted", 100);
        magnet_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("magnet_marker", 10);

        // 创建订阅器
        real_sub_ = nh_.subscribe("/magnetic_field/real", 100, 
            &MagneticFieldVisualizer::realCallback, this);
        sim_sub_ = nh_.subscribe("/magnetic_field/simulation", 100, 
            &MagneticFieldVisualizer::simCallback, this);
        pred_sub_ = nh_.subscribe("/magnetic_field/predicted", 100, 
            &MagneticFieldVisualizer::predCallback, this);
        magnet_pose_sub_ = nh_.subscribe("/magnet_pose/simulation", 10, 
            &MagneticFieldVisualizer::magnetPoseCallback, this);

        // 初始化标记
        initializeMarkers();
        initializeMagnetMarker();
    }

private:
    // 添加ID到索引的映射
    std::map<int, size_t> id_to_index_;

/**
     * @brief 初始化所有传感器的标记
     * @details 为每个传感器创建三种不同类型的箭头标记（实际、仿真、预测），
     *          并设置它们的基本属性（颜色、大小、类型等）
     */
    void initializeMarkers() {
        const auto& sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();
        
        // 预分配空间
        sensor_markers_.markers.resize(sensors.size() * 3); // 每个传感器3种数据源
        
        // 初始化每个传感器的标记
        for (size_t i = 0; i < sensors.size(); ++i) {
            const auto& sensor = sensors[i];
            // 存储ID到索引的映射
            id_to_index_[sensor.id] = i;
            
            for (int j = 0; j < 3; ++j) {
                auto& marker = sensor_markers_.markers[i * 3 + j];
                marker.header.frame_id = magnetic_pose_estimation::SensorConfig::getInstance().getParentFrame();
                marker.ns = "magnetic_field_" + std::to_string(j);
                marker.id = sensor.id;  // 使用 id 而不是 label
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = sensor.pose;
                marker.scale.x = 0.005;  // 箭头长度
                marker.scale.y = 0.0005; // 箭头宽度
                marker.scale.z = 0.0005; // 箭头高度
                
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

/**
     * @brief 初始化磁铁标记的可视化属性
     * @details 设置磁铁的形状为圆柱体，并配置其大小、颜色等属性
     */
    void initializeMagnetMarker() {
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = "magnet";
        magnet_marker_.id = 0;
        magnet_marker_.type = visualization_msgs::Marker::CYLINDER;
        magnet_marker_.action = visualization_msgs::Marker::ADD;
        
        // 设置磁铁可视化大小
        magnet_marker_.scale.x = 0.002; // 直径
        magnet_marker_.scale.y = 0.002; // 直径
        magnet_marker_.scale.z = 0.004; // 高度
        
        // 设置磁铁颜色（黑色）
        magnet_marker_.color.r = 0.0;
        magnet_marker_.color.g = 0.0;
        magnet_marker_.color.b = 0.0;
        magnet_marker_.color.a = 1.0;
    }

/**
     * @brief 更新磁场标记
     * @param msg 包含磁场数据的消息指针
     * @param source_index 数据源索引（0:实际, 1:仿真, 2:预测）
     * @details 根据接收到的磁场数据更新对应传感器的箭头标记
     */
    void updateMarkers(const magnetic_pose_estimation::MagneticField::ConstPtr& msg, int source_index) {
        magnetic_pose_estimation::SensorInfo sensor;

        if (magnetic_pose_estimation::SensorConfig::getInstance().getSensorById(msg->sensor_id, sensor)) {
            // 使用映射获取正确的索引
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
            
            // 计算磁场向量的大小
            double magnitude = std::sqrt(
                std::pow(msg->mag_x, 2) + 
                std::pow(msg->mag_y, 2) + 
                std::pow(msg->mag_z, 2)
            );
            
            // 使用points来定义箭头
            marker.type = visualization_msgs::Marker::ARROW;
            marker.points.resize(2);
            
            // 起点（传感器位置）
            marker.points[0].x = 0.0;
            marker.points[0].y = 0.0;
            marker.points[0].z = 0.0;

            // 终点（磁场方向）
            double scale = 0.001; // 缩放因子，可以调整
            marker.points[1].x = msg->mag_x * scale;
            marker.points[1].y = msg->mag_y * scale;
            marker.points[1].z = msg->mag_z * scale;

            // 设置箭头的粗细
            marker.scale.x = 0.0001; // 箭头轴的直径
            marker.scale.y = 0.0002; // 箭头头部的直径

            // 确保 pose.position 是 (0, 0, 0)
            marker.pose.position.x = msg->sensor_pose.position.x;
            marker.pose.position.y = msg->sensor_pose.position.y;
            marker.pose.position.z = msg->sensor_pose.position.z;

            // 设置生存时间
            marker.lifetime = ros::Duration(0.1); 

            // 创建单独的MarkerArray用于发布
            visualization_msgs::MarkerArray current_markers;
            current_markers.markers.push_back(marker);
            
            // 根据数据源选择对应的发布器
            switch(source_index) {
                case 0:
                    real_marker_pub_.publish(current_markers);
                    break;
                case 1:
                    sim_marker_pub_.publish(current_markers);
                    break;
                case 2:
                    pred_marker_pub_.publish(current_markers);
                    break;
            }

            // 在发布 marker 之后，添加发布传感器 TF 的代码
            geometry_msgs::TransformStamped sensor_tf;
            sensor_tf.header.stamp = ros::Time::now();
            sensor_tf.header.frame_id = "world";  // 或其他合适的父坐标系
            sensor_tf.child_frame_id = "sensor_" + std::to_string(msg->sensor_id);  // 为每个传感器创建唯一的 frame_id
            
            // 设置传感器位置
            sensor_tf.transform.translation.x = msg->sensor_pose.position.x;
            sensor_tf.transform.translation.y = msg->sensor_pose.position.y;
            sensor_tf.transform.translation.z = msg->sensor_pose.position.z;
            
            // 设置传感器方向
            sensor_tf.transform.rotation = msg->sensor_pose.orientation;
            
            // 发布传感器 TF
            tf_broadcaster_.sendTransform(sensor_tf);
        }
    }

/**
     * @brief 处理磁铁位姿更新的回调函数
     * @param msg 包含磁铁位姿信息的消息指针
     * @details 更新磁铁标记的位置和方向，并发布相应的TF变换
     */
    void magnetPoseCallback(const magnetic_pose_estimation::MagnetPose::ConstPtr& msg) {
        // 更新磁铁标记
        magnet_marker_.header.stamp = msg->header.stamp;
        
        // 设置位姿
        magnet_marker_.pose.position = msg->position;
        magnet_marker_.pose.orientation = msg->orientation;
        
        // 发布磁铁标记
        visualization_msgs::MarkerArray magnet_markers;
        magnet_markers.markers.push_back(magnet_marker_);
        magnet_marker_pub_.publish(magnet_markers);

        // 发布 TF 变换
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "world";
        transform.child_frame_id = "magnet";
        
        // 设置位置和方向
        transform.transform.translation.x = msg->position.x;
        transform.transform.translation.y = msg->position.y;
        transform.transform.translation.z = msg->position.z;
        transform.transform.rotation = msg->orientation;

        // 发布 TF
        tf_broadcaster_.sendTransform(transform);
    }

/**
     * @brief 处理实际磁场数据的回调函数
     * @param msg 包含实际磁场数据的消息指针
     */
    void realCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 0);
    }

/**
     * @brief 处理仿真磁场数据的回调函数
     * @param msg 包含仿真磁场数据的消息指针
     */
    void simCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 1);
    }

/**
     * @brief 处理预测磁场数据的回调函数
     * @param msg 包含预测磁场数据的消息指针
     */
    void predCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        updateMarkers(msg, 2);
    }

    ros::NodeHandle& nh_;
    ros::Publisher real_marker_pub_, sim_marker_pub_, pred_marker_pub_, magnet_marker_pub_;  // 分别用于发布四种类型的标记
    ros::Subscriber real_sub_, sim_sub_, pred_sub_, magnet_pose_sub_;
    visualization_msgs::MarkerArray sensor_markers_;
    visualization_msgs::Marker magnet_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序执行状态码
 * @details 初始化ROS节点，创建磁场可视化器实例，并启动异步消息处理
 */
int main(int argc, char **argv)
{
    // 设置本地化支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");

    // 初始化ROS节点
    ros::init(argc, argv, "magnetic_field_visualizer");
    ros::NodeHandle nh;

    // 创建异步处理器，设置50个线程
    ros::AsyncSpinner spinner(50);
    spinner.start(); // 启动异步处理

    // 创建磁场可视化器实例
    MagneticFieldVisualizer visualizer(nh);

    // 等待节点关闭
    ros::waitForShutdown();
    return 0;
}