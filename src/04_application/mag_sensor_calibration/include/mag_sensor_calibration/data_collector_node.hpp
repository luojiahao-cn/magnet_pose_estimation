#pragma once

#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_msgs/MagSensorBatch.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <atomic>

namespace mag_sensor_calibration
{

/**
 * @brief 单个数据点（包含角度和传感器读数）
 */
struct DataPoint
{
    ros::Time timestamp;
    uint32_t sensor_id;
    double angle;                    // 角度（弧度）
    Eigen::Vector3d mag_reading;    // 磁场读数 (mT)
    geometry_msgs::Pose sensor_pose;  // 传感器位姿（可选）
    
    DataPoint()
        : sensor_id(0)
        , angle(0.0)
        , mag_reading(Eigen::Vector3d::Zero())
    {}
};

/**
 * @brief 数据采集节点
 * 
 * 功能：
 * 1. 订阅传感器数据（单个或批量）
 * 2. 从TF获取当前角度（通过查询传感器阵列相对于参考坐标系的旋转）
 * 3. 记录数据点（角度 + 传感器读数）
 * 4. 保存数据到CSV文件
 */
class DataCollectorNode
{
public:
    DataCollectorNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~DataCollectorNode();

    void start();

    /**
     * @brief 开始采集数据
     * @param session_id 会话ID
     * @param reference_frame 参考坐标系（用于计算角度）
     * @param sensor_array_frame 传感器阵列坐标系
     * @param rotation_axis 旋转轴 ("x", "y", "z")
     * @param save_to_csv 是否保存到CSV
     * @param csv_filepath CSV文件路径
     */
    void startCollection(const std::string& session_id,
                        const std::string& reference_frame,
                        const std::string& sensor_array_frame,
                        const std::string& rotation_axis,
                        bool save_to_csv,
                        const std::string& csv_filepath);

    /**
     * @brief 停止采集数据
     */
    void stopCollection();

    /**
     * @brief 获取已采集的数据
     * @param sensor_id 传感器ID（0表示所有传感器）
     * @return 数据点列表
     */
    std::vector<DataPoint> getCollectedData(uint32_t sensor_id = 0) const;

    /**
     * @brief 清空已采集的数据
     */
    void clearData();

    /**
     * @brief 获取采集状态
     */
    bool isCollecting() const { return collecting_; }

    /**
     * @brief 获取已采集的样本数量
     */
    size_t getSampleCount() const;

private:
    void setLogLevel();
    void loadParameters();
    void setupSubscribers();
    void setupPublishers();

    // 传感器数据回调
    void sensorDataCallback(const mag_core_msgs::MagSensorDataConstPtr& msg);
    void sensorBatchCallback(const mag_core_msgs::MagSensorBatchConstPtr& msg);

    // 计算当前角度
    double computeCurrentAngle(const std::string& reference_frame,
                              const std::string& sensor_array_frame,
                              const std::string& rotation_axis);

    // 保存数据到CSV
    void saveToCSV(const DataPoint& point);
    void flushCSV();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ROS接口
    ros::Subscriber sensor_sub_;
    ros::Subscriber batch_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 配置参数
    std::string input_topic_;
    bool use_batch_topic_;

    // 采集状态
    std::atomic<bool> collecting_;
    std::string session_id_;
    std::string reference_frame_;
    std::string sensor_array_frame_;
    std::string rotation_axis_;
    bool save_to_csv_;
    std::string csv_filepath_;
    std::ofstream csv_file_;

    // 数据存储
    mutable std::mutex data_mutex_;
    std::map<uint32_t, std::vector<DataPoint>> collected_data_;  // 按传感器ID分组
    size_t total_samples_;
};

} // namespace mag_sensor_calibration

