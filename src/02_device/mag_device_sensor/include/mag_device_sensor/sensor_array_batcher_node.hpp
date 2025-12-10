#pragma once

#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_msgs/MagSensorBatch.h>

#include <ros/ros.h>

#include <map>
#include <mutex>
#include <string>

namespace mag_device_sensor
{

/**
 * @brief 传感器阵列批量打包节点
 * 
 * 功能：
 * 1. 订阅单个传感器的磁场数据（MagSensorData）
 * 2. 收集所有传感器的数据
 * 3. 当收集齐一组数据时，立即打包成批量数据（MagSensorBatch）发布
 * 
 * 优势：
 * - 减少消息数量（从 N×100Hz 降到实际发布频率）
 * - 确保批量数据的时间戳同步
 * - 避免订阅者队列溢出
 * - 收集齐一组数据才发布，保证数据完整性
 */
class SensorArrayBatcherNode
{
public:
    SensorArrayBatcherNode(ros::NodeHandle nh, ros::NodeHandle pnh);

    void start();

private:
    /**
     * @brief 设置 ROS 日志级别
     */
    void setLogLevel();

    /**
     * @brief 加载配置参数
     */
    void loadParameters();

    /**
     * @brief 设置订阅者和发布者
     */
    void setupSubscribers();
    void setupPublishers();

    /**
     * @brief 单个传感器数据回调
     * @param msg 传感器数据消息
     */
    void sensorCallback(const mag_core_msgs::MagSensorDataConstPtr &msg);

    /**
     * @brief 构建并发布批量数据
     * @return 是否成功发布
     */
    bool publishBatch();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // 配置参数
    std::string input_topic_;           ///< 输入话题（单个传感器数据）
    std::string output_topic_;          ///< 输出话题（批量数据）
    std::string output_frame_;          ///< 参考坐标系（用于批量消息 header.frame_id，订阅者用于查询传感器位置）
    double timeout_seconds_;             ///< 传感器数据超时时间（秒）
    size_t total_sensors_;               ///< 期望的传感器总数（收集齐此数量后立即发布，0 表示使用 min_sensors）
    size_t min_sensors_;                 ///< 最小传感器数量（如果 total_sensors=0，则使用此值作为目标数量）

    // ROS 接口
    ros::Subscriber sensor_sub_;        ///< 单个传感器数据订阅者
    ros::Publisher batch_pub_;          ///< 批量数据发布者

    // 数据缓存（线程安全）
    std::mutex cache_mutex_;
    std::map<uint32_t, mag_core_msgs::MagSensorData> sensor_cache_;  ///< 传感器数据缓存（sensor_id -> data）
    ros::Time last_update_time_;        ///< 最后更新时间
};

} // namespace mag_device_sensor

