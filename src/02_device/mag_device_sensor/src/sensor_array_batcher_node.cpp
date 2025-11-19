#include <mag_device_sensor/sensor_array_batcher_node.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <ros/ros.h>

namespace mag_device_sensor
{

SensorArrayBatcherNode::SensorArrayBatcherNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
{
    loadParameters();
    setupSubscribers();
    setupPublishers();
}

void SensorArrayBatcherNode::loadParameters()
{
    namespace xml = mag_core_utils::xmlrpc;

    const auto &node = pnh_.param("config", XmlRpc::XmlRpcValue());
    const std::string context = "~config";

    // 解析话题配置
    const auto topics_ctx = xml::makeContext(context, "topics");
    const auto &topics = xml::requireStructField(node, "topics", context);
    input_topic_ = xml::requireStringField(topics, "input", topics_ctx);
    output_topic_ = xml::requireStringField(topics, "output", topics_ctx);

    // 解析坐标系配置
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(node, "frames", context);
    output_frame_ = xml::requireStringField(frames, "output", frames_ctx);

    // 解析发布参数
    const auto params_ctx = xml::makeContext(context, "params");
    const auto &params = xml::requireStructField(node, "params", context);
    timeout_seconds_ = xml::readNumber(
        xml::requireMember(params, "timeout_seconds", params_ctx),
        params_ctx + "/timeout_seconds");
    
    // 期望的传感器总数（可选，如果设置则收集齐此数量后立即发布）
    if (params.hasMember("total_sensors")) {
        total_sensors_ = static_cast<size_t>(xml::readNumber(
            xml::requireMember(params, "total_sensors", params_ctx),
            params_ctx + "/total_sensors"));
    } else {
        total_sensors_ = 0;  // 默认使用 min_sensors
    }
    
    // 最小传感器数量（如果 total_sensors=0，则使用此值作为目标数量）
    if (params.hasMember("min_sensors")) {
        min_sensors_ = static_cast<size_t>(xml::readNumber(
            xml::requireMember(params, "min_sensors", params_ctx),
            params_ctx + "/min_sensors"));
    } else {
        min_sensors_ = 0;  // 默认不限制
    }

    // 确定目标传感器数量
    size_t target_sensors = (total_sensors_ > 0) ? total_sensors_ : min_sensors_;

    ROS_INFO("[sensor_array_batcher] 配置加载完成:");
    ROS_INFO("  输入话题: %s", input_topic_.c_str());
    ROS_INFO("  输出话题: %s", output_topic_.c_str());
    ROS_INFO("  参考坐标系: %s (用于批量消息 header.frame_id)", output_frame_.c_str());
    ROS_INFO("  超时时间: %.3f 秒", timeout_seconds_);
    if (total_sensors_ > 0) {
        ROS_INFO("  期望传感器总数: %zu (收集齐后立即发布)", total_sensors_);
    } else if (min_sensors_ > 0) {
        ROS_INFO("  目标传感器数量: %zu (收集齐后立即发布)", min_sensors_);
    } else {
        ROS_WARN("  未设置 total_sensors 或 min_sensors，将收集到任何数据就发布（不推荐）");
    }
}

void SensorArrayBatcherNode::setupSubscribers()
{
    sensor_sub_ = nh_.subscribe(
        input_topic_,
        100,  // 队列大小：足够大以处理多个传感器的高频数据
        &SensorArrayBatcherNode::sensorCallback,
        this);
    ROS_INFO("[sensor_array_batcher] 已订阅: %s", input_topic_.c_str());
}

void SensorArrayBatcherNode::setupPublishers()
{
    batch_pub_ = nh_.advertise<mag_core_msgs::MagSensorBatch>(output_topic_, 10);
    ROS_INFO("[sensor_array_batcher] 已发布: %s", output_topic_.c_str());
}

void SensorArrayBatcherNode::sensorCallback(const mag_core_msgs::MagSensorDataConstPtr &msg)
{
    mag_core_msgs::MagSensorBatch batch_msg;
    bool should_publish = false;
    
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        
        // 更新缓存
        sensor_cache_[msg->sensor_id] = *msg;
        last_update_time_ = ros::Time::now();
        
        // 确定目标传感器数量
        size_t target_sensors = (total_sensors_ > 0) ? total_sensors_ : min_sensors_;
        
        // 如果收集齐了目标数量的传感器，准备发布
        if (target_sensors > 0 && sensor_cache_.size() >= target_sensors) {
            // 构建批量消息（在锁内，避免数据竞争）
            batch_msg.measurements.reserve(sensor_cache_.size());
            ros::Time latest_stamp = ros::Time(0);
            
            for (const auto &kv : sensor_cache_) {
                const auto &sensor_data = kv.second;
                batch_msg.measurements.push_back(sensor_data);
                if (sensor_data.header.stamp > latest_stamp) {
                    latest_stamp = sensor_data.header.stamp;
                }
            }
            
            batch_msg.header.stamp = latest_stamp.isZero() ? ros::Time::now() : latest_stamp;
            batch_msg.header.frame_id = output_frame_;
            
            // 清空缓存，准备收集下一组数据
            sensor_cache_.clear();
            should_publish = true;
        }
    }  // 锁在这里自动释放
    
    // 在锁外发布（避免在锁内进行网络操作）
    if (should_publish) {
        batch_pub_.publish(batch_msg);
        ROS_DEBUG("[sensor_array_batcher] 发布批量数据: %zu 个传感器", batch_msg.measurements.size());
    }
}

bool SensorArrayBatcherNode::publishBatch()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // 检查是否有数据
    if (sensor_cache_.empty()) {
        return false;
    }

    // 检查数据是否超时
    ros::Time now = ros::Time::now();
    if ((now - last_update_time_).toSec() > timeout_seconds_) {
        ROS_WARN_THROTTLE(2.0, "[sensor_array_batcher] 传感器数据超时，清空缓存");
        sensor_cache_.clear();
        return false;
    }

    // 构建批量消息（只收集数据，不转换坐标系）
    mag_core_msgs::MagSensorBatch batch_msg;
    batch_msg.measurements.reserve(sensor_cache_.size());
    ros::Time latest_stamp = ros::Time(0);

    // 收集所有传感器数据
    for (const auto &kv : sensor_cache_) {
        const auto &sensor_data = kv.second;
        batch_msg.measurements.push_back(sensor_data);
        
        // 更新最新时间戳
        if (sensor_data.header.stamp > latest_stamp) {
            latest_stamp = sensor_data.header.stamp;
        }
    }

    // 设置批量消息的 header（使用最新时间戳和传感器阵列坐标系）
    batch_msg.header.stamp = latest_stamp.isZero() ? now : latest_stamp;
    batch_msg.header.frame_id = output_frame_;  // 参考坐标系（订阅者用于查询传感器位置）

    // 确定目标传感器数量
    size_t target_sensors = (total_sensors_ > 0) ? total_sensors_ : min_sensors_;
    
    // 检查传感器数量
    if (target_sensors > 0 && batch_msg.measurements.size() < target_sensors) {
        ROS_DEBUG_THROTTLE(1.0, "[sensor_array_batcher] 传感器数量不足: %zu < %zu，跳过发布",
                          batch_msg.measurements.size(), target_sensors);
        return false;
    }

    // 发布批量数据
    if (!batch_msg.measurements.empty()) {
        batch_pub_.publish(batch_msg);
        ROS_DEBUG("[sensor_array_batcher] 发布批量数据: %zu 个传感器", batch_msg.measurements.size());
        return true;
    }
    
    return false;
}

void SensorArrayBatcherNode::start()
{
    ROS_INFO("[sensor_array_batcher] 节点已启动，等待传感器数据...");
    ros::spin();
}

} // namespace mag_device_sensor

