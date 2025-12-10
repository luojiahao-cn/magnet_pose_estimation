#include <mag_device_sensor/sensor_array_batcher_node.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>
#include <mag_core_utils/logger_utils.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <algorithm>

namespace mag_device_sensor
{

SensorArrayBatcherNode::SensorArrayBatcherNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
{
    // 设置日志级别
    setLogLevel();
    loadParameters();
    setupSubscribers();
    setupPublishers();
    
    // 统一输出初始化信息
    namespace logger = mag_core_utils::logger;
    std::vector<std::pair<std::string, std::string>> init_items;
    init_items.emplace_back("已订阅话题", input_topic_);
    init_items.emplace_back("已发布话题", output_topic_);
    ROS_INFO_STREAM("[sensor_array_batcher] " << logger::formatInit(init_items));
}

void SensorArrayBatcherNode::setLogLevel()
{
    std::string log_level_str = "INFO";
    pnh_.param("logging_level", log_level_str, log_level_str);
    
    // 转换为大写
    std::transform(log_level_str.begin(), log_level_str.end(), log_level_str.begin(), ::toupper);
    
    ros::console::Level level = ros::console::levels::Info;
    if (log_level_str == "DEBUG") {
        level = ros::console::levels::Debug;
    } else if (log_level_str == "INFO") {
        level = ros::console::levels::Info;
    } else if (log_level_str == "WARN") {
        level = ros::console::levels::Warn;
    } else if (log_level_str == "ERROR") {
        level = ros::console::levels::Error;
    } else if (log_level_str == "FATAL") {
        level = ros::console::levels::Fatal;
    }
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level);
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

    // 使用统一的日志格式化工具
    namespace logger = mag_core_utils::logger;
    std::vector<std::pair<std::string, std::string>> config_items;
    config_items.emplace_back("输入话题", input_topic_);
    config_items.emplace_back("输出话题", output_topic_);
    config_items.emplace_back("参考坐标系", output_frame_);
    config_items.emplace_back("超时时间", logger::formatTime(timeout_seconds_));
    
    if (total_sensors_ > 0) {
        config_items.emplace_back("期望传感器总数", std::to_string(total_sensors_) + " (收集齐后立即发布)");
    } else if (min_sensors_ > 0) {
        config_items.emplace_back("目标传感器数量", std::to_string(min_sensors_) + " (收集齐后立即发布)");
    } else {
        config_items.emplace_back("传感器数量", "未设置（将收集到任何数据就发布，不推荐）");
    }

    ROS_INFO_STREAM("[sensor_array_batcher] " << logger::formatConfig(config_items));
    
    if (total_sensors_ == 0 && min_sensors_ == 0) {
        ROS_WARN_STREAM("[sensor_array_batcher] 未设置 total_sensors 或 min_sensors，将收集到任何数据就发布（不推荐）");
    }
}

void SensorArrayBatcherNode::setupSubscribers()
{
    sensor_sub_ = nh_.subscribe(
        input_topic_,
        100,  // 队列大小：足够大以处理多个传感器的高频数据
        &SensorArrayBatcherNode::sensorCallback,
        this);
}

void SensorArrayBatcherNode::setupPublishers()
{
    batch_pub_ = nh_.advertise<mag_core_msgs::MagSensorBatch>(output_topic_, 10);
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
        ROS_DEBUG_STREAM("[sensor_array_batcher] ✓ 发布批量数据: " << batch_msg.measurements.size() << " 个传感器");
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
        ROS_WARN_STREAM_THROTTLE(2.0, "[sensor_array_batcher] ✗ 传感器数据超时，清空缓存");
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
        ROS_DEBUG_STREAM_THROTTLE(1.0, "[sensor_array_batcher] 传感器数量不足 (当前: " 
                                  << batch_msg.measurements.size() << ", 需要: " << target_sensors << ")，跳过发布");
        return false;
    }

    // 发布批量数据
    if (!batch_msg.measurements.empty()) {
        batch_pub_.publish(batch_msg);
        ROS_DEBUG_STREAM("[sensor_array_batcher] ✓ 发布批量数据: " << batch_msg.measurements.size() << " 个传感器");
        return true;
    }
    
    return false;
}

void SensorArrayBatcherNode::start()
{
    ROS_INFO_STREAM("[sensor_array_batcher] 节点已启动，等待传感器数据...");
    ros::spin();
}

} // namespace mag_device_sensor

