#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_msgs/MagSensorBatch.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <mag_core_utils/xmlrpc_utils.hpp>
#include <mag_core_utils/logger_utils.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <locale>

namespace mag_device_sensor
{

    class SensorArrayBatcherNode
    {
    public:
        SensorArrayBatcherNode(ros::NodeHandle nh, ros::NodeHandle pnh);

        void start();

    private:
        void setLogLevel();
        void loadParameters();
        void setupSubscribers();
        void setupPublishers();
        void sensorCallback(const mag_core_msgs::MagSensorDataConstPtr &msg);
        bool publishBatch();

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        // 配置参数
        std::string input_topic_;
        std::string output_topic_;
        std::string output_frame_;
        double timeout_seconds_;
        size_t total_sensors_;
        size_t min_sensors_;

        // ROS 接口
        ros::Subscriber sensor_sub_;
        ros::Publisher batch_pub_;

        // 数据缓存（线程安全）
        std::mutex cache_mutex_;
        std::map<uint32_t, mag_core_msgs::MagSensorData> sensor_cache_;
        ros::Time last_update_time_;
    };

    SensorArrayBatcherNode::SensorArrayBatcherNode(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)), pnh_(std::move(pnh))
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

    void SensorArrayBatcherNode::start()
    {
        // 只是为了占位，实际逻辑在构造函数启动
    }

    void SensorArrayBatcherNode::setLogLevel()
    {
        std::string log_level_str = "INFO";
        pnh_.param("logging_level", log_level_str, log_level_str);

        // 转换为大写
        std::transform(log_level_str.begin(), log_level_str.end(), log_level_str.begin(), ::toupper);

        ros::console::Level level = ros::console::levels::Info;
        if (log_level_str == "DEBUG")
        {
            level = ros::console::levels::Debug;
        }
        else if (log_level_str == "INFO")
        {
            level = ros::console::levels::Info;
        }
        else if (log_level_str == "WARN")
        {
            level = ros::console::levels::Warn;
        }
        else if (log_level_str == "ERROR")
        {
            level = ros::console::levels::Error;
        }
        else if (log_level_str == "FATAL")
        {
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
        if (params.hasMember("total_sensors"))
        {
            total_sensors_ = static_cast<size_t>(xml::readNumber(
                xml::requireMember(params, "total_sensors", params_ctx),
                params_ctx + "/total_sensors"));
        }
        else
        {
            total_sensors_ = 0; // 默认使用 min_sensors
        }

        // 最小传感器数量（如果 total_sensors=0，则使用此值作为目标数量）
        if (params.hasMember("min_sensors"))
        {
            min_sensors_ = static_cast<size_t>(xml::readNumber(
                xml::requireMember(params, "min_sensors", params_ctx),
                params_ctx + "/min_sensors"));
        }
        else
        {
            min_sensors_ = 0; // 默认不限制
        }

        // 确定目标传感器数量
        size_t target_sensors = (total_sensors_ > 0) ? total_sensors_ : min_sensors_;
        ROS_INFO("Target sensor count: %zu (total: %zu, min: %zu)", target_sensors, total_sensors_, min_sensors_);
    }

    void SensorArrayBatcherNode::setupSubscribers()
    {
        sensor_sub_ = nh_.subscribe(input_topic_, 1000, &SensorArrayBatcherNode::sensorCallback, this);
    }

    void SensorArrayBatcherNode::setupPublishers()
    {
        batch_pub_ = nh_.advertise<mag_core_msgs::MagSensorBatch>(output_topic_, 100);
    }

    void SensorArrayBatcherNode::sensorCallback(const mag_core_msgs::MagSensorDataConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);

        // 检查超时，如果距离上次清理/发布时间太久，则清空
        if ((ros::Time::now() - last_update_time_).toSec() > timeout_seconds_)
        {
            if (!sensor_cache_.empty())
            {
                ROS_WARN_THROTTLE(1.0, "Sensor data timed out, clearing cache (size was %zu)", sensor_cache_.size());
                sensor_cache_.clear();
            }
        }
        last_update_time_ = ros::Time::now();

        // 存入缓存
        sensor_cache_[msg->sensor_id] = *msg;

        // 检查是否满足发布条件
        if (total_sensors_ > 0)
        {
            if (sensor_cache_.size() >= total_sensors_)
            {
                publishBatch();
            }
        }
        else if (min_sensors_ > 0)
        {
            // 如果只设置了 min_sensors，我们可能无法知道什么时候收集"齐"了
            // 这里假设如果达到 min_sensors 就发布，但这样可能会频繁发布部分数据
            // 通常建议使用 timer 或者 frame synchronization
            // 但这里我们简单实现：如果数量增加，且还没发布过...
            // 实际上单靠 min_sensors 很难触发 "Group Complete" 事件，
            // 除非我们知道总共只有多少个传感器。
            // 所以建议配置 total_sensors。

            // 此处为了兼容，如果设置了 min_sensors 且 cache 数量达到 min_sensors，也发布
            // 注意：这会导致第一个到达 min_sensors 的消息触发发布，后续的可能会触发新的不完整发布
            // 所以最好是配合 timeout 或者确定的 total_sensors。
            // 暂时实现为：达到 min_sensors 立即发布并清空
            if (sensor_cache_.size() >= min_sensors_)
            {
                publishBatch();
            }
        }
    }

    bool SensorArrayBatcherNode::publishBatch()
    {
        mag_core_msgs::MagSensorBatch batch_msg;
        batch_msg.header.stamp = last_update_time_; // 使用最新数据的时间
        batch_msg.header.frame_id = output_frame_;

        batch_msg.measurements.reserve(sensor_cache_.size());
        for (const auto &pair : sensor_cache_)
        {
            batch_msg.measurements.push_back(pair.second);
        }

        batch_pub_.publish(batch_msg);
        sensor_cache_.clear();
        return true;
    }

} // namespace mag_device_sensor

int main(int argc, char **argv)
{
    // 设置本地化，支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");

    ros::init(argc, argv, "sensor_array_batcher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    mag_device_sensor::SensorArrayBatcherNode node(nh, pnh);
    node.start();

    ros::spin();

    return 0;
}
