#include <mag_device_sensor/sensor_node.hpp>

#include <ros/console.h>

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace mag_device_sensor
{
namespace
{
constexpr double kDefaultRawMax = 32767.0;
}

SensorNode::SensorNode(ros::NodeHandle nh,
                       ros::NodeHandle pnh,
                       const mag_core_description::SensorArrayDescription &array,
                       DriverConfig driver_config,
                       TopicConfig topic_config,
                       TfConfig tf_config)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      array_(array),
      tf_publisher_(std::make_unique<mag_core_description::SensorArrayTfPublisher>(array_)),
      tf_config_(tf_config),
      driver_config_(std::move(driver_config)),
      topic_config_(std::move(topic_config))
{
    // 设置日志级别
    setLogLevel();
    if (driver_config_.raw_max <= 0.0)
    {
        driver_config_.raw_max = kDefaultRawMax;
    }
    // 根据配置初始化消息发布器
    setupPublishers();
}

SensorNode::~SensorNode()
{
    // 停止后台读取线程并关闭串口
    running_.store(false);
    if (worker_.joinable())
    {
        worker_.join();
    }
    if (serial_.isOpen())
    {
        serial_.close();
    }
}

void SensorNode::start()
{
    // 打开串口并启动后台采集线程
    openSerial();
    running_.store(true);
    freq_window_start_ = std::chrono::steady_clock::now();
    worker_ = std::thread(&SensorNode::runLoop, this);

    if (tf_config_.enable && tf_publisher_)
    {
        if (tf_config_.publish_rate > 0.0)
        {
            tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_config_.publish_rate), &SensorNode::onTfTimer, this);
        }
        else
        {
            tf_publisher_->publishStatic();
        }
    }
}

void SensorNode::setLogLevel()
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

void SensorNode::setupPublishers()
{
    // 根据配置创建 raw / field 话题发布器
    if (!topic_config_.raw_topic.empty())
    {
        raw_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.raw_topic, 50);
    }
    if (!topic_config_.field_topic.empty())
    {
        field_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.field_topic, 50);
    }
}

void SensorNode::openSerial()
{
    // 底层串口初始化，失败时抛出异常让上层处理
    try
    {
        serial_.setPort(driver_config_.serial_port);
        serial_.setBaudrate(static_cast<uint32_t>(driver_config_.baud_rate));
        serial::Timeout timeout = serial::Timeout::simpleTimeout(driver_config_.timeout_ms);
        serial_.setTimeout(timeout);
        serial_.open();
        ROS_INFO_STREAM("[mag_device_sensor] 串口已打开 " << driver_config_.serial_port
                                                        << ", baud=" << driver_config_.baud_rate);
    }
    catch (const serial::IOException &e)
    {
        throw std::runtime_error(std::string("打开串口失败: ") + e.what());
    }
}

bool SensorNode::parseLine(const std::string &line, int &id, double &x, double &y, double &z) const
{
    // 将串口文本解析为 id 与三轴原始数据，遇到异常格式返回 false
    std::string trimmed = line;
    while (!trimmed.empty() && std::isspace(trimmed.back()))
    {
        trimmed.pop_back();
    }
    size_t start = 0;
    while (start < trimmed.size() && std::isspace(trimmed[start]))
    {
        ++start;
    }
    if (start > 0 && start < trimmed.size())
    {
        trimmed = trimmed.substr(start);
    }
    if (trimmed.size() < 6)
    {
        return false;
    }

    size_t colon = trimmed.find(':');
    if (colon == std::string::npos)
    {
        return false;
    }
    std::string id_part = trimmed.substr(0, colon);
    if (id_part.size() < 3 || id_part.front() != '[' || id_part.back() != ']')
    {
        return false;
    }
    try
    {
        id = std::stoi(id_part.substr(1, id_part.size() - 2));
    }
    catch (...)
    {
        return false;
    }

    std::istringstream iss(trimmed.substr(colon + 1));
    if (!(iss >> x >> y >> z))
    {
        return false;
    }
    return true;
}

double SensorNode::rawToMilliTesla(double raw) const
{
    // Raw 量程按线性映射换算成 mT
    return (raw / driver_config_.raw_max) * driver_config_.full_scale_mT;
}

void SensorNode::publishMeasurement(int id, double x_raw, double y_raw, double z_raw)
{
    // 根据传感器描述发布原始量程与换算后的磁场消息
    const auto *sensor = array_.findSensor(id);
    if (!sensor)
    {
        ROS_WARN_STREAM_THROTTLE(2.0, "[mag_device_sensor] 未知传感器 id=" << id << "，已忽略");
        return;
    }

    ros::Time stamp = ros::Time::now();
    if (raw_pub_)
    {
        mag_core_msgs::MagSensorData msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = sensor->frame_id;
        msg.sensor_id = id;
        msg.mag_x = x_raw;
        msg.mag_y = y_raw;
        msg.mag_z = z_raw;
        raw_pub_.publish(msg);
    }

    if (field_pub_)
    {
        mag_core_msgs::MagSensorData msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = sensor->frame_id;
        msg.sensor_id = id;
        msg.mag_x = rawToMilliTesla(x_raw);
        msg.mag_y = rawToMilliTesla(y_raw);
        msg.mag_z = rawToMilliTesla(z_raw);
        field_pub_.publish(msg);
    }
}

void SensorNode::runLoop()
{
    // 后台线程：持续读取串口并发布消息，同时统计频率
    while (ros::ok() && running_.load())
    {
        try
        {
            if (serial_.available())
            {
                std::string line = serial_.readline();
                int id;
                double x, y, z;
                if (parseLine(line, id, x, y, z))
                {
                    publishMeasurement(id, x, y, z);
                    ++message_counter_;
                }
                else
                {
                    ROS_WARN_STREAM_THROTTLE(5.0, "[mag_device_sensor] 串口数据格式不匹配: " << line);
                }
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN_STREAM_THROTTLE(2.0, "[mag_device_sensor] 读取串口失败: " << e.what());
        }

        if (driver_config_.freq_stat_period > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - freq_window_start_).count();
            if (elapsed >= driver_config_.freq_stat_period)
            {
                double hz = message_counter_ / elapsed;
                ROS_INFO_STREAM("[mag_device_sensor] 发布频率 " << hz << " Hz (窗口 " << elapsed 
                                << "s 内 " << static_cast<unsigned long long>(message_counter_) << " 条)");
                message_counter_ = 0;
                freq_window_start_ = now;
            }
        }

    std::this_thread::sleep_for(std::chrono::milliseconds(driver_config_.poll_sleep_ms));
    }
}

void SensorNode::onTfTimer(const ros::TimerEvent &)
{
    // 定时广播阵列与传感器的 TF
    if (tf_publisher_)
    {
        tf_publisher_->publishDynamic(ros::Time::now());
    }
}

} // namespace mag_device_sensor
