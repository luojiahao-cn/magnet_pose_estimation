#include <mag_device_sensor/sensor_config_loader.hpp>
#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <locale>

namespace mag_device_sensor
{

    class SensorDriverNode
    {
    public:
        SensorDriverNode(ros::NodeHandle nh,
                         ros::NodeHandle pnh,
                         const mag_core_description::SensorArrayDescription &array,
                         DriverConfig driver_config,
                         TopicConfig topic_config,
                         TfConfig tf_config);

        ~SensorDriverNode();

        void start();

    private:
        void setLogLevel();
        void setupPublishers();
        void openSerial();

        bool parseLine(const std::string &line, int &id, double &x, double &y, double &z) const;
        void publishMeasurement(int id, double x_raw, double y_raw, double z_raw);
        double rawToMilliTesla(double raw) const;

        void runLoop();
        void onTfTimer(const ros::TimerEvent &);

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        mag_core_description::SensorArrayDescription array_;
        std::unique_ptr<mag_core_description::SensorArrayTfPublisher> tf_publisher_;
        ros::Timer tf_timer_;
        TfConfig tf_config_;

        DriverConfig driver_config_;
        TopicConfig topic_config_;

        serial::Serial serial_;
        std::thread worker_;
        std::atomic<bool> running_{false};

        ros::Publisher raw_pub_;
        ros::Publisher field_pub_;

        std::uint64_t message_counter_{0};
        std::chrono::steady_clock::time_point freq_window_start_;
    };

    namespace
    {
        constexpr double kDefaultRawMax = 32767.0;
    }

    SensorDriverNode::SensorDriverNode(ros::NodeHandle nh,
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

    SensorDriverNode::~SensorDriverNode()
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

    void SensorDriverNode::start()
    {
        // 打开串口并启动后台采集线程
        openSerial();
        running_.store(true);
        freq_window_start_ = std::chrono::steady_clock::now();
        worker_ = std::thread(&SensorDriverNode::runLoop, this);

        if (tf_config_.enable && tf_publisher_)
        {
            if (tf_config_.publish_rate > 0.0)
            {
                tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_config_.publish_rate), &SensorDriverNode::onTfTimer, this);
            }
            else
            {
                tf_publisher_->publishStatic();
            }
        }
    }

    void SensorDriverNode::setLogLevel()
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

    void SensorDriverNode::setupPublishers()
    {
        if (!topic_config_.raw_topic.empty())
        {
            raw_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.raw_topic, 100);
        }
        if (!topic_config_.field_topic.empty())
        {
            field_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.field_topic, 100);
        }
    }

    void SensorDriverNode::openSerial()
    {
        try
        {
            serial_.setPort(driver_config_.serial_port);
            serial_.setBaudrate(driver_config_.baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(driver_config_.timeout_ms);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (const serial::IOException &e)
        {
            throw std::runtime_error("Unable to open port " + driver_config_.serial_port + ": " + e.what());
        }

        if (serial_.isOpen())
        {
            ROS_INFO("Serial Port initialized: %s @ %d", driver_config_.serial_port.c_str(), driver_config_.baud_rate);
        }
        else
        {
            throw std::runtime_error("Failed to open port " + driver_config_.serial_port);
        }
    }

    bool SensorDriverNode::parseLine(const std::string &line, int &id, double &x, double &y, double &z) const
    {
        // 假设格式: "id,x,y,z" 或者 "id x y z"
        // 需要根据实际协议调整
        // 这里简单实现：查找逗号或空格分割
        // 示例实现，需要根据硬件协议修改
        if (line.empty())
            return false;

        // 简单的sscanf或者stringstream
        // 假设是: ID:0 X:123 Y:456 Z:789 格式
        // 或者 ID,X,Y,Z
        // 这里我们假设标准CSV: id,x,y,z

        // 替换逗号为空格
        std::string temp = line;
        std::replace(temp.begin(), temp.end(), ',', ' ');
        std::stringstream ss(temp);
        if (ss >> id >> x >> y >> z)
        {
            return true;
        }
        return false;
    }

    void SensorDriverNode::publishMeasurement(int id, double x_raw, double y_raw, double z_raw)
    {
        ros::Time now = ros::Time::now();

        // 检查是否有该传感器的定义
        if (!array_.findSensor(id))
        {
            ROS_WARN_THROTTLE(1.0, "收到未配置的传感器 ID: %d", id);
            return;
        }

        // 发布 Raw
        if (raw_pub_)
        {
            mag_core_msgs::MagSensorData msg;
            msg.header.stamp = now;
            msg.header.frame_id = array_.sensorFrameName(id);
            msg.sensor_id = id;
            msg.mag_x = x_raw;
            msg.mag_y = y_raw;
            msg.mag_z = z_raw;
            raw_pub_.publish(msg);
        }

        // 发布 Field (mT)
        if (field_pub_)
        {
            mag_core_msgs::MagSensorData msg;
            msg.header.stamp = now;
            msg.header.frame_id = array_.sensorFrameName(id);
            msg.sensor_id = id;
            msg.mag_x = rawToMilliTesla(x_raw);
            msg.mag_y = rawToMilliTesla(y_raw);
            msg.mag_z = rawToMilliTesla(z_raw);
            field_pub_.publish(msg);
        }
    }

    double SensorDriverNode::rawToMilliTesla(double raw) const
    {
        // 简单线性变换
        return (raw / driver_config_.raw_max) * driver_config_.full_scale_mT;
    }

    void SensorDriverNode::runLoop()
    {
        while (running_ && ros::ok())
        {
            if (serial_.available())
            {
                std::string line;
                // 读取一行
                size_t len = serial_.readline(line);
                if (len > 0)
                {
                    // 去除末尾换行
                    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
                    {
                        line.pop_back();
                    }

                    int id;
                    double x, y, z;
                    if (parseLine(line, id, x, y, z))
                    {
                        publishMeasurement(id, x, y, z);
                        message_counter_++;
                    }
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(driver_config_.poll_sleep_ms));
            }

            // 统计频率
            if (driver_config_.freq_stat_period > 0)
            {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - freq_window_start_).count();
                if (elapsed >= driver_config_.freq_stat_period)
                {
                    double freq = message_counter_ / elapsed;
                    ROS_INFO("Sensor update rate: %.2f Hz", freq);
                    message_counter_ = 0;
                    freq_window_start_ = now;
                }
            }
        }
    }

    void SensorDriverNode::onTfTimer(const ros::TimerEvent &)
    {
        if (tf_publisher_)
        {
            tf_publisher_->publishDynamic(ros::Time::now());
        }
    }

} // namespace mag_device_sensor

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string driver_ns = "mag_device_sensor.driver";
        const std::string array_ns = "mag_device_sensor.array";

        XmlRpc::XmlRpcValue driver_param;
        XmlRpc::XmlRpcValue array_param;

        std::size_t driver_error = 0;
        driver_error += !rps::get(driver_ns, pnh, "config", driver_param);
        rps::shutdownIfError(driver_ns, driver_error);

        std::size_t array_error = 0;
        array_error += !rps::get(array_ns, pnh, "array/config", array_param);
        rps::shutdownIfError(array_ns, array_error);

        auto bundle = mag_device_sensor::loadSensorDriverConfig(driver_param, driver_ns + ".config");
        auto array_config = mag_core_description::SensorArrayDescription::loadFromParam(array_param, array_ns + ".config");
        mag_core_description::SensorArrayDescription array;
        array.load(array_config);

        mag_device_sensor::SensorDriverNode node(nh,
                                                 pnh,
                                                 array,
                                                 bundle.driver,
                                                 bundle.topics,
                                                 bundle.tf);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_sensor 驱动节点启动失败: %s", e.what());
        return 1;
    }
    return 0;
}
