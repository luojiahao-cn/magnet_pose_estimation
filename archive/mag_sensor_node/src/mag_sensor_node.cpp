#include <mag_sensor_node/array_description.hpp>

#include <magnet_msgs/MagSensorData.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <atomic>
#include <chrono>
#include <cctype>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
    // 以 16bit 有符号整形量程作为默认基准
    constexpr double kDefaultRawMax = 32767.0;
}

class MagSensorDriverNode
{
public:
    MagSensorDriverNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~MagSensorDriverNode();

    void start();

private:
    void loadArrayDescription();
    void loadParameters();
    void setupPublishers();
    void openSerial();

    bool parseLine(const std::string &line, int &id, double &x, double &y, double &z) const;
    void publishMeasurement(int id, double x_raw, double y_raw, double z_raw);
    double rawToMilliTesla(double raw) const;

    void runLoop();
    void onTfTimer(const ros::TimerEvent &);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    mag_sensor_node::SensorArrayDescription array_description_;
    std::unique_ptr<mag_sensor_node::SensorArrayTfPublisher> tf_publisher_;
    ros::Timer tf_timer_;
    bool publish_tf_{true};
    double tf_publish_rate_{30.0};

    serial::Serial serial_;
    std::thread worker_;
    std::atomic<bool> running_{false};

    ros::Publisher raw_pub_;
    ros::Publisher field_pub_;
    std::string topic_raw_;
    std::string topic_field_;

    std::string serial_port_;
    int baud_rate_{0};
    int timeout_ms_{0};
    int poll_sleep_ms_{1};

    double full_scale_mT_{3.2};
    double raw_max_{kDefaultRawMax};

    double freq_stat_period_{20.0};
    std::uint64_t freq_counter_{0};
    std::chrono::steady_clock::time_point freq_window_start_;
};

MagSensorDriverNode::MagSensorDriverNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh))
{
    loadArrayDescription();
    loadParameters();
    setupPublishers();
    openSerial();
}

MagSensorDriverNode::~MagSensorDriverNode()
{
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

void MagSensorDriverNode::start()
{
    running_.store(true);
    freq_window_start_ = std::chrono::steady_clock::now();
    worker_ = std::thread(&MagSensorDriverNode::runLoop, this);

    if (publish_tf_ && tf_publisher_)
    {
        if (tf_publish_rate_ > 0.0)
        {
            ros::Duration period(1.0 / tf_publish_rate_);
            tf_timer_ = nh_.createTimer(period, &MagSensorDriverNode::onTfTimer, this);
        }
        else
        {
            // publish_rate <= 0 视为静态 TF
            tf_publisher_->publishStatic();
        }
    }
}

void MagSensorDriverNode::loadArrayDescription()
{
    // 统一读取阵列结构，保证实物与仿真共用一份几何描述
    array_description_.load(pnh_, "array");
    tf_publish_rate_ = array_description_.tfPublishRate();
    tf_publisher_ = std::make_unique<mag_sensor_node::SensorArrayTfPublisher>(array_description_);
}

void MagSensorDriverNode::loadParameters()
{
    if (!pnh_.getParam("driver/serial_port", serial_port_))
    {
        throw std::runtime_error("缺少 driver/serial_port 串口配置");
    }
    if (!pnh_.getParam("driver/baud_rate", baud_rate_))
    {
        throw std::runtime_error("缺少 driver/baud_rate 波特率配置");
    }
    if (!pnh_.getParam("driver/timeout_ms", timeout_ms_))
    {
        throw std::runtime_error("缺少 driver/timeout_ms 串口超时配置");
    }
    pnh_.param("driver/poll_sleep_ms", poll_sleep_ms_, 1);

    pnh_.param("topics/raw", topic_raw_, std::string("/mag_sensor/data_raw"));
    pnh_.param("topics/field", topic_field_, std::string("/mag_sensor/data_mT"));
    if (topic_raw_.empty() && topic_field_.empty())
    {
        throw std::runtime_error("至少需要提供 topics/raw 或 topics/field 中的一个话题名");
    }

    pnh_.param("calibration/full_scale_mT", full_scale_mT_, 3.2);
    pnh_.param("calibration/raw_max", raw_max_, kDefaultRawMax);
    if (full_scale_mT_ <= 0.0 || raw_max_ <= 0.0)
    {
        throw std::runtime_error("calibration/full_scale_mT 与 calibration/raw_max 需为正数");
    }

    pnh_.param("driver/freq_stat_period", freq_stat_period_, 20.0);
    pnh_.param("tf/enable", publish_tf_, true);
    if (pnh_.hasParam("tf/publish_rate"))
    {
        pnh_.param("tf/publish_rate", tf_publish_rate_, tf_publish_rate_);
    }
}

void MagSensorDriverNode::setupPublishers()
{
    if (!topic_raw_.empty())
    {
        raw_pub_ = nh_.advertise<magnet_msgs::MagSensorData>(topic_raw_, 50);
    }
    if (!topic_field_.empty())
    {
        field_pub_ = nh_.advertise<magnet_msgs::MagSensorData>(topic_field_, 50);
    }
}

void MagSensorDriverNode::openSerial()
{
    try
    {
        serial_.setPort(serial_port_);
        serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms_);
        serial_.setTimeout(timeout);
        serial_.open();
        ROS_INFO_STREAM("[mag_sensor_node] 串口已打开 " << serial_port_ << ", baud=" << baud_rate_);
    }
    catch (const serial::IOException &e)
    {
        throw std::runtime_error(std::string("打开串口失败: ") + e.what());
    }
}

bool MagSensorDriverNode::parseLine(const std::string &line, int &id, double &x, double &y, double &z) const
{
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

double MagSensorDriverNode::rawToMilliTesla(double raw) const
{
    return (raw / raw_max_) * full_scale_mT_;
}

void MagSensorDriverNode::publishMeasurement(int id, double x_raw, double y_raw, double z_raw)
{
    const auto *sensor = array_description_.findSensor(id);
    if (!sensor)
    {
        ROS_WARN_THROTTLE(2.0, "[mag_sensor_node] 未知传感器 id=%d，已忽略", id);
        return;
    }

    ros::Time stamp = ros::Time::now();
    if (raw_pub_)
    {
        magnet_msgs::MagSensorData msg;
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
        magnet_msgs::MagSensorData msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = sensor->frame_id;
        msg.sensor_id = id;
        msg.mag_x = rawToMilliTesla(x_raw);
        msg.mag_y = rawToMilliTesla(y_raw);
        msg.mag_z = rawToMilliTesla(z_raw);
        field_pub_.publish(msg);
    }
}

void MagSensorDriverNode::runLoop()
{
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
                    ++freq_counter_;
                }
                else
                {
                    ROS_WARN_THROTTLE(5.0, "[mag_sensor_node] 串口数据格式不匹配: %s", line.c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(2.0, "[mag_sensor_node] 读取串口失败: %s", e.what());
        }

        if (freq_stat_period_ > 0.0)
        {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - freq_window_start_).count();
            if (elapsed >= freq_stat_period_)
            {
                double hz = freq_counter_ / elapsed;
                ROS_INFO("[mag_sensor_node] 发布频率 %.2f Hz (窗口 %.1fs 内 %llu 条)", hz, elapsed, static_cast<unsigned long long>(freq_counter_));
                freq_counter_ = 0;
                freq_window_start_ = now;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_sleep_ms_));
    }
}

void MagSensorDriverNode::onTfTimer(const ros::TimerEvent &)
{
    if (tf_publisher_)
    {
        tf_publisher_->publishDynamic(ros::Time::now());
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_sensor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try
    {
        MagSensorDriverNode node(nh, pnh);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_sensor_node 启动失败: %s", e.what());
        return 1;
    }
    return 0;
}
