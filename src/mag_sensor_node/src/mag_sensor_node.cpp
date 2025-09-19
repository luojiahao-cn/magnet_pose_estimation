#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <sstream>
#include <chrono>
#include <mag_sensor_node/mag_sensor_data.h>
#include <mag_sensor_node/sensor_config.hpp>

class MagSerialNode
{
public:
    MagSerialNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
    {
        loadParams();
        loadConfig();
        openSerial();
        mT_topic_ = topic_ + "_mT";
        pub_raw_ = nh_.advertise<mag_sensor_node::mag_sensor_data>(topic_, 100);
        pub_mT_ = nh_.advertise<mag_sensor_node::mag_sensor_data>(mT_topic_, 100);
    }
    void start() { worker_ = std::thread(&MagSerialNode::runLoop, this); }
    ~MagSerialNode()
    {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
    }

private:
    void loadParams()
    {
        auto require = [&](const std::string &key, auto &var)
        {
            if (!pnh_.getParam(key, var))
                throw std::runtime_error("缺少必需参数: ~" + key);
        };
        require("port", port_);
        require("baud_rate", baud_rate_);
        require("timeout", timeout_ms_);
        require("topic", topic_);
        require("frame_id", frame_id_);
        require("sleep_ms", sleep_ms_);
        require("freq_stat_period", freq_stat_period_);
    }
    void loadConfig()
    {
        ros::NodeHandle global_nh;
        if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(global_nh))
        {
            throw std::runtime_error("load sensor config failed");
        }
    }
    void openSerial()
    {
        try
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms_);
            serial_.setTimeout(to);
            serial_.open();
            ROS_INFO("[mag_sensor_node] 串口已打开 %s", port_.c_str());
        }
        catch (const serial::IOException &e)
        {
            ROS_FATAL("[mag_sensor_node] 打开串口失败 %s: %s", port_.c_str(), e.what());
            throw;
        }
    }
    bool parseLine(const std::string &line, int &id, double &mx, double &my, double &mz)
    {
        // 仅支持格式: [NN]: v1 v2 v3  （例如: [01]: 001761 -03264 -02251）
        std::string trimmed = line;
        while (!trimmed.empty() && (trimmed.back() == '\n' || trimmed.back() == '\r' || isspace(trimmed.back())))
            trimmed.pop_back();
        size_t start = 0;
        while (start < trimmed.size() && isspace(trimmed[start]))
            ++start;
        if (start > 0)
            trimmed = trimmed.substr(start);
        if (trimmed.size() < 6)
            return false; // 最小形态 [0]: 0 0 0
        size_t colon = trimmed.find(':');
        if (colon == std::string::npos)
            return false;
        std::string id_part = trimmed.substr(0, colon);
        if (id_part.size() < 3 || id_part.front() != '[' || id_part.back() != ']')
            return false;
        try
        {
            id = std::stoi(id_part.substr(1, id_part.size() - 2));
        }
        catch (...)
        {
            return false;
        }
        std::string values = trimmed.substr(colon + 1);
        std::istringstream iss(values);
        if (!(iss >> mx >> my >> mz))
            return false;
        return true;
    }
    inline double rawToMilliTesla(double raw_value) const
    {
        // 假设 raw_value 对应 int16 范围缩放，这里输入已被读为数值（可能仍是整数字符串转 double）
        return (raw_value / 32767.0) * 3.2; // 输出 mT
    }
    void publishMeasurement(int id, double mx, double my, double mz)
    {
        mag_sensor_node::SensorInfo info;
        if (!mag_sensor_node::SensorConfig::getInstance().getSensorById(id, info))
            return; // 未知 ID 直接忽略
        ros::Time stamp = ros::Time::now();
        // 原始数据发布
        mag_sensor_node::mag_sensor_data msg_raw;
        msg_raw.header.stamp = stamp;
        msg_raw.header.frame_id = frame_id_;
        msg_raw.sensor_id = id;
        msg_raw.mag_x = mx;
        msg_raw.mag_y = my;
        msg_raw.mag_z = mz;
        msg_raw.sensor_pose = info.pose;
        pub_raw_.publish(msg_raw);

        // mT 数据发布：复用同一消息类型，mag_* 字段写转换值
        mag_sensor_node::mag_sensor_data msg_mT = msg_raw;
        msg_mT.mag_x = rawToMilliTesla(mx);
        msg_mT.mag_y = rawToMilliTesla(my);
        msg_mT.mag_z = rawToMilliTesla(mz);
        pub_mT_.publish(msg_mT);
    }
    void runLoop()
    {
        std::uint64_t count = 0;
        auto period_start = std::chrono::steady_clock::now();
        while (ros::ok() && running_)
        {
            if (serial_.available())
            {
                try
                {
                    std::string line = serial_.readline();
                    if (!line.empty())
                    {
                        int id;
                        double mx, my, mz;
                        if (parseLine(line, id, mx, my, mz))
                        {
                            publishMeasurement(id, mx, my, mz);
                            ++count;
                        }
                        else
                        {
                            ROS_WARN_THROTTLE(2.0, "[mag_sensor_node] 行解析失败，格式不匹配");
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    ROS_WARN_THROTTLE(5.0, "[mag_sensor_node] 读取/发布异常: %s", e.what());
                }
            }
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - period_start).count();
            if (elapsed >= freq_stat_period_)
            {
                if (elapsed > 0)
                {
                    double hz = static_cast<double>(count) / elapsed;
                    ROS_INFO("[mag_sensor_node] 发布频率约 %.2f Hz (计数=%llu, 窗口=%.1fs)", hz, static_cast<unsigned long long>(count), elapsed);
                }
                count = 0;
                period_start = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));
        }
    }

    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_raw_;
    ros::Publisher pub_mT_;
    std::string mT_topic_;
    serial::Serial serial_;
    std::thread worker_;
    std::atomic<bool> running_{true};
    std::string port_, topic_, frame_id_;
    int baud_rate_{0}, timeout_ms_{0}, sleep_ms_{1};
    double freq_stat_period_{5.0};
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_sensor_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try
    {
        MagSerialNode node(nh, pnh);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_sensor_node 退出: %s", e.what());
        return 1;
    }
    return 0;
}
