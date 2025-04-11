#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <mutex>
#include <sstream>
#include "magnetic_pose_estimation/MagneticField.h"
#include "magnetic_pose_estimation/sensor_config.hpp"

class SerialHandler
{
public:
    SerialHandler(ros::NodeHandle &nh) : nh_(nh)
    {
        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh))
        {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 从参数服务器读取串口配置
        std::string port, topic, frame_id;
        int baud_rate, timeout, queue_size, sleep_time;
        
        // 读取所有配置参数
        nh.param<std::string>("serial/port", port, "/dev/ttyUSB0");
        nh.param<int>("serial/baud_rate", baud_rate, 921600);
        nh.param<int>("serial/timeout", timeout, 1000);
        nh.param<std::string>("serial/topic", topic, "/magnetic_field/real");
        nh.param<int>("serial/queue_size", queue_size, 100);
        nh.param<std::string>("serial/frame_id", frame_id, "world");
        nh.param<int>("serial/sleep_time", sleep_time, 1);

        sleep_time_ = sleep_time;
        frame_id_ = frame_id;

        try
        {
            serial_.setPort(port);
            serial_.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
            serial_.setTimeout(to);
            serial_.open();
            ROS_INFO("成功连接到串口 %s", port.c_str());
        }
        catch (const serial::IOException &e)
        {
            ROS_ERROR("无法打开串口 %s: %s", port.c_str(), e.what());
            throw;
        }

        // 初始化发布器
        magnetic_field_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>(
            topic, queue_size);
    }

    void startReading()
    {
        read_thread_ = std::thread(&SerialHandler::readAndProcessData, this);
        read_thread_.detach();
    }

private:
    std::pair<int, std::vector<double>> parseSerialLine(const std::string &line)
    {
        std::istringstream iss(line);
        std::string part;
        std::getline(iss, part, ':');

        int sensor_id = std::stoi(part.substr(1));
        std::vector<double> magnetic_field(3);

        for (double &value : magnetic_field)
        {
            iss >> value;
        }

        return {sensor_id, magnetic_field};
    }

    void readAndProcessData()
    {
        while (ros::ok())
        {
            if (serial_.available())
            {
                try
                {
                    std::string line = serial_.readline();
                    if (!line.empty())
                    {
                        auto [sensor_id, magnetic_field] = parseSerialLine(line);

                        magnetic_pose_estimation::SensorInfo sensor_info;
                        if (magnetic_pose_estimation::SensorConfig::getInstance().getSensorById(sensor_id, sensor_info))
                        {
                            magnetic_pose_estimation::MagneticField msg;
                            msg.header.stamp = ros::Time::now();
                            msg.header.frame_id = frame_id_;
                            msg.sensor_id = sensor_id;
                            msg.mag_x = magnetic_field[0];
                            msg.mag_y = magnetic_field[1];
                            msg.mag_z = magnetic_field[2];
                            msg.sensor_pose = sensor_info.pose;

                            magnetic_field_pub_.publish(msg);
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("数据处理错误: %s", e.what());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_));
        }
    }

    ros::NodeHandle &nh_;
    ros::Publisher magnetic_field_pub_;
    serial::Serial serial_;
    std::thread read_thread_;
    std::mutex mutex_;
    std::string frame_id_;
    int sleep_time_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "sensor_reader");
    ros::NodeHandle nh;

    try
    {
        SerialHandler handler(nh);
        handler.startReading();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("发生异常: %s", e.what());
        return 1;
    }

    return 0;
}