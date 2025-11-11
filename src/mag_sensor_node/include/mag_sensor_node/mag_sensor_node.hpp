#pragma once

#include <magnet_msgs/MagSensorData.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <atomic>
#include <string>
#include <thread>

class MagSerialNode
{
public:
    MagSerialNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void start();
    ~MagSerialNode();

private:
    void loadParams();
    void loadConfig();
    void openSerial();
    bool parseLine(const std::string &line, int &id, double &mx, double &my, double &mz);
    inline double rawToMilliTesla(double raw_value) const;
    void publishMeasurement(int id, double mx, double my, double mz);
    void runLoop();

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