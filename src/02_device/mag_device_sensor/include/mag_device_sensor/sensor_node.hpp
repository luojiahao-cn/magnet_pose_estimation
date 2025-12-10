#pragma once

#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_msgs/MagSensorData.h>

#include <ros/ros.h>

#include <serial/serial.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mag_device_sensor
{

struct DriverConfig
{
    std::string serial_port;
    int baud_rate{0};
    int timeout_ms{0};
    int poll_sleep_ms{1};

    double full_scale_mT{3.2};
    double raw_max{32767.0};

    double freq_stat_period{20.0};
};

struct TopicConfig
{
    std::string raw_topic{"/mag_sensor/data_raw"};
    std::string field_topic{"/mag_sensor/data_mT"};
};

struct TfConfig
{
    bool enable{true};
    double publish_rate{0.0};
};

class SensorNode
{
public:
    SensorNode(ros::NodeHandle nh,
               ros::NodeHandle pnh,
               const mag_core_description::SensorArrayDescription &array,
               DriverConfig driver_config,
               TopicConfig topic_config,
               TfConfig tf_config);

    ~SensorNode();

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

} // namespace mag_device_sensor
