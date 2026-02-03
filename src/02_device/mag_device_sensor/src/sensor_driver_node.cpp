#include <ros/ros.h>
#include <serial/serial.h>
#include <mag_core_msgs/MagSensorArray.h>
#include <string>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>

class SensorDriverNode
{
public:
    SensorDriverNode(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(nh), pnh_(pnh)
    {
        // 加载参数
        pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        pnh_.param<int>("baud", baud_, 921600);
        pnh_.param<double>("full_scale_mT", full_scale_mT_, 3.2);
        pnh_.param<double>("raw_max", raw_max_, 32767.0);

        std::string raw_topic, field_topic;
        pnh_.param<std::string>("raw_topic", raw_topic, "data_raw");
        pnh_.param<std::string>("field_topic", field_topic, "data_mT");

        // 初始化发布者
        raw_pub_ = pnh_.advertise<mag_core_msgs::MagSensorArray>(raw_topic, 100);
        field_pub_ = pnh_.advertise<mag_core_msgs::MagSensorArray>(field_topic, 100);



        try
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port: " << port_ << ". Error: " << e.what());
            ros::shutdown();
            return;
        }

        if (serial_.isOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized: " << port_ << " @ " << baud_);
            running_ = true;
            worker_ = std::thread(&SensorDriverNode::runLoop, this);
        }
    }

    ~SensorDriverNode()
    {
        running_ = false;
        if (worker_.joinable())
            worker_.join();
        if (serial_.isOpen())
            serial_.close();
    }

private:
    void runLoop()
    {
        ros::Time last_data_time = ros::Time::now();

        while (ros::ok() && running_)
        {
            if (serial_.available())
            {
                std::string line;
                size_t len = serial_.readline(line);
                if (len > 0)
                {
                    while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
                    {
                        line.pop_back();
                    }

                    if (!line.empty()) {
                        processLine(line);
                        last_data_time = ros::Time::now();
                    }
                }
            }
            else
            {
                if ((ros::Time::now() - last_data_time).toSec() > 5.0) {
                   ROS_WARN_THROTTLE(5.0, "No data received from sensor for 5 seconds...");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    void processLine(const std::string &line)
    {
        std::stringstream ss(line);
        double x, y, z;
        uint32_t id = 1;

        mag_core_msgs::MagSensorArray raw_msg;
        mag_core_msgs::MagSensorArray field_msg;

        ros::Time now = ros::Time::now();
        raw_msg.header.stamp = now;
        raw_msg.header.frame_id = "sensor_array";
        field_msg.header = raw_msg.header;

        bool parsed_any = false;
        while (ss >> x >> y >> z)
        {
            parsed_any = true;

            // Raw data
            raw_msg.sensor_ids.push_back(id);
            raw_msg.mag_x.push_back(x);
            raw_msg.mag_y.push_back(y);
            raw_msg.mag_z.push_back(z);

            // Converted data (mT)
            field_msg.sensor_ids.push_back(id);
            field_msg.mag_x.push_back((x / raw_max_) * full_scale_mT_);
            field_msg.mag_y.push_back((y / raw_max_) * full_scale_mT_);
            field_msg.mag_z.push_back((z / raw_max_) * full_scale_mT_);

            id++;
        }

        if (parsed_any) {
            raw_pub_.publish(raw_msg);
            field_pub_.publish(field_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    serial::Serial serial_;
    std::string port_;
    int baud_;
    double full_scale_mT_;
    double raw_max_;

    ros::Publisher raw_pub_;
    ros::Publisher field_pub_;
    std::thread worker_;
    std::atomic<bool> running_{false};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mag_device_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    SensorDriverNode node(nh, pnh);

    ros::spin();
    return 0;
}
