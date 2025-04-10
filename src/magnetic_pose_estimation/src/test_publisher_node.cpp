#include <ros/ros.h>
#include <cmath>
#include "magnetic_pose_estimation/MagneticField.h"
#include "magnetic_pose_estimation/sensor_config.hpp"

class TestPublisher {
public:
    TestPublisher(ros::NodeHandle& nh) : nh_(nh), time_(0.0) {
        // 创建发布器
        real_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/real", 100);
        sim_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/simulation", 100);
        pred_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/predicted", 100);

        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 创建定时器，10Hz
        timer_ = nh_.createTimer(ros::Duration(0.1), &TestPublisher::timerCallback, this);
    }

private:
    void timerCallback(const ros::TimerEvent&) {
        const auto& sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();
        
        for (const auto& sensor : sensors) {
            // 创建消息
            magnetic_pose_estimation::MagneticField msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "world";
            msg.sensor_id = sensor.id;
            msg.sensor_pose = sensor.pose;

            // 生成模拟磁场数据
            double x = sensor.pose.position.x;
            double y = sensor.pose.position.y;
            
            // 实际数据 - 使用简单的正弦波
            msg.mag_x = 0.1 * std::sin(2.0 * M_PI * 0.5 * time_ + x);
            msg.mag_y = 0.1 * std::cos(2.0 * M_PI * 0.5 * time_ + y);
            msg.mag_z = 0.1 * std::sin(2.0 * M_PI * 0.5 * time_ + x + y);
            real_pub_.publish(msg);

            // 仿真数据 - 稍微偏移的正弦波
            msg.mag_x = 0.12 * std::sin(2.0 * M_PI * 0.5 * time_ + x + 0.1);
            msg.mag_y = 0.12 * std::cos(2.0 * M_PI * 0.5 * time_ + y + 0.1);
            msg.mag_z = 0.12 * std::sin(2.0 * M_PI * 0.5 * time_ + x + y + 0.1);
            sim_pub_.publish(msg);

            // 预测数据 - 另一个偏移的正弦波
            msg.mag_x = 0.08 * std::sin(2.0 * M_PI * 0.5 * time_ + x - 0.1);
            msg.mag_y = 0.08 * std::cos(2.0 * M_PI * 0.5 * time_ + y - 0.1);
            msg.mag_z = 0.08 * std::sin(2.0 * M_PI * 0.5 * time_ + x + y - 0.1);
            pred_pub_.publish(msg);
        }

        time_ += 0.1;
    }

    ros::NodeHandle& nh_;
    ros::Publisher real_pub_, sim_pub_, pred_pub_;
    ros::Timer timer_;
    double time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magnetic_field_test_publisher");
    ros::NodeHandle nh;

    TestPublisher publisher(nh);

    ros::spin();
    return 0;
}