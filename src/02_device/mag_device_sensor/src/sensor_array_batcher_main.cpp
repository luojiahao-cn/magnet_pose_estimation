#include <mag_device_sensor/sensor_array_batcher_node.hpp>

#include <ros/ros.h>

#include <locale>

int main(int argc, char **argv)
{
    // 设置本地化，支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    ros::init(argc, argv, "sensor_array_batcher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    mag_device_sensor::SensorArrayBatcherNode node(nh, pnh);
    node.start();

    return 0;
}

