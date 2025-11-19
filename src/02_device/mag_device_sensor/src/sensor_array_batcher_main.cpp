#include <mag_device_sensor/sensor_array_batcher_node.hpp>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_array_batcher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    mag_device_sensor::SensorArrayBatcherNode node(nh, pnh);
    node.start();

    return 0;
}

