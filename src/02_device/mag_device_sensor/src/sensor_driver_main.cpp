#include <mag_device_sensor/sensor_node.hpp>

#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_utils/param_reader.hpp>

#include <ros/ros.h>

#include <locale>

using mag_core_utils::param::StructReader;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        // 读取驱动配置与传感器阵列描述
        auto root = StructReader::fromParameter(pnh, "config");
        mag_core_description::SensorArrayDescription array;
        array.load(pnh, "array");

        // 构建驱动节点并启动后台线程
        auto driver = mag_device_sensor::loadDriverConfig(root);
        auto topics = mag_device_sensor::loadTopicConfig(root);
        auto tf = mag_device_sensor::loadTfConfig(root);

        mag_device_sensor::SensorNode node(nh, pnh, array, driver, topics, tf);
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
