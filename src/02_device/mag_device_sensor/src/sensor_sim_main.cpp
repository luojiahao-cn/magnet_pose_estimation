#include <mag_device_sensor/sim_node.hpp>

#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_utils/param_reader.hpp>

#include <ros/ros.h>

#include <locale>

using mag_core_utils::param::StructReader;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_sensor_sim");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        // 读入仿真所需的配置和阵列信息
        auto root = StructReader::fromParameter(pnh, "config");
        mag_core_description::SensorArrayDescription array;
        array.load(pnh, "array");

        // 构建仿真节点并启动定时器
        auto topics = mag_device_sensor::loadTopicConfig(root);
        auto tf = mag_device_sensor::loadTfConfig(root);
        auto calibration = mag_device_sensor::loadCalibrationConfig(root);
        auto noise = mag_device_sensor::loadNoiseConfig(root);
        auto simulation = mag_device_sensor::loadSimulationConfig(root);

        mag_device_sensor::SensorSimNode node(nh, pnh, array, topics, tf, calibration, noise, simulation);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_sensor_sim 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}
