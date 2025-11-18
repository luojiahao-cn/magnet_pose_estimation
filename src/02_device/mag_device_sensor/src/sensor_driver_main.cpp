#include <mag_device_sensor/sensor_config_loader.hpp>
#include <mag_device_sensor/sensor_node.hpp>

#include <mag_core_description/sensor_array_config_loader.hpp>
#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <locale>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string driver_ns = "mag_device_sensor.driver";
        const std::string array_ns = "mag_device_sensor.array";

        XmlRpc::XmlRpcValue driver_param;
        XmlRpc::XmlRpcValue array_param;

        std::size_t driver_error = 0;
        driver_error += !rps::get(driver_ns, pnh, "config", driver_param);
        rps::shutdownIfError(driver_ns, driver_error);

        std::size_t array_error = 0;
        array_error += !rps::get(array_ns, pnh, "array/config", array_param);
        rps::shutdownIfError(array_ns, array_error);

        auto bundle = mag_device_sensor::loadSensorDriverConfig(driver_param, driver_ns + ".config");
        auto array_config = mag_core_description::loadSensorArrayConfig(array_param, array_ns + ".config");
        mag_core_description::SensorArrayDescription array;
        array.load(array_config);

        mag_device_sensor::SensorNode node(nh,
                           pnh,
                           array,
                           bundle.driver,
                           bundle.topics,
                           bundle.tf);
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
