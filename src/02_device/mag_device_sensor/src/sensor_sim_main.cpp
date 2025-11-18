#include <mag_device_sensor/sensor_config_loader.hpp>
#include <mag_device_sensor/sim_node.hpp>

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
    ros::init(argc, argv, "mag_device_sensor_sim");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string sim_ns = "mag_device_sensor.sim";
        const std::string array_ns = "mag_device_sensor.array";

        XmlRpc::XmlRpcValue sim_param;
        XmlRpc::XmlRpcValue array_param;

        std::size_t sim_error = 0;
        sim_error += !rps::get(sim_ns, pnh, "config", sim_param);
        rps::shutdownIfError(sim_ns, sim_error);

        std::size_t array_error = 0;
        array_error += !rps::get(array_ns, pnh, "array/config", array_param);
        rps::shutdownIfError(array_ns, array_error);

        auto bundle = mag_device_sensor::loadSensorSimulationConfig(sim_param, sim_ns + ".config");
        auto array_config = mag_core_description::loadSensorArrayConfig(array_param, array_ns + ".config");
        mag_core_description::SensorArrayDescription array;
        array.load(array_config);

        mag_device_sensor::SensorSimNode node(nh,
                              pnh,
                              array,
                              bundle.topics,
                              bundle.tf,
                              bundle.calibration,
                              bundle.noise,
                              bundle.simulation);
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
