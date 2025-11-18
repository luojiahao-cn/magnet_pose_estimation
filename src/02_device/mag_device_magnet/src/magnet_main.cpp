#include <mag_device_magnet/magnet_config_loader.hpp>
#include <mag_device_magnet/magnet_node.hpp>

#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include <locale>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_magnet");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string ns = "mag_device_magnet";

        XmlRpc::XmlRpcValue config;
        std::size_t error = 0;
        error += !rps::get(ns, pnh, "config", config);
        rps::shutdownIfError(ns, error);

        auto magnet_config = mag_device_magnet::loadMagnetConfig(config, ns + ".config");

        mag_device_magnet::MagnetNode node(nh,
                                           pnh,
                                           magnet_config.frame,
                                           magnet_config.motion,
                                           magnet_config.topics,
                                           magnet_config.tf,
                                           magnet_config.trajectory,
                                           magnet_config.orientation);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_magnet 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}
