#include <mag_device_arm/arm_node.hpp>

#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <locale>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_arm");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    namespace rps = rosparam_shortcuts;
    const std::string ns = "mag_device_arm";

    try
    {
        XmlRpc::XmlRpcValue config;
        std::size_t error = 0;
        error += !rps::get(ns, pnh, "config", config);
        rps::shutdownIfError(ns, error);

        auto arm_configs = mag_device_arm::loadArmConfigs(config, ns + ".config");

        mag_device_arm::ArmNode node(nh, pnh, std::move(arm_configs));
        node.start();

        ros::waitForShutdown();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_arm 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}
