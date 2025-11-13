#include <mag_device_arm/arm_node.hpp>

#include <mag_core_utils/param_reader.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <locale>

using mag_core_utils::param::StructReader;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_arm");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    try
    {
        auto root = StructReader::fromParameter(pnh, "config");
        auto arm_configs = mag_device_arm::loadArmConfigs(root);

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
