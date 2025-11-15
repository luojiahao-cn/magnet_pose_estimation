#include <mag_device_magnet/magnet_node.hpp>

#include <ros/ros.h>

#include <locale>

using mag_core_utils::param::StructReader;

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_magnet");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        auto root = StructReader::fromParameter(pnh, "config");
        auto frame = mag_device_magnet::loadFrameConfig(root);
        auto topics = mag_device_magnet::loadTopicConfig(root);
        auto motion = mag_device_magnet::loadMotionConfig(root);
        auto tf = mag_device_magnet::loadTfConfig(root);
        auto trajectory = mag_device_magnet::loadTrajectoryConfig(root);
        auto orientation = mag_device_magnet::loadOrientationConfig(root);

        mag_device_magnet::MagnetNode node(nh, pnh, frame, motion, topics, tf, trajectory, orientation);
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
