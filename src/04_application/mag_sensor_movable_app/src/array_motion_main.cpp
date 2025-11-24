/**
 * @file array_motion_main.cpp
 * @brief 传感器阵列运动控制节点主程序
 * 
 * 传感器阵列运动控制 ROS 节点的入口程序，负责初始化节点、加载配置并启动节点
 */

#include <mag_sensor_movable_app/array_motion_config_loader.hpp>
#include <mag_sensor_movable_app/array_motion_node.hpp>

#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <ros/ros.h>

#include <locale>

/**
 * @brief 主函数
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码（0 表示成功，1 表示失败）
 */
int main(int argc, char **argv)
{
    // 设置本地化，支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    // 初始化 ROS 节点
    ros::init(argc, argv, "array_motion_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string ns = "array_motion_node";

        // 从 ROS 参数服务器加载配置
        XmlRpc::XmlRpcValue config;
        std::size_t error = 0;
        error += !rps::get(ns, pnh, "config", config);
        rps::shutdownIfError(ns, error);

        // 解析配置并创建配置包
        auto array_motion_config = mag_sensor_movable_app::loadArrayMotionConfig(config, ns + ".config");

        // 创建并启动传感器阵列运动控制节点
        mag_sensor_movable_app::ArrayMotionNode node(nh,
                                                     pnh,
                                                     array_motion_config.frame,
                                                     array_motion_config.motion,
                                                     array_motion_config.trajectory,
                                                     array_motion_config.orientation);
        node.start();
        
        // 进入 ROS 事件循环
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("array_motion_node 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}

