#include <ros/ros.h>
#include <mag_core_description/sensor_array_description.hpp>

/**
 * @brief 磁传感器阵列辅助描述节点
 *
 * 该节点负责从参数服务器读取传感器阵列配置并发布对应的 TF 变换。
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mag_description_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    setlocale(LC_ALL, "zh_CN.UTF-8");

    // 获取传感器阵列配置参数
    XmlRpc::XmlRpcValue array_config;
    // 适配 YAML 文件中顶层是 'config' 的情况
    if (!nh.getParam("config", array_config))
    {
        ROS_ERROR("未能在参数服务器中找到 'config' 配置，请检查 YAML 是否已加载。");
        return 1;
    }

    try
    {
        // 1. 初始化描述对象并从参数加载
        mag_core_description::SensorArrayDescription desc;
        // 注意：因为已直接获取了 config 下的内容，此处 context 传入空字符串
        desc.load(mag_core_description::SensorArrayDescription::loadFromParam(array_config, ""));

        // 2. 初始化发布器
        mag_core_description::SensorArrayTfPublisher tf_pub(desc);

        // 3. 发布静态变换
        // 静态变换在 tf2 中只需要发布一次（只要节点不关闭）
        tf_pub.publishStatic();

        ROS_INFO("磁传感器阵列 TF 静态变换发布成功。");

        // 保持节点运行以维护静态变换的 latch 状态
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("mag_description_node 运行出错: %s", e.what());
        return 1;
    }

    return 0;
}
