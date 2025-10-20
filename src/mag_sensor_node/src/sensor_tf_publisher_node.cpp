#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mag_sensor_node/sensor_config.hpp>

#include <string>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "sensor_tf_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(pnh))
    {
        ROS_FATAL("[sensor_tf_publisher] 传感器配置加载失败");
        return 1;
    }

    std::string parent_frame;
    std::string array_frame;
    if (!pnh.getParam("sensor_config/array/parent_frame", parent_frame))
    {
        ROS_FATAL("[sensor_tf_publisher] 缺少参数 ~sensor_config/array/parent_frame");
        return 1;
    }
    if (!pnh.getParam("sensor_config/array/frame_id", array_frame))
    {
        ROS_FATAL("[sensor_tf_publisher] 缺少参数 ~sensor_config/array/frame_id");
        return 1;
    }

    const auto &cfg = mag_sensor_node::SensorConfig::getInstance();
    const auto &sensors = cfg.getAllSensors();
    const geometry_msgs::Pose &array_off = cfg.getArrayOffset();
    if (sensors.empty())
    {
        ROS_ERROR("[sensor_tf_publisher] 无传感器条目，退出");
        return 1;
    }

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> tfs;
    tfs.reserve(sensors.size());

    // 先发布 parent -> array/offset 的静态TF（若非单位）
    {
        geometry_msgs::TransformStamped tf_arr;
        tf_arr.header.stamp = ros::Time::now();
        tf_arr.header.frame_id = parent_frame;
        tf_arr.child_frame_id = array_frame;
        tf_arr.transform.translation.x = array_off.position.x;
        tf_arr.transform.translation.y = array_off.position.y;
        tf_arr.transform.translation.z = array_off.position.z;
        tf_arr.transform.rotation = array_off.orientation;
        tfs.push_back(tf_arr);
    }

    for (const auto &s : sensors)
    {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = array_frame; // parent after offset
        tf.child_frame_id = "sensor_" + std::to_string(s.id);
        tf.transform.translation.x = s.pose.position.x;
        tf.transform.translation.y = s.pose.position.y;
        tf.transform.translation.z = s.pose.position.z;
        tf.transform.rotation = s.pose.orientation;
        tfs.push_back(tf);
    }

    static_broadcaster.sendTransform(tfs);
    ROS_INFO("[sensor_tf_publisher] 已发布 %zu 个静态TF, 父坐标系=%s", tfs.size(), parent_frame.c_str());

    ros::spin(); // static tf latched; keep node alive
    return 0;
}
