#include "magnetic_pose_estimation/sensor_config.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace magnetic_pose_estimation {

bool SensorConfig::loadConfig(const ros::NodeHandle& nh) {
    try {
        // 读取坐标系配置
        if (!nh.getParam("frames/parent_frame", parent_frame_)) {
            ROS_ERROR("未找到 parent_frame 参数");
            return false;
        }
        if (!nh.getParam("frames/array_frame", array_frame_)) {
            ROS_ERROR("未找到 array_frame 参数");
            return false;
        }

        // 读取传感器阵列位姿
        std::vector<double> position, orientation;
        if (!nh.getParam("frames/array_pose/position", position) || 
            !nh.getParam("frames/array_pose/orientation", orientation)) {
            ROS_ERROR("未找到阵列位姿参数");
            return false;
        }

        // 设置阵列位姿
        array_pose_.position.x = position[0];
        array_pose_.position.y = position[1];
        array_pose_.position.z = position[2];
        
        // 读取传感器配置
        XmlRpc::XmlRpcValue sensors_list;
        if (!nh.getParam("sensors", sensors_list)) {
            ROS_ERROR("未找到传感器配置参数");
            return false;
        }

        if (sensors_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("传感器配置必须是数组格式");
            return false;
        }

        // 清空现有配置
        sensors_.clear();
        id_index_map_.clear();

        // 遍历所有传感器
        for (int i = 0; i < sensors_list.size(); ++i) {
            try {
                if (sensors_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                    ROS_WARN("跳过无效的传感器配置项 %d", i);
                    continue;
                }

                SensorInfo sensor;
                sensor.id = static_cast<int>(sensors_list[i]["id"]);

                // 直接从 XmlRpcValue 中读取位置
                XmlRpc::XmlRpcValue position = sensors_list[i]["position"];
                if (position.getType() != XmlRpc::XmlRpcValue::TypeArray || 
                    position.size() != 3) {
                    ROS_WARN("传感器 %d 的位置数据格式错误", sensor.id);
                    continue;
                }

                sensor.pose.position.x = static_cast<double>(position[0]);
                sensor.pose.position.y = static_cast<double>(position[1]);
                sensor.pose.position.z = static_cast<double>(position[2]);

                // 直接从 XmlRpcValue 中读取方向
                XmlRpc::XmlRpcValue orientation = sensors_list[i]["orientation"];
                if (orientation.getType() != XmlRpc::XmlRpcValue::TypeArray || 
                    orientation.size() != 3) {
                    ROS_WARN("传感器 %d 的方向数据格式错误", sensor.id);
                    continue;
                }

                // 将RPY转换为四元数
                tf2::Quaternion q;
                q.setRPY(
                    static_cast<double>(orientation[0]),
                    static_cast<double>(orientation[1]),
                    static_cast<double>(orientation[2])
                );
                sensor.pose.orientation.x = q.x();
                sensor.pose.orientation.y = q.y();
                sensor.pose.orientation.z = q.z();
                sensor.pose.orientation.w = q.w();

                id_index_map_[sensor.id] = sensors_.size();
                sensors_.push_back(sensor);
                
                ROS_DEBUG("成功加载传感器 %d 配置", sensor.id);
            }
            catch (const std::exception& e) {
                ROS_WARN("处理传感器 %d 配置时出错: %s", i, e.what());
                continue;
            }
        }

        std::string node_name = ros::this_node::getName();
        ROS_INFO("%s 成功加载 %lu 个传感器的配置", node_name.c_str(), sensors_.size());
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("加载配置时发生错误: %s", e.what());
        return false;
    }
}

const std::vector<SensorInfo>& SensorConfig::getAllSensors() const {
    return sensors_;
}


bool SensorConfig::getSensorById(int id, SensorInfo& sensor) const {
    auto it = id_index_map_.find(id);
    if (it == id_index_map_.end()) {
        return false;
    }
    sensor = sensors_[it->second];
    return true;
}

}  // namespace magnetic_pose_estimation

// // 在其他cpp文件中使用
// #include "magnetic_pose_estimation/sensor_config.hpp"

// void someFunction(ros::NodeHandle &nh)
// {
//     // 获取配置实例并加载配置
//     auto &config = magnetic_pose_estimation::SensorConfig::getInstance();
//     if (!config.loadConfig(nh))
//     {
//         ROS_ERROR("Failed to load sensor configuration");
//         return;
//     }

//     // 获取所有传感器信息
//     const auto &sensors = config.getAllSensors();

//     // 获取单个传感器信息
//     magnetic_pose_estimation::SensorInfo sensor;
//     if (config.getSensorById(1, sensor))
//     {
//         // 使用传感器信息
//         geometry_msgs::Pose sensor_pose = sensor.pose;
//     }

//     // 获取坐标系信息
//     std::string parent_frame = config.getParentFrame();
//     geometry_msgs::Pose array_pose = config.getArrayPose();
// }