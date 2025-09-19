#include <mag_sensor_node/sensor_config.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace mag_sensor_node
{

    bool SensorConfig::loadConfig(const ros::NodeHandle &nh)
    {
        try
        {
            // 传感器列表
            XmlRpc::XmlRpcValue sensors_list;
            if (!nh.getParam("sensors", sensors_list))
            {
                ROS_ERROR("[SensorConfig] 缺少 sensors 数组");
                return false;
            }
            if (sensors_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("[SensorConfig] sensors 必须是数组");
                return false;
            }

            sensors_.clear();
            id_index_map_.clear();
            sensors_.reserve(sensors_list.size());
            for (int i = 0; i < sensors_list.size(); ++i)
            {
                try
                {
                    if (sensors_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        ROS_WARN("[SensorConfig] 跳过索引 %d: 非结构体", i);
                        continue;
                    }
                    if (!sensors_list[i].hasMember("id") || !sensors_list[i].hasMember("position") || !sensors_list[i].hasMember("orientation"))
                    {
                        ROS_WARN("[SensorConfig] 跳过索引 %d: 缺少 id/position/orientation 字段", i);
                        continue;
                    }
                    SensorInfo sensor;
                    sensor.id = static_cast<int>(sensors_list[i]["id"]);
                    if (sensor.id < 0)
                    {
                        ROS_WARN("[SensorConfig] 传感器索引 %d 的 id=%d 非法", i, sensor.id);
                        continue;
                    }
                    if (id_index_map_.count(sensor.id))
                    {
                        ROS_WARN("[SensorConfig] 检测到重复 id=%d, 跳过后者 (索引 %d)", sensor.id, i);
                        continue;
                    }
                    XmlRpc::XmlRpcValue pos = sensors_list[i]["position"];
                    XmlRpc::XmlRpcValue ori = sensors_list[i]["orientation"];
                    if (pos.getType() != XmlRpc::XmlRpcValue::TypeArray || pos.size() != 3)
                    {
                        ROS_WARN("[SensorConfig] id=%d position 需为长度3数组", sensor.id);
                        continue;
                    }
                    if (ori.getType() != XmlRpc::XmlRpcValue::TypeArray || ori.size() != 3)
                    {
                        ROS_WARN("[SensorConfig] id=%d orientation 需为长度3数组(RPY)", sensor.id);
                        continue;
                    }
                    sensor.pose.position.x = static_cast<double>(pos[0]);
                    sensor.pose.position.y = static_cast<double>(pos[1]);
                    sensor.pose.position.z = static_cast<double>(pos[2]);
                    {
                        tf2::Quaternion q; q.setRPY(static_cast<double>(ori[0]), static_cast<double>(ori[1]), static_cast<double>(ori[2]));
                        sensor.pose.orientation.x = q.x();
                        sensor.pose.orientation.y = q.y();
                        sensor.pose.orientation.z = q.z();
                        sensor.pose.orientation.w = q.w();
                    }
                    id_index_map_[sensor.id] = sensors_.size();
                    sensors_.push_back(sensor);
                }
                catch (const std::exception &e)
                {
                    ROS_WARN("[SensorConfig] 处理索引 %d 时异常: %s", i, e.what());
                }
            }
            ROS_INFO("[SensorConfig] 加载完成: %lu 个有效传感器", sensors_.size());
            if (sensors_.empty())
            {
                ROS_WARN("[SensorConfig] 未加载到任何传感器条目");
            }
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[SensorConfig] 加载过程中发生异常: %s", e.what());
            return false;
        }
    }

    bool SensorConfig::getSensorById(int id, SensorInfo &sensor) const
    {
        auto it = id_index_map_.find(id);
        if (it == id_index_map_.end())
            return false;
        sensor = sensors_[it->second];
        return true;
    }

} // namespace mag_sensor_node
