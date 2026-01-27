#include <mag_core_description/sensor_array_description.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdexcept>
#include <string>

namespace mag_core_description
{
    namespace
    {

        namespace xml = mag_core_utils::xmlrpc;

        geometry_msgs::Pose poseFromXyzRpyHelper(const std::vector<double> &xyz,
                                                 const std::vector<double> &rpy)
        {
            geometry_msgs::Pose pose;
            pose.position.x = xyz[0];
            pose.position.y = xyz[1];
            pose.position.z = xyz[2];
            tf2::Quaternion q;
            q.setRPY(rpy[0], rpy[1], rpy[2]);
            pose.orientation = tf2::toMsg(q);
            return pose;
        }

        SensorArraySensorConfig parseSensorEntry(const XmlRpc::XmlRpcValue &node,
                                                 const std::string &context,
                                                 const std::string &sensor_frame_prefix)
        {
            const auto &entry = xml::asStruct(node, context);
            SensorArraySensorConfig cfg;
            cfg.id = static_cast<int>(xml::readNumber(xml::requireMember(entry, "id", context),
                                                      xml::makeContext(context, "id")));
            cfg.frame_id = sensor_frame_prefix + std::to_string(cfg.id);
            if (xml::hasMember(entry, "frame"))
            {
                cfg.frame_id = xml::requireStringField(entry, "frame", context);
            }

            const auto pose_ctx = xml::makeContext(context, "pose");
            const auto &pose_node = xml::requireStructField(entry, "pose", context);
            const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
            const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
            cfg.pose = poseFromXyzRpyHelper(xyz, rpy);
            return cfg;
        }

        // 将姿态转换为变换消息
        geometry_msgs::TransformStamped poseToTransform(const geometry_msgs::Pose &pose,
                                                        const std::string &parent,
                                                        const std::string &child,
                                                        const ros::Time &stamp)
        {
            geometry_msgs::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = parent;
            tf.child_frame_id = child;
            tf.transform.translation.x = pose.position.x;
            tf.transform.translation.y = pose.position.y;
            tf.transform.translation.z = pose.position.z;
            tf.transform.rotation = pose.orientation;
            return tf;
        }

    }

    SensorArrayConfig SensorArrayDescription::loadFromParam(const XmlRpc::XmlRpcValue &root,
                                                            const std::string &context)
    {
        const auto &node = xml::asStruct(root, context);

        SensorArrayConfig cfg;
        const auto frames_ctx = xml::makeContext(context, "frames");
        const auto &frames = xml::requireStructField(node, "frames", context);
        cfg.parent_frame = xml::requireStringField(frames, "parent", frames_ctx);
        cfg.array_frame = xml::requireStringField(frames, "array", frames_ctx);
        cfg.sensor_frame_prefix = xml::optionalStringField(frames, "sensor_frame_prefix", frames_ctx, cfg.sensor_frame_prefix);

        const auto pose_ctx = xml::makeContext(frames_ctx, "pose");
        const auto &pose_node = xml::requireStructField(frames, "pose", frames_ctx);
        const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
        const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
        cfg.array_pose = poseFromXyzRpyHelper(xyz, rpy);

        const auto sensors_ctx = xml::makeContext(context, "sensors");
        const auto &sensors = xml::requireArrayField(node, "sensors", context);
        cfg.sensors.reserve(sensors.size());
        for (int i = 0; i < sensors.size(); ++i)
        {
            const auto entry_ctx = xml::makeContext(sensors_ctx, "[" + std::to_string(i) + "]");
            cfg.sensors.push_back(parseSensorEntry(sensors[i], entry_ctx, cfg.sensor_frame_prefix));
        }

        if (cfg.sensors.empty())
        {
            throw std::runtime_error(context + ": sensors 列表不可为空");
        }

        return cfg;
    }

    // SensorArrayDescription 类：描述传感器阵列的配置和信息
    void SensorArrayDescription::load(const SensorArrayConfig &config)
    {
        parent_frame_ = config.parent_frame;
        array_frame_ = config.array_frame;
        array_pose_ = config.array_pose;
        if (!config.sensor_frame_prefix.empty())
        {
            sensor_frame_prefix_ = config.sensor_frame_prefix;
        }

        sensors_.clear();
        index_by_id_.clear();
        sensors_.reserve(config.sensors.size());

        for (const auto &sensor_cfg : config.sensors)
        {
            if (index_by_id_.count(sensor_cfg.id) != 0)
            {
                throw std::runtime_error("duplicate sensor id: " + std::to_string(sensor_cfg.id));
            }

            SensorEntry entry;
            entry.id = sensor_cfg.id;
            entry.pose = sensor_cfg.pose;
            entry.frame_id = sensor_cfg.frame_id.empty()
                                 ? sensor_frame_prefix_ + std::to_string(sensor_cfg.id)
                                 : sensor_cfg.frame_id;

            index_by_id_.emplace(entry.id, sensors_.size());
            sensors_.push_back(entry);
        }

        if (sensors_.empty())
        {
            throw std::runtime_error("sensor array must contain at least one sensor");
        }
    }

    // 根据ID查找传感器
    const SensorEntry *SensorArrayDescription::findSensor(int id) const
    {
        const auto it = index_by_id_.find(id);
        if (it == index_by_id_.end())
        {
            return nullptr;
        }
        return &sensors_.at(it->second);
    }

    // 获取传感器的帧名称
    std::string SensorArrayDescription::sensorFrameName(int id) const
    {
        const auto *entry = findSensor(id);
        if (!entry)
        {
            return sensor_frame_prefix_ + std::to_string(id);
        }
        return entry->frame_id;
    }

    // SensorArrayTfPublisher 类：发布传感器阵列的TF变换
    SensorArrayTfPublisher::SensorArrayTfPublisher(const SensorArrayDescription &description)
        : description_(description)
    {
    }

    // 构建所有变换消息
    std::vector<geometry_msgs::TransformStamped> SensorArrayTfPublisher::buildTransforms(const ros::Time &stamp) const
    {
        std::vector<geometry_msgs::TransformStamped> result;
        result.reserve(description_.sensors().size() + 1);

        result.push_back(poseToTransform(description_.arrayPose(),
                                         description_.parentFrame(),
                                         description_.arrayFrame(),
                                         stamp));

        for (const auto &sensor : description_.sensors())
        {
            result.push_back(poseToTransform(sensor.pose,
                                             description_.arrayFrame(),
                                             sensor.frame_id,
                                             stamp));
        }
        return result;
    }

    // 发布动态变换
    void SensorArrayTfPublisher::publishDynamic(const ros::Time &stamp)
    {
        dynamic_broadcaster_.sendTransform(buildTransforms(stamp));
    }

    // 发布静态变换（仅一次）
    void SensorArrayTfPublisher::publishStatic()
    {
        if (static_sent_)
        {
            return;
        }
        static_broadcaster_.sendTransform(buildTransforms(ros::Time::now()));
        static_sent_ = true;
    }

    // 构建传感器相对于阵列的变换（不包含阵列相对于父坐标系的变换）
    std::vector<geometry_msgs::TransformStamped> SensorArrayTfPublisher::buildSensorTransforms(const ros::Time &stamp) const
    {
        std::vector<geometry_msgs::TransformStamped> result;
        result.reserve(description_.sensors().size());

        for (const auto &sensor : description_.sensors())
        {
            result.push_back(poseToTransform(sensor.pose,
                                             description_.arrayFrame(),
                                             sensor.frame_id,
                                             stamp));
        }
        return result;
    }

    // 只发布传感器相对于阵列的静态 TF
    void SensorArrayTfPublisher::publishSensorTfsOnly()
    {
        if (sensor_tfs_sent_)
        {
            return;
        }
        static_broadcaster_.sendTransform(buildSensorTransforms(ros::Time::now()));
        sensor_tfs_sent_ = true;
    }

} // namespace mag_core_description
