#include <mag_device_sensor/sim_node.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace mag_device_sensor
{
namespace
{
constexpr double kMu0Div4Pi = 1e-7;
}

CalibrationConfig loadCalibrationConfig(const mag_core_utils::param::StructReader &root,
                                        CalibrationConfig defaults)
{
    // 仿真节点与驱动节点共享的标定量程配置
    if (root.has("calibration"))
    {
        const auto calib = root.childStruct("calibration");
        defaults.full_scale_mT = calib.optionalNumber("full_scale_mT", defaults.full_scale_mT);
        defaults.raw_max = calib.optionalNumber("raw_max", defaults.raw_max);
    }
    if (defaults.full_scale_mT <= 0.0 || defaults.raw_max <= 0.0)
    {
        throw std::runtime_error("calibration.* 参数需为正数");
    }
    return defaults;
}

NoiseConfig loadNoiseConfig(const mag_core_utils::param::StructReader &root,
                            NoiseConfig defaults)
{
    // 读取噪声模型的开关与参数
    if (root.has("simulation"))
    {
        const auto simulation = root.childStruct("simulation");
        if (simulation.has("noise"))
        {
            const auto noise = simulation.childStruct("noise");
            defaults.enable = noise.optionalBool("enable", defaults.enable);
            defaults.type = noise.optionalString("type", defaults.type);
            defaults.mean = noise.optionalNumber("mean", defaults.mean);
            defaults.stddev = noise.optionalNumber("stddev", defaults.stddev);
            defaults.amplitude = noise.optionalNumber("amplitude", defaults.amplitude);
        }
    }
    return defaults;
}

SimulationConfig loadSimulationConfig(const mag_core_utils::param::StructReader &root,
                                      SimulationConfig defaults)
{
    if (!root.has("simulation"))
    {
        throw std::runtime_error("缺少 simulation 配置");
    }
    // 核心仿真参数：更新频率、磁体姿态来源以及偶极强度
    const auto simulation = root.childStruct("simulation");
    defaults.update_rate_hz = simulation.optionalNumber("update_rate_hz", defaults.update_rate_hz);
    if (defaults.update_rate_hz <= 0.0)
    {
        throw std::runtime_error("simulation/update_rate_hz 需为正数");
    }

    if (!simulation.has("magnet_frame"))
    {
        throw std::runtime_error("缺少 simulation/magnet_frame 配置");
    }
    const auto magnet_frame = simulation.childStruct("magnet_frame");
    defaults.magnet_parent_frame = magnet_frame.requireString("parent");
    defaults.magnet_child_frame = magnet_frame.requireString("child");

    if (simulation.has("magnet"))
    {
        const auto magnet = simulation.childStruct("magnet");
        defaults.dipole_strength = magnet.optionalNumber("dipole_strength", defaults.dipole_strength);
        if (defaults.dipole_strength <= 0.0)
        {
            throw std::runtime_error("simulation/magnet/dipole_strength 需为正数");
        }
        if (magnet.has("base_direction"))
        {
            const auto vec = magnet.requireVector3("base_direction");
            defaults.base_direction = Eigen::Vector3d(vec[0], vec[1], vec[2]);
        }
    }

    if (defaults.base_direction.norm() == 0.0)
    {
        throw std::runtime_error("simulation/magnet/base_direction 不可为零向量");
    }
    defaults.base_direction.normalize();
    return defaults;
}

SensorSimNode::SensorSimNode(ros::NodeHandle nh,
                             ros::NodeHandle pnh,
                             const mag_core_description::SensorArrayDescription &array,
                             TopicConfig topic_config,
                             TfConfig tf_config,
                             CalibrationConfig calibration,
                             NoiseConfig noise,
                             SimulationConfig simulation)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      array_(array),
      tf_publisher_(std::make_unique<mag_core_description::SensorArrayTfPublisher>(array_)),
      topic_config_(std::move(topic_config)),
      tf_config_(tf_config),
      calibration_(std::move(calibration)),
      noise_(std::move(noise)),
      simulation_(std::move(simulation)),
      tf_listener_(tf_buffer_)
{
    if (calibration_.full_scale_mT <= 0.0 || calibration_.raw_max <= 0.0)
    {
        throw std::runtime_error("calibration.* 参数需为正数");
    }
    // 仿真节点也可发布阵列 TF，因此与驱动节点共用发布器
    setupPublishers();
}

void SensorSimNode::start()
{
    // 启动仿真更新计时器以及可选的 TF 计时器
    setupTimers();
}

Eigen::MatrixXd SensorSimNode::computeField(const Eigen::Matrix<double, Eigen::Dynamic, 3> &sensor_positions,
                                            const Eigen::Vector3d &magnet_position,
                                            const Eigen::Vector3d &magnet_direction,
                                            double dipole_strength)
{
    // 基于偶极子模型计算每个传感器位置上的磁场，单位 mT
    Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnet_position.transpose();
    Eigen::VectorXd r_norm = r_vec.rowwise().norm();
    r_norm = r_norm.array().max(1e-12);
    Eigen::Vector3d m = dipole_strength * magnet_direction;
    Eigen::VectorXd m_dot_r = r_vec * m;
    Eigen::MatrixXd field = (kMu0Div4Pi * 1e3) *
                            (3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5))) -
                             m.transpose().replicate(r_vec.rows(), 1).array().colwise() / r_norm.array().pow(3));
    return field;
}

void SensorSimNode::setupPublishers()
{
    // 初始化仿真数据的 raw 与 field 发布器
    if (!topic_config_.raw_topic.empty())
    {
        raw_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.raw_topic, 50);
    }
    if (!topic_config_.field_topic.empty())
    {
        field_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.field_topic, 50);
    }
}

void SensorSimNode::setupTimers()
{
    // 定期触发仿真采样；若需要也周期发送 TF
    simulation_timer_ = nh_.createTimer(ros::Duration(1.0 / simulation_.update_rate_hz),
                                        &SensorSimNode::onSimulationTimer,
                                        this);
    if (tf_config_.enable && tf_publisher_)
    {
        if (tf_config_.publish_rate > 0.0)
        {
            tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_config_.publish_rate),
                                        &SensorSimNode::onTfTimer,
                                        this);
        }
        else
        {
            tf_publisher_->publishStatic();
        }
    }
}

void SensorSimNode::onSimulationTimer(const ros::TimerEvent &)
{
    // 从 TF 树获取磁体姿态，若失败则跳过当前周期
    geometry_msgs::TransformStamped magnet_tf;
    try
    {
        magnet_tf = tf_buffer_.lookupTransform(simulation_.magnet_parent_frame,
                                               simulation_.magnet_child_frame,
                                               ros::Time(0),
                                               ros::Duration(0.05));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(2.0,
                          "[mag_device_sensor_sim] 无法查询磁铁 TF (%s->%s): %s",
                          simulation_.magnet_parent_frame.c_str(),
                          simulation_.magnet_child_frame.c_str(),
                          ex.what());
        return;
    }

    publishReadings(magnet_tf, ros::Time::now());
}

void SensorSimNode::onTfTimer(const ros::TimerEvent &)
{
    if (tf_publisher_)
    {
        tf_publisher_->publishDynamic(ros::Time::now());
    }
}

Eigen::Vector3d SensorSimNode::sampleNoise()
{
    // 根据配置返回一次性噪声采样，支持高斯或均匀分布
    Eigen::Vector3d noise = Eigen::Vector3d::Zero();
    if (!noise_.enable)
    {
        return noise;
    }
    if (noise_.type == "gaussian")
    {
        std::normal_distribution<double> dist(noise_.mean, noise_.stddev);
        noise.x() = dist(rng_);
        noise.y() = dist(rng_);
        noise.z() = dist(rng_);
    }
    else if (noise_.type == "uniform")
    {
        std::uniform_real_distribution<double> dist(-noise_.amplitude, noise_.amplitude);
        noise.x() = dist(rng_);
        noise.y() = dist(rng_);
        noise.z() = dist(rng_);
    }
    else
    {
        ROS_WARN_ONCE("[mag_device_sensor_sim] 未识别的噪声类型 %s，将忽略噪声", noise_.type.c_str());
    }
    return noise;
}

void SensorSimNode::publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp)
{
    // 将阵列描述映射为传感器在父坐标系下的位置
    const auto &sensors = array_.sensors();
    if (sensors.empty())
    {
        ROS_WARN_THROTTLE(5.0, "[mag_device_sensor_sim] 无可用传感器描述，跳过发布");
        return;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(sensors.size(), 3);

    tf2::Transform parent_array;
    tf2::fromMsg(array_.arrayPose(), parent_array);

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        tf2::Transform array_sensor;
        tf2::fromMsg(sensors[i].pose, array_sensor);
        tf2::Transform parent_sensor = parent_array * array_sensor;
        const tf2::Vector3 &origin = parent_sensor.getOrigin();
        sensor_positions(static_cast<int>(i), 0) = origin.x();
        sensor_positions(static_cast<int>(i), 1) = origin.y();
        sensor_positions(static_cast<int>(i), 2) = origin.z();
    }

    Eigen::Vector3d magnet_position(magnet_tf.transform.translation.x,
                                    magnet_tf.transform.translation.y,
                                    magnet_tf.transform.translation.z);

    tf2::Quaternion magnet_q;
    tf2::fromMsg(magnet_tf.transform.rotation, magnet_q);
    tf2::Vector3 base_dir(simulation_.base_direction.x(),
                          simulation_.base_direction.y(),
                          simulation_.base_direction.z());
    tf2::Vector3 rotated = tf2::quatRotate(magnet_q, base_dir);
    Eigen::Vector3d magnet_direction(rotated.x(), rotated.y(), rotated.z());

    Eigen::MatrixXd fields_mT = computeField(sensor_positions,
                                             magnet_position,
                                             magnet_direction.normalized(),
                                             simulation_.dipole_strength);

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        // 先叠加噪声，再分别发布 raw 与 mT 数据
        Eigen::Vector3d value_mT(fields_mT(static_cast<int>(i), 0),
                                 fields_mT(static_cast<int>(i), 1),
                                 fields_mT(static_cast<int>(i), 2));
        value_mT += sampleNoise();

        if (raw_pub_)
        {
            mag_core_msgs::MagSensorData msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = sensors[i].frame_id;
            msg.sensor_id = sensors[i].id;
            msg.mag_x = toRaw(value_mT.x());
            msg.mag_y = toRaw(value_mT.y());
            msg.mag_z = toRaw(value_mT.z());
            raw_pub_.publish(msg);
        }

        if (field_pub_)
        {
            mag_core_msgs::MagSensorData msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = sensors[i].frame_id;
            msg.sensor_id = sensors[i].id;
            msg.mag_x = value_mT.x();
            msg.mag_y = value_mT.y();
            msg.mag_z = value_mT.z();
            field_pub_.publish(msg);
        }
    }
}

} // namespace mag_device_sensor
