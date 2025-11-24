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
    // 记录启动时间，用于延迟TF查询失败的警告
    start_time_ = ros::Time::now();
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
    else if (tf_publisher_)
    {
        // 即使 TF 发布被禁用，也发布传感器相对于阵列的静态 TF
        // 这对于可移动传感器阵列场景很重要，因为阵列的 TF 由其他节点发布
        tf_publisher_->publishSensorTfsOnly();
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
        // 启动后3秒内不报warn，给TF树建立时间
        const ros::Duration time_since_start = ros::Time::now() - start_time_;
        if (time_since_start.toSec() > 3.0)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[mag_device_sensor_sim] ✗ 无法查询磁铁 TF (%s -> %s): %s",
                              simulation_.magnet_parent_frame.c_str(),
                              simulation_.magnet_child_frame.c_str(),
                              ex.what());
        }
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
        ROS_WARN_ONCE("[mag_device_sensor_sim] ✗ 未识别的噪声类型 '%s'，将忽略噪声", noise_.type.c_str());
    }
    return noise;
}

void SensorSimNode::publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp)
{
    // 将阵列描述映射为传感器在父坐标系下的位置
    const auto &sensors = array_.sensors();
    if (sensors.empty())
    {
        ROS_WARN_THROTTLE(5.0, "[mag_device_sensor_sim] ✗ 无可用传感器描述，跳过发布");
        return;
    }

    // 从 TF 树查询传感器阵列的当前位姿（支持可移动传感器阵列）
    tf2::Transform parent_array;
    try
    {
        geometry_msgs::TransformStamped array_tf = tf_buffer_.lookupTransform(
            array_.parentFrame(), array_.arrayFrame(), stamp, ros::Duration(0.1));
        tf2::fromMsg(array_tf.transform, parent_array);
    }
    catch (const tf2::TransformException &ex)
    {
        // 如果查询失败，回退到使用配置文件中的静态位姿
        const ros::Duration time_since_start = ros::Time::now() - start_time_;
        if (time_since_start.toSec() > 3.0)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[mag_device_sensor_sim] ✗ 无法查询传感器阵列 TF (%s -> %s)，使用配置中的静态位姿: %s",
                              array_.parentFrame().c_str(), array_.arrayFrame().c_str(), ex.what());
        }
        tf2::fromMsg(array_.arrayPose(), parent_array);
    }

    Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(sensors.size(), 3);
    std::vector<tf2::Transform> parent_sensor_transforms(sensors.size());

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        tf2::Transform array_sensor;
        tf2::fromMsg(sensors[i].pose, array_sensor);
        tf2::Transform parent_sensor = parent_array * array_sensor;
        parent_sensor_transforms[i] = parent_sensor;
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
        // 获取世界坐标系下的磁场
        Eigen::Vector3d field_world_mT(fields_mT(static_cast<int>(i), 0),
                                       fields_mT(static_cast<int>(i), 1),
                                       fields_mT(static_cast<int>(i), 2));
        
        // 将磁场从世界坐标系转换到传感器坐标系
        // getBasis() 返回从传感器坐标系到世界坐标系的旋转矩阵
        // 需要取逆来得到从世界坐标系到传感器坐标系的旋转
        tf2::Matrix3x3 R_world_to_sensor = parent_sensor_transforms[i].getBasis().transpose();
        tf2::Vector3 field_sensor_tf2 = R_world_to_sensor * tf2::Vector3(field_world_mT.x(),
                                                                          field_world_mT.y(),
                                                                          field_world_mT.z());
        Eigen::Vector3d value_mT(field_sensor_tf2.x(), field_sensor_tf2.y(), field_sensor_tf2.z());
        
        // 叠加噪声
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
