#include <mag_sensor_node/array_description.hpp>

#include <magnet_msgs/MagSensorData.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    // μ0/4π 常量（单位 V·s/A·m）
    constexpr double kMu0Div4Pi = 1e-7;
}

class MagSensorSimNode
{
public:
    MagSensorSimNode(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
    static Eigen::MatrixXd computeField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                        const Eigen::Vector3d &magnet_position,
                                        const Eigen::Vector3d &magnet_direction,
                                        double dipole_strength);

    void loadArrayDescription();
    void loadParameters();
    void setupPublishers();
    void setupTimers();

    void onSimulationTimer(const ros::TimerEvent &);
    void onTfTimer(const ros::TimerEvent &);
    void publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp);

    Eigen::Vector3d sampleNoise();
    double toMilliTesla(double raw) const { return (raw / raw_max_) * full_scale_mT_; }
    double toRawCount(double mT) const { return (mT / full_scale_mT_) * raw_max_; }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    mag_sensor_node::SensorArrayDescription array_description_;
    std::unique_ptr<mag_sensor_node::SensorArrayTfPublisher> tf_publisher_;

    ros::Publisher raw_pub_;
    ros::Publisher field_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Timer simulation_timer_;
    ros::Timer tf_timer_;

    bool publish_tf_{true};
    double tf_publish_rate_{30.0};
    double update_rate_{100.0};

    std::string topic_raw_;
    std::string topic_field_;

    std::string magnet_parent_frame_;
    std::string magnet_child_frame_;
    double dipole_strength_{10.0};
    Eigen::Vector3d base_direction_{0.0, 0.0, 1.0};

    bool noise_enable_{false};
    std::string noise_type_ = "gaussian";
    double noise_mean_{0.0};
    double noise_stddev_{1.0};
    double noise_amplitude_{0.0};

    double full_scale_mT_{3.2};
    double raw_max_{32767.0};

    std::mt19937 rng_{std::random_device{}()};
};

MagSensorSimNode::MagSensorSimNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)), tf_listener_(tf_buffer_)
{
    loadArrayDescription();
    loadParameters();
    setupPublishers();
    setupTimers();
}

void MagSensorSimNode::loadArrayDescription()
{
    // 仿真与实物共用同一份阵列描述，避免重复维护
    array_description_.load(pnh_, "array");
    tf_publish_rate_ = array_description_.tfPublishRate();
    tf_publisher_ = std::make_unique<mag_sensor_node::SensorArrayTfPublisher>(array_description_);
}

void MagSensorSimNode::loadParameters()
{
    pnh_.param("topics/raw", topic_raw_, std::string());
    pnh_.param("topics/field", topic_field_, std::string("/mag_sensor/data_mT"));
    if (topic_raw_.empty() && topic_field_.empty())
    {
        throw std::runtime_error("仿真节点至少需要发布一个话题 (topics/raw 或 topics/field)");
    }

    pnh_.param("simulation/update_rate_hz", update_rate_, 100.0);
    if (update_rate_ <= 0.0)
    {
        throw std::runtime_error("simulation/update_rate_hz 需为正数");
    }

    if (!pnh_.getParam("simulation/magnet_frame/parent", magnet_parent_frame_))
    {
        throw std::runtime_error("缺少 simulation/magnet_frame/parent 配置");
    }
    if (!pnh_.getParam("simulation/magnet_frame/child", magnet_child_frame_))
    {
        throw std::runtime_error("缺少 simulation/magnet_frame/child 配置");
    }

    pnh_.param("simulation/magnet/dipole_strength", dipole_strength_, 10.0);
    if (dipole_strength_ <= 0.0)
    {
        throw std::runtime_error("simulation/magnet/dipole_strength 需为正数");
    }

    std::vector<double> base_dir_raw;
    if (pnh_.getParam("simulation/magnet/base_direction", base_dir_raw))
    {
        if (base_dir_raw.size() != 3)
        {
            throw std::runtime_error("simulation/magnet/base_direction 必须为长度 3 的数组");
        }
        base_direction_ = Eigen::Vector3d(base_dir_raw[0], base_dir_raw[1], base_dir_raw[2]);
    }
    if (base_direction_.norm() == 0.0)
    {
        throw std::runtime_error("simulation/magnet/base_direction 不可为零向量");
    }
    base_direction_.normalize();

    pnh_.param("simulation/noise/enable", noise_enable_, false);
    pnh_.param("simulation/noise/type", noise_type_, std::string("gaussian"));
    pnh_.param("simulation/noise/mean", noise_mean_, 0.0);
    pnh_.param("simulation/noise/stddev", noise_stddev_, 1.0);
    pnh_.param("simulation/noise/amplitude", noise_amplitude_, 1.0);

    pnh_.param("calibration/full_scale_mT", full_scale_mT_, 3.2);
    pnh_.param("calibration/raw_max", raw_max_, 32767.0);
    if (full_scale_mT_ <= 0.0 || raw_max_ <= 0.0)
    {
        throw std::runtime_error("calibration.* 参数需为正数");
    }

    pnh_.param("tf/enable", publish_tf_, true);
    if (pnh_.hasParam("tf/publish_rate"))
    {
        pnh_.param("tf/publish_rate", tf_publish_rate_, tf_publish_rate_);
    }
}

void MagSensorSimNode::setupPublishers()
{
    if (!topic_raw_.empty())
    {
        raw_pub_ = nh_.advertise<magnet_msgs::MagSensorData>(topic_raw_, 20);
    }
    if (!topic_field_.empty())
    {
        field_pub_ = nh_.advertise<magnet_msgs::MagSensorData>(topic_field_, 20);
    }
}

void MagSensorSimNode::setupTimers()
{
    simulation_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MagSensorSimNode::onSimulationTimer, this);
    if (publish_tf_ && tf_publisher_)
    {
        if (tf_publish_rate_ > 0.0)
        {
            tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_publish_rate_), &MagSensorSimNode::onTfTimer, this);
        }
        else
        {
            tf_publisher_->publishStatic();
        }
    }
}

Eigen::MatrixXd MagSensorSimNode::computeField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                const Eigen::Vector3d &magnet_position,
                                                const Eigen::Vector3d &magnet_direction,
                                                double dipole_strength)
{
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

void MagSensorSimNode::onSimulationTimer(const ros::TimerEvent &)
{
    geometry_msgs::TransformStamped magnet_tf;
    try
    {
        magnet_tf = tf_buffer_.lookupTransform(magnet_parent_frame_, magnet_child_frame_, ros::Time(0), ros::Duration(0.05));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(2.0, "[mag_sensor_sim] 无法查询磁铁 TF (%s->%s): %s", magnet_parent_frame_.c_str(), magnet_child_frame_.c_str(), ex.what());
        return;
    }

    publishReadings(magnet_tf, ros::Time::now());
}

void MagSensorSimNode::onTfTimer(const ros::TimerEvent &)
{
    if (tf_publisher_)
    {
        tf_publisher_->publishDynamic(ros::Time::now());
    }
}

Eigen::Vector3d MagSensorSimNode::sampleNoise()
{
    Eigen::Vector3d noise = Eigen::Vector3d::Zero();
    if (!noise_enable_)
    {
        return noise;
    }

    if (noise_type_ == "gaussian")
    {
        std::normal_distribution<double> dist(noise_mean_, noise_stddev_);
        noise.x() = dist(rng_);
        noise.y() = dist(rng_);
        noise.z() = dist(rng_);
    }
    else if (noise_type_ == "uniform")
    {
        std::uniform_real_distribution<double> dist(-noise_amplitude_, noise_amplitude_);
        noise.x() = dist(rng_);
        noise.y() = dist(rng_);
        noise.z() = dist(rng_);
    }
    else
    {
        ROS_WARN_ONCE("[mag_sensor_sim] 未识别的噪声类型 %s，将忽略噪声", noise_type_.c_str());
    }
    return noise;
}

void MagSensorSimNode::publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp)
{
    const auto &sensors = array_description_.sensors();
    if (sensors.empty())
    {
        ROS_WARN_THROTTLE(5.0, "[mag_sensor_sim] 无可用传感器描述，跳过发布");
        return;
    }

    Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(sensors.size(), 3);

    tf2::Transform parent_array;
    tf2::fromMsg(array_description_.arrayPose(), parent_array);

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
    tf2::Vector3 base_dir(base_direction_.x(), base_direction_.y(), base_direction_.z());
    tf2::Vector3 rotated = tf2::quatRotate(magnet_q, base_dir);
    Eigen::Vector3d magnet_direction(rotated.x(), rotated.y(), rotated.z());

    Eigen::MatrixXd fields_mT = computeField(sensor_positions, magnet_position, magnet_direction.normalized(), dipole_strength_);

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        Eigen::Vector3d noise = sampleNoise();
        Eigen::Vector3d value_mT(fields_mT(static_cast<int>(i), 0),
                                 fields_mT(static_cast<int>(i), 1),
                                 fields_mT(static_cast<int>(i), 2));
        value_mT += noise;

        if (raw_pub_)
        {
            magnet_msgs::MagSensorData msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = sensors[i].frame_id;
            msg.sensor_id = sensors[i].id;
            msg.mag_x = toRawCount(value_mT.x());
            msg.mag_y = toRawCount(value_mT.y());
            msg.mag_z = toRawCount(value_mT.z());
            raw_pub_.publish(msg);
        }

        if (field_pub_)
        {
            magnet_msgs::MagSensorData msg;
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

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_sensor_sim_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    try
    {
        MagSensorSimNode node(nh, pnh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_sensor_sim_node 启动失败: %s", e.what());
        return 1;
    }
    return 0;
}
