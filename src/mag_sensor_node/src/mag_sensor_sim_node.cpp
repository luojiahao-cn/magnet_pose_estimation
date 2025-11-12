#include <geometry_msgs/Point.h>
#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <mag_sensor_node/mag_sensor_sim_node.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <cmath>
#include <mag_sensor_node/sensor_config.hpp>
#include <random>

using magnet_msgs::MagSensorData;
using magnet_msgs::MagnetPose;

MagSensorSimNode::MagSensorSimNode(ros::NodeHandle &nh)
    : nh_(nh), tf_listener_(tf_buffer_)
{
    ros::NodeHandle pnh("~");
    if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(pnh))
        throw std::runtime_error("sensor config not loaded");

    loadParams();
    setupPublishers();
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MagSensorSimNode::onTimer, this);
}

// 计算磁偶极子模型的磁场（返回单位 mT）
Eigen::MatrixXd MagSensorSimNode::computeMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                       const Eigen::Vector3d &magnetic_position,
                                                       const Eigen::Vector3d &magnetic_direction,
                                                       double magnetic_moment_size)
{
    const double mu_0 = 4 * M_PI * 1e-7;
    Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnetic_position.transpose();
    Eigen::VectorXd r_norm = r_vec.rowwise().norm();
    r_norm = r_norm.array().max(1e-12);
    Eigen::Vector3d m = magnetic_moment_size * magnetic_direction;
    Eigen::VectorXd m_dot_r = r_vec * m;
    Eigen::MatrixXd B =
        (mu_0 / (4 * M_PI)) * (3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5))) -
                               m.transpose().replicate(r_vec.rows(), 1).array().colwise() / r_norm.array().pow(3));
    return B * 1e3;
}

void MagSensorSimNode::loadParams()
{
    ros::NodeHandle pnh("~");

    // Helper: require a parameter or throw with a clear message.
    auto require = [&](const std::string &key, auto &var) {
        if (!pnh.getParam(key, var))
            throw std::runtime_error("缺少必需参数: " + key);
    };

    // Helper: optional 3-vector parameter
    auto optVec3 = [&](const std::string &key, Eigen::Vector3d &out) -> bool {
        std::vector<double> v;
        if (!pnh.getParam(key, v) || v.size() != 3)
            return false;
        out = Eigen::Vector3d(v[0], v[1], v[2]);
        return true;
    };

    // Required parameters
    if (!optVec3("sim_config/magnet/direction", magnetic_direction_))
        throw std::runtime_error("缺少或非法参数: sim_config/magnet/direction");

    require("sensor_config/array/frame_id", frame_id_);
    // Require array parent frame so we can publish array-level TF
    require("sensor_config/array/parent_frame", array_parent_frame_);
    // TF frames for magnet pose lookup. These are required now and must be supplied
    // by the caller under sim_config/magnet_tf.
    require("sim_config/magnet_tf/parent_frame", parent_frame_);
    require("sim_config/magnet_tf/child_frame", child_frame_);
    require("sim_config/magnet/strength", magnet_strength_);

    // Only the update rate is needed by this node (for TF polling).
    require("sim_config/path/update_rate", update_rate_);

    // Noise configuration
    require("sim_config/noise/enable", noise_enable_);
    require("sim_config/noise/type", noise_type_);
    require("sim_config/noise/mean", noise_mean_);
    require("sim_config/noise/stddev", noise_stddev_);
    require("sim_config/noise/amplitude", noise_amplitude_);
}

void MagSensorSimNode::setupPublishers()
{
    std::string magnetic_field_topic;
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("sim_config/topics/magnetic_field_topic", magnetic_field_topic))
        throw std::runtime_error("缺少必需参数: sim_config/topics/magnetic_field_topic");

    magnetic_field_pub_ = nh_.advertise<MagSensorData>(magnetic_field_topic, 25);
    ROS_INFO_STREAM("[mag_sensor_sim] publishing magnetic field on '" << magnetic_field_topic << "'");
}

void MagSensorSimNode::onTimer(const ros::TimerEvent &)
{
    // First publish array and sensor TFs according to configuration so that frames exist
    // even if the magnet TF is not currently available.
    const auto &cfg = mag_sensor_node::SensorConfig::getInstance();
    const auto &sensors = cfg.getAllSensors();
    const geometry_msgs::Pose &array_off = cfg.getArrayOffset();

    ros::Time now = ros::Time::now();

    // publish array frame relative to array_parent_frame_
    geometry_msgs::TransformStamped array_tf;
    array_tf.header.stamp = now;
    array_tf.header.frame_id = array_parent_frame_;
    array_tf.child_frame_id = frame_id_;
    array_tf.transform.translation.x = array_off.position.x;
    array_tf.transform.translation.y = array_off.position.y;
    array_tf.transform.translation.z = array_off.position.z;
    array_tf.transform.rotation = array_off.orientation;
    tf_broadcaster_.sendTransform(array_tf);

    // publish each sensor as child of the array frame
    for (size_t i = 0; i < sensors.size(); ++i)
    {
        geometry_msgs::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = frame_id_; // the array/frame id for sensors
        t.child_frame_id = std::string("sensor_") + std::to_string(sensors[i].id);
        t.transform.translation.x = sensors[i].pose.position.x;
        t.transform.translation.y = sensors[i].pose.position.y;
        t.transform.translation.z = sensors[i].pose.position.z;
        t.transform.rotation = sensors[i].pose.orientation;
        tf_broadcaster_.sendTransform(t);
    }

    // Now attempt to obtain magnet pose from TF between parent_frame_ and child_frame_
    geometry_msgs::TransformStamped tfst;
    try
    {
        // lookup latest transform; allow small timeout
        tfst = tf_buffer_.lookupTransform(parent_frame_, child_frame_, ros::Time(0), ros::Duration(0.1));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN_THROTTLE(2.0, "无法从 TF 获取磁铁位姿 (from '%s' to '%s'): %s", parent_frame_.c_str(), child_frame_.c_str(), ex.what());
        return;
    }

    MagnetPose magnet_pose;
    magnet_pose.header.stamp = now;
    magnet_pose.header.frame_id = parent_frame_;
    magnet_pose.position.x = tfst.transform.translation.x;
    magnet_pose.position.y = tfst.transform.translation.y;
    magnet_pose.position.z = tfst.transform.translation.z;
    magnet_pose.orientation = tfst.transform.rotation;
    magnet_pose.magnetic_strength = magnet_strength_;

    // Compute and publish magnetic field readings based on TF-obtained magnet pose
    publishSensorMagneticFields(magnet_pose);
}

void MagSensorSimNode::publishSensorMagneticFields(const MagnetPose &magnet_pose)
{
    const auto &cfg = mag_sensor_node::SensorConfig::getInstance();
    const auto &sensors = cfg.getAllSensors();
    const geometry_msgs::Pose &array_off = cfg.getArrayOffset();
    if (sensors.empty())
    {
        ROS_WARN_THROTTLE(5.0, "未找到传感器配置，跳过磁场计算");
        return;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 3> positions(sensors.size(), 3);
    for (size_t i = 0; i < sensors.size(); ++i)
    {
        positions(i, 0) = sensors[i].pose.position.x;
        positions(i, 1) = sensors[i].pose.position.y;
        positions(i, 2) = sensors[i].pose.position.z;
    }
    
    Eigen::Vector3d magnet_position(magnet_pose.position.x, magnet_pose.position.y, magnet_pose.position.z);
    Eigen::Vector3d base_direction(0, 0, 1);
    Eigen::Quaterniond q(
        magnet_pose.orientation.w, magnet_pose.orientation.x, magnet_pose.orientation.y, magnet_pose.orientation.z);
    Eigen::Vector3d magnet_direction = q * base_direction;
    Eigen::MatrixXd fields = computeMagneticField(positions, magnet_position, magnet_direction, magnet_strength_);

    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> gaussian_dist(noise_mean_, noise_stddev_);
    std::uniform_real_distribution<double> uniform_dist(-noise_amplitude_, noise_amplitude_);

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        MagSensorData msg;
        msg.header.stamp = magnet_pose.header.stamp;
        msg.header.frame_id = magnet_pose.header.frame_id;
        msg.sensor_id = sensors[i].id;
        Eigen::Vector3d noise = Eigen::Vector3d::Zero();
        if (noise_enable_)
        {
            if (noise_type_ == "gaussian")
            {
                noise.x() = gaussian_dist(generator);
                noise.y() = gaussian_dist(generator);
                noise.z() = gaussian_dist(generator);
            }
            else if (noise_type_ == "uniform")
            {
                noise.x() = uniform_dist(generator);
                noise.y() = uniform_dist(generator);
                noise.z() = uniform_dist(generator);
            }
        }
        msg.mag_x = fields(i, 0) + noise.x();
        msg.mag_y = fields(i, 1) + noise.y();
        msg.mag_z = fields(i, 2) + noise.z();
        magnetic_field_pub_.publish(msg);
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_sensor_sim_node");
    ros::NodeHandle nh;
    try
    {
        MagSensorSimNode node(nh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_sensor_sim_node 退出: %s", e.what());
        return 1;
    }
    return 0;
}
