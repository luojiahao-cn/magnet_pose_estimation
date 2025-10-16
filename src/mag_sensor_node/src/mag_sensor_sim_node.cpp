#include <geometry_msgs/Point.h>
#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <mag_sensor_node/mag_sensor_sim_node.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <cmath>
#include <mag_sensor_node/sensor_config.hpp>
#include <random>

MagSensorSimNode::MagSensorSimNode(ros::NodeHandle &nh) : nh_(nh)
{
    if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(nh_))
    {
        ROS_ERROR("传感器配置加载失败");
        throw std::runtime_error("sensor config not loaded");
    }
    loadParams();
    setupPublishers();
    initializeMotionSystem();
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
    return B * 1e3; // 转为 mT
}

void MagSensorSimNode::loadParams()
{
    nh_.param<double>("sim_config/magnet/strength", magnet_strength_, 10.0);
    std::vector<double> direction;
    nh_.param("sim_config/magnet/direction", direction, std::vector<double>{0, 0, 1});
    magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);

    nh_.param<double>("sim_config/path/width", rect_width_, 0.03);
    nh_.param<double>("sim_config/path/height", rect_height_, 0.03);
    nh_.param<double>("sim_config/path/z", rect_z_, 0.03);
    nh_.param<double>("sim_config/path/center/x", x_center_, 0.02);
    nh_.param<double>("sim_config/path/center/y", y_center_, 0.02);
    nh_.param<double>("sim_config/path/update_rate", update_rate_, 100.0);
    nh_.param<double>("sim_config/path/velocity", translation_velocity_, 0.01);

    nh_.param<std::string>("sim_config/motion/mode", motion_mode_, std::string("translate_only"));
    nh_.param<std::string>("sim_config/motion/axis", rotation_axis_, std::string("z"));
    nh_.param<double>("sim_config/motion/angular_velocity", angular_velocity_, 0.1);
    nh_.param<double>("sim_config/motion/initial_roll", initial_roll_, 0.0);
    nh_.param<double>("sim_config/motion/initial_pitch", initial_pitch_, 0.0);
    nh_.param<double>("sim_config/motion/initial_yaw", initial_yaw_, 0.0);

    std::vector<double> static_pos{0.02, 0.02, 0.03};
    std::vector<double> static_orient{0.0, 0.5, 0.0};
    nh_.param("sim_config/motion/static_position", static_pos, static_pos);
    nh_.param("sim_config/motion/static_orientation", static_orient, static_orient);
    static_position_ = Eigen::Vector3d(static_pos[0], static_pos[1], static_pos[2]);
    static_orientation_ = Eigen::Vector3d(static_orient[0], static_orient[1], static_orient[2]);

    nh_.param<bool>("sim_config/noise/enable", noise_enable_, false);
    nh_.param<std::string>("sim_config/noise/type", noise_type_, std::string("gaussian"));
    nh_.param<double>("sim_config/noise/mean", noise_mean_, 0.0);
    nh_.param<double>("sim_config/noise/stddev", noise_stddev_, 1.0);
    nh_.param<double>("sim_config/noise/amplitude", noise_amplitude_, 1.0);
}

void MagSensorSimNode::setupPublishers()
{
    std::string magnet_pose_topic = "/magnet_pose/simulation";
    std::string magnetic_field_topic = "/magnetic_field/raw_data";
    nh_.param<std::string>("sim_config/topics/magnet_pose_topic", magnet_pose_topic, magnet_pose_topic);
    nh_.param<std::string>("sim_config/topics/magnetic_field_topic", magnetic_field_topic, magnetic_field_topic);

    magnet_pose_pub_ = nh_.advertise<mag_sensor_node::MagnetPose>(magnet_pose_topic, 25);
    magnetic_field_pub_ = nh_.advertise<mag_sensor_node::MagSensorData>(magnetic_field_topic, 25);
    ROS_INFO_STREAM("[mag_sensor_sim] topics: magnet_pose='" << magnet_pose_topic
                    << "', magnetic_field='" << magnetic_field_topic << "'");
}

void MagSensorSimNode::initializeMotionSystem()
{
    path_total_length_ = 2.0 * (rect_width_ + rect_height_);
    start_time_ = ros::Time::now();
    current_position_.x = x_center_ - rect_width_ / 2.0;
    current_position_.y = y_center_ - rect_height_ / 2.0;
    current_position_.z = rect_z_;
}

void MagSensorSimNode::onTimer(const ros::TimerEvent &)
{
    mag_sensor_node::MagnetPose magnet_pose;
    magnet_pose.header.stamp = ros::Time::now();
    magnet_pose.header.frame_id = "sensor_base_frame";
    updateMagnetPose(magnet_pose);
    magnet_pose.magnetic_strength = magnet_strength_;
    magnet_pose_pub_.publish(magnet_pose);
    publishSensorMagneticFields(magnet_pose);
}

void MagSensorSimNode::updateMagnetPose(mag_sensor_node::MagnetPose &magnet_pose)
{
    updateMagnetPosition(magnet_pose);
    updateMagnetOrientation(magnet_pose);
}

void MagSensorSimNode::updateMagnetPosition(mag_sensor_node::MagnetPose &magnet_pose)
{
    switch (getMotionType())
    {
    case MotionType::STATIC:
        magnet_pose.position.x = static_position_.x();
        magnet_pose.position.y = static_position_.y();
        magnet_pose.position.z = static_position_.z();
        break;
    case MotionType::TRANSLATE_WITH_PATH:
    {
        double elapsed_time = (ros::Time::now() - start_time_).toSec();
        current_position_ = calculatePositionFromVelocity(elapsed_time);
        magnet_pose.position = current_position_;
        break;
    }
    case MotionType::TRANSLATE_STATIONARY:
        magnet_pose.position.x = x_center_ - rect_width_ / 2.0;
        magnet_pose.position.y = y_center_ - rect_height_ / 2.0;
        magnet_pose.position.z = rect_z_;
        break;
    case MotionType::CENTER_FIXED:
    default:
        magnet_pose.position.x = x_center_;
        magnet_pose.position.y = y_center_;
        magnet_pose.position.z = rect_z_;
        break;
    }
}

MagSensorSimNode::MotionType MagSensorSimNode::getMotionType() const
{
    if (motion_mode_ == "static")
        return MotionType::STATIC;
    if (translation_enable_ && follow_path_)
        return MotionType::TRANSLATE_WITH_PATH;
    if (translation_enable_)
        return MotionType::TRANSLATE_STATIONARY;
    return MotionType::CENTER_FIXED;
}

void MagSensorSimNode::updateMagnetOrientation(mag_sensor_node::MagnetPose &magnet_pose)
{
    double roll, pitch, yaw;
    if (motion_mode_ == "static")
    {
        roll = static_orientation_.x();
        pitch = static_orientation_.y();
        yaw = static_orientation_.z();
    }
    else if (rotation_enable_)
    {
        calculateDynamicOrientation(roll, pitch, yaw);
    }
    else
    {
        roll = initial_roll_;
        pitch = initial_pitch_;
        yaw = initial_yaw_;
    }
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    magnet_pose.orientation.x = q.x();
    magnet_pose.orientation.y = q.y();
    magnet_pose.orientation.z = q.z();
    magnet_pose.orientation.w = q.w();
}

void MagSensorSimNode::calculateDynamicOrientation(double &roll, double &pitch, double &yaw)
{
    static ros::Time rotation_start_time = ros::Time::now();
    double elapsed_time = (ros::Time::now() - rotation_start_time).toSec();
    double rotation_angle = elapsed_time * angular_velocity_;
    roll = initial_roll_;
    pitch = initial_pitch_;
    yaw = initial_yaw_;
    if (rotation_axis_ == "x" || rotation_axis_ == "xyz")
        roll += rotation_angle;
    if (rotation_axis_ == "y" || rotation_axis_ == "xyz")
        pitch += rotation_angle;
    if (rotation_axis_ == "z" || rotation_axis_ == "xyz")
        yaw += rotation_angle;
}

geometry_msgs::Point MagSensorSimNode::calculatePositionFromVelocity(double elapsed_time)
{
    geometry_msgs::Point position;
    double distance_traveled = translation_velocity_ * elapsed_time;
    double distance_on_path = fmod(distance_traveled, path_total_length_);
    double bottom_edge = rect_width_;
    double right_edge = rect_height_;
    double top_edge = rect_width_;
    double left_edge = rect_height_;
    double left = x_center_ - rect_width_ / 2.0;
    double right = x_center_ + rect_width_ / 2.0;
    double bottom = y_center_ - rect_height_ / 2.0;
    double top = y_center_ + rect_height_ / 2.0;
    position.z = rect_z_;
    if (distance_on_path <= bottom_edge)
    {
        double ratio = distance_on_path / bottom_edge;
        position.x = left + ratio * rect_width_;
        position.y = bottom;
    }
    else if (distance_on_path <= bottom_edge + right_edge)
    {
        double ratio = (distance_on_path - bottom_edge) / right_edge;
        position.x = right;
        position.y = bottom + ratio * rect_height_;
    }
    else if (distance_on_path <= bottom_edge + right_edge + top_edge)
    {
        double ratio = (distance_on_path - bottom_edge - right_edge) / top_edge;
        position.x = right - ratio * rect_width_;
        position.y = top;
    }
    else
    {
        double ratio = (distance_on_path - bottom_edge - right_edge - top_edge) / left_edge;
        position.x = left;
        position.y = top - ratio * rect_height_;
    }
    return position;
}

void MagSensorSimNode::publishSensorMagneticFields(const mag_sensor_node::MagnetPose &magnet_pose)
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
        mag_sensor_node::MagSensorData msg;
        msg.header.stamp = magnet_pose.header.stamp;
        msg.header.frame_id = magnet_pose.header.frame_id;  // Set frame_id to match magnet pose
        msg.sensor_id = sensors[i].id;
        // 组合 array_offset 与局部传感器位姿：array_off * sensor.pose
        {
            tf2::Transform T_off, T_s;
            tf2::fromMsg(array_off, T_off);
            tf2::fromMsg(sensors[i].pose, T_s);
            tf2::Transform T = T_off * T_s;
            geometry_msgs::Pose pose;
            pose.position.x = T.getOrigin().x();
            pose.position.y = T.getOrigin().y();
            pose.position.z = T.getOrigin().z();
            pose.orientation = tf2::toMsg(T.getRotation());
            msg.sensor_pose = pose;
        }
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
