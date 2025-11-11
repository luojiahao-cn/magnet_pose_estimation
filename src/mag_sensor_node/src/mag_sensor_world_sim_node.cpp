#include <mag_sensor_node/mag_sensor_world_sim_node.hpp>

#include <mag_sensor_node/sensor_config.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Geometry>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

using magnet_msgs::MagSensorData;
using magnet_msgs::MagnetPose;
using mag_sensor_node::SensorConfig;

namespace
{
    constexpr double kMu0 = 4.0 * M_PI * 1e-7;
}

Eigen::MatrixXd MagSensorWorldSimNode::computeMagneticField(const Eigen::Matrix<double, -1, 3> &sensor_positions,
                                                            const Eigen::Vector3d &magnetic_position,
                                                            const Eigen::Vector3d &magnetic_direction,
                                                            double magnetic_moment_size)
{
    Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnetic_position.transpose();
    Eigen::VectorXd r_norm = r_vec.rowwise().norm();
    r_norm = r_norm.array().max(1e-12);
    Eigen::Vector3d m = magnetic_moment_size * magnetic_direction;
    Eigen::VectorXd m_dot_r = r_vec * m;
    Eigen::MatrixXd B =
        (kMu0 / (4.0 * M_PI)) * (3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5))) -
                                  m.transpose().replicate(r_vec.rows(), 1).array().colwise() / r_norm.array().pow(3));
    return B * 1e3;
}

MagSensorWorldSimNode::MagSensorWorldSimNode(ros::NodeHandle &nh) : nh_(nh)
{
    ros::NodeHandle pnh("~");
    if (!SensorConfig::getInstance().loadConfig(pnh))
    {
        throw std::runtime_error("sensor config not loaded");
    }
    loadParams();
    setupPublishers();
    initializeMotionSystem();
    timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &MagSensorWorldSimNode::onTimer, this);
}

void MagSensorWorldSimNode::loadParams()
{
    ros::NodeHandle pnh("~");
    auto require = [&](const std::string &key, auto &var)
    {
        if (!pnh.getParam(key, var))
        {
            throw std::runtime_error("缺少必需参数: " + key);
        }
    };

    if (!pnh.getParam("sim_config/world/frame_id", world_frame_id_))
    {
        throw std::runtime_error("缺少必需参数: sim_config/world/frame_id");
    }

    pnh.param<std::string>("sensor_config/array/frame_id", array_frame_id_, array_frame_id_);
    pnh.param("sim_config/sensor_array/publish_tf", publish_tf_, false);

    require("sim_config/magnet/strength", magnet_strength_);
    std::vector<double> direction;
    if (!pnh.getParam("sim_config/magnet/direction", direction) || direction.size() != 3)
    {
        throw std::runtime_error("缺少或非法参数: sim_config/magnet/direction");
    }
    magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);

    require("sim_config/path/width", rect_width_);
    require("sim_config/path/height", rect_height_);
    require("sim_config/path/z", rect_z_);
    require("sim_config/path/center/x", x_center_);
    require("sim_config/path/center/y", y_center_);
    require("sim_config/path/update_rate", update_rate_);
    require("sim_config/path/velocity", translation_velocity_);

    require("sim_config/motion/mode", motion_mode_);
    require("sim_config/motion/axis", rotation_axis_);
    require("sim_config/motion/angular_velocity", angular_velocity_);
    require("sim_config/motion/initial_roll", initial_roll_);
    require("sim_config/motion/initial_pitch", initial_pitch_);
    require("sim_config/motion/initial_yaw", initial_yaw_);

    std::vector<double> static_pos;
    std::vector<double> static_orient;
    if (!pnh.getParam("sim_config/motion/static_position", static_pos) || static_pos.size() != 3)
    {
        throw std::runtime_error("缺少或非法参数: sim_config/motion/static_position");
    }
    if (!pnh.getParam("sim_config/motion/static_orientation", static_orient) || static_orient.size() != 3)
    {
        throw std::runtime_error("缺少或非法参数: sim_config/motion/static_orientation");
    }
    static_position_ = Eigen::Vector3d(static_pos[0], static_pos[1], static_pos[2]);
    static_orientation_ = Eigen::Vector3d(static_orient[0], static_orient[1], static_orient[2]);

    require("sim_config/noise/enable", noise_enable_);
    require("sim_config/noise/type", noise_type_);
    require("sim_config/noise/mean", noise_mean_);
    require("sim_config/noise/stddev", noise_stddev_);
    require("sim_config/noise/amplitude", noise_amplitude_);

    // 运动模式解析
    translation_enable_ = false;
    rotation_enable_ = false;
    follow_path_ = true;
    if (motion_mode_ == "static")
    {
        translation_enable_ = false;
        rotation_enable_ = false;
    }
    else if (motion_mode_ == "translate_only")
    {
        translation_enable_ = true;
        rotation_enable_ = false;
        follow_path_ = true;
    }
    else if (motion_mode_ == "rotate_only")
    {
        translation_enable_ = false;
        rotation_enable_ = true;
    }
    else if (motion_mode_ == "translate_and_rotate")
    {
        translation_enable_ = true;
        rotation_enable_ = true;
        follow_path_ = true;
    }
    else
    {
        throw std::runtime_error("非法 motion mode: " + motion_mode_);
    }

    std::string array_source;
    require("sim_config/sensor_array/source", array_source);
    if (array_source == "static")
    {
        array_pose_source_ = ArrayPoseSource::STATIC;
        std::vector<double> position;
        std::vector<double> orientation;
        if (!pnh.getParam("sim_config/sensor_array/static_pose/position", position) || position.size() != 3)
        {
            throw std::runtime_error("缺少或非法参数: sim_config/sensor_array/static_pose/position");
        }
        if (!pnh.getParam("sim_config/sensor_array/static_pose/orientation", orientation) || orientation.size() != 3)
        {
            throw std::runtime_error("缺少或非法参数: sim_config/sensor_array/static_pose/orientation");
        }
        geometry_msgs::Pose pose;
        pose.position.x = position[0];
        pose.position.y = position[1];
        pose.position.z = position[2];
        tf2::Quaternion q;
        q.setRPY(orientation[0], orientation[1], orientation[2]);
        pose.orientation = tf2::toMsg(q);
        {
            std::lock_guard<std::mutex> lock(array_pose_mutex_);
            latest_array_pose_ = pose;
            latest_array_pose_stamp_ = ros::Time::now();
            array_pose_ready_ = true;
        }
    }
    else if (array_source == "topic")
    {
        array_pose_source_ = ArrayPoseSource::TOPIC;
        std::string topic_name;
        require("sim_config/sensor_array/topic/name", topic_name);
        double timeout_sec = 0.0;
        if (pnh.getParam("sim_config/sensor_array/topic/timeout", timeout_sec) && timeout_sec > 0.0)
        {
            array_pose_timeout_ = ros::Duration(timeout_sec);
        }
        array_pose_sub_ = nh_.subscribe(topic_name, 1, &MagSensorWorldSimNode::handleArrayPose, this);
    }
    else
    {
        throw std::runtime_error("非法参数 sim_config/sensor_array/source: " + array_source);
    }
}

void MagSensorWorldSimNode::setupPublishers()
{
    ros::NodeHandle pnh("~");
    std::string magnet_pose_topic;
    std::string magnetic_field_topic;
    if (!pnh.getParam("sim_config/topics/magnet_pose_topic", magnet_pose_topic))
    {
        throw std::runtime_error("缺少必需参数: sim_config/topics/magnet_pose_topic");
    }
    if (!pnh.getParam("sim_config/topics/magnetic_field_topic", magnetic_field_topic))
    {
        throw std::runtime_error("缺少必需参数: sim_config/topics/magnetic_field_topic");
    }

    magnet_pose_pub_ = nh_.advertise<MagnetPose>(magnet_pose_topic, 25);
    magnetic_field_pub_ = nh_.advertise<MagSensorData>(magnetic_field_topic, 25);
    ROS_INFO_STREAM("[mag_sensor_world_sim] topics: magnet_pose='" << magnet_pose_topic << "', magnetic_field='"
                                                                  << magnetic_field_topic << "'");
}

void MagSensorWorldSimNode::initializeMotionSystem()
{
    path_total_length_ = 2.0 * (rect_width_ + rect_height_);
    start_time_ = ros::Time::now();
    current_position_.x = x_center_ - rect_width_ / 2.0;
    current_position_.y = y_center_ - rect_height_ / 2.0;
    current_position_.z = rect_z_;
}

void MagSensorWorldSimNode::onTimer(const ros::TimerEvent &)
{
    MagnetPose magnet_pose;
    magnet_pose.header.stamp = ros::Time::now();
    magnet_pose.header.frame_id = world_frame_id_;
    updateMagnetPose(magnet_pose);
    magnet_pose.magnetic_strength = magnet_strength_;
    magnet_pose_pub_.publish(magnet_pose);
    publishSensorMagneticFields(magnet_pose);
}

void MagSensorWorldSimNode::updateMagnetPose(MagnetPose &magnet_pose)
{
    updateMagnetPosition(magnet_pose);
    updateMagnetOrientation(magnet_pose);
}

void MagSensorWorldSimNode::updateMagnetPosition(MagnetPose &magnet_pose)
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

MagSensorWorldSimNode::MotionType MagSensorWorldSimNode::getMotionType() const
{
    if (motion_mode_ == "static")
    {
        return MotionType::STATIC;
    }
    if (translation_enable_ && follow_path_)
    {
        return MotionType::TRANSLATE_WITH_PATH;
    }
    if (translation_enable_)
    {
        return MotionType::TRANSLATE_STATIONARY;
    }
    return MotionType::CENTER_FIXED;
}

void MagSensorWorldSimNode::updateMagnetOrientation(MagnetPose &magnet_pose)
{
    double roll;
    double pitch;
    double yaw;
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
    magnet_pose.orientation = tf2::toMsg(q);
}

void MagSensorWorldSimNode::calculateDynamicOrientation(double &roll, double &pitch, double &yaw)
{
    static ros::Time rotation_start_time = ros::Time::now();
    double elapsed_time = (ros::Time::now() - rotation_start_time).toSec();
    double rotation_angle = elapsed_time * angular_velocity_;
    roll = initial_roll_;
    pitch = initial_pitch_;
    yaw = initial_yaw_;
    if (rotation_axis_ == "x" || rotation_axis_ == "xyz")
    {
        roll += rotation_angle;
    }
    if (rotation_axis_ == "y" || rotation_axis_ == "xyz")
    {
        pitch += rotation_angle;
    }
    if (rotation_axis_ == "z" || rotation_axis_ == "xyz")
    {
        yaw += rotation_angle;
    }
}

geometry_msgs::Point MagSensorWorldSimNode::calculatePositionFromVelocity(double elapsed_time)
{
    geometry_msgs::Point position;
    double distance_traveled = translation_velocity_ * elapsed_time;
    double distance_on_path = std::fmod(distance_traveled, path_total_length_);
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

bool MagSensorWorldSimNode::getCurrentArrayPose(geometry_msgs::Pose &pose) const
{
    std::lock_guard<std::mutex> lock(array_pose_mutex_);
    if (!array_pose_ready_)
    {
        return false;
    }
    if (array_pose_source_ == ArrayPoseSource::TOPIC && array_pose_timeout_.toSec() > 0.0)
    {
        const ros::Duration age = ros::Time::now() - latest_array_pose_stamp_;
        if (age > array_pose_timeout_)
        {
            return false;
        }
    }
    pose = latest_array_pose_;
    return true;
}

void MagSensorWorldSimNode::publishSensorMagneticFields(const MagnetPose &magnet_pose)
{
    const auto &cfg = SensorConfig::getInstance();
    const auto &sensors = cfg.getAllSensors();
    if (sensors.empty())
    {
        ROS_WARN_THROTTLE(5.0, "未找到传感器配置，跳过磁场计算");
        return;
    }
    geometry_msgs::Pose array_pose_world;
    if (!getCurrentArrayPose(array_pose_world))
    {
        ROS_WARN_THROTTLE(5.0, "[mag_sensor_world_sim] 当前无法获取传感器阵列在 world 下的位姿");
        return;
    }

    if (publish_tf_)
    {
        publishArrayTf(array_pose_world, magnet_pose.header.stamp);
    }

    tf2::Transform array_tf;
    tf2::fromMsg(array_pose_world, array_tf);

    std::vector<geometry_msgs::Pose> sensor_world_poses;
    sensor_world_poses.reserve(sensors.size());
    Eigen::Matrix<double, Eigen::Dynamic, 3> positions(sensors.size(), 3);
    for (size_t i = 0; i < sensors.size(); ++i)
    {
        tf2::Transform sensor_tf;
        tf2::fromMsg(sensors[i].pose, sensor_tf);
    tf2::Transform world_sensor_tf = array_tf * sensor_tf;
    geometry_msgs::Pose world_pose;
    tf2::toMsg(world_sensor_tf, world_pose);
        positions(i, 0) = world_pose.position.x;
        positions(i, 1) = world_pose.position.y;
        positions(i, 2) = world_pose.position.z;
        sensor_world_poses.push_back(world_pose);
    }

    Eigen::Vector3d magnet_position(magnet_pose.position.x, magnet_pose.position.y, magnet_pose.position.z);
    Eigen::Quaterniond q(magnet_pose.orientation.w, magnet_pose.orientation.x, magnet_pose.orientation.y,
                        magnet_pose.orientation.z);
    Eigen::Vector3d base_direction(0.0, 0.0, 1.0);
    Eigen::Vector3d magnet_direction = q * base_direction;
    Eigen::MatrixXd fields = computeMagneticField(positions, magnet_position, magnet_direction, magnet_strength_);

    static std::default_random_engine generator(std::random_device{}());
    std::normal_distribution<double> gaussian_dist(noise_mean_, noise_stddev_);
    std::uniform_real_distribution<double> uniform_dist(-noise_amplitude_, noise_amplitude_);

    for (size_t i = 0; i < sensors.size(); ++i)
    {
        MagSensorData msg;
        msg.header.stamp = magnet_pose.header.stamp;
        msg.header.frame_id = world_frame_id_;
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

void MagSensorWorldSimNode::handleArrayPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!msg->header.frame_id.empty() && msg->header.frame_id != world_frame_id_)
    {
        ROS_WARN_THROTTLE(5.0, "[mag_sensor_world_sim] 收到的传感器阵列位姿帧为 '%s'，预期 '%s'",
                          msg->header.frame_id.c_str(), world_frame_id_.c_str());
    }
    std::lock_guard<std::mutex> lock(array_pose_mutex_);
    latest_array_pose_ = msg->pose;
    latest_array_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    array_pose_ready_ = true;
}

void MagSensorWorldSimNode::publishArrayTf(const geometry_msgs::Pose &array_pose_world, const ros::Time &stamp)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = world_frame_id_;
    tf_msg.child_frame_id = array_frame_id_;
    tf_msg.transform.translation.x = array_pose_world.position.x;
    tf_msg.transform.translation.y = array_pose_world.position.y;
    tf_msg.transform.translation.z = array_pose_world.position.z;
    tf_msg.transform.rotation = array_pose_world.orientation;
    tf_broadcaster_.sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_sensor_world_sim_node");
    ros::NodeHandle nh;
    try
    {
        MagSensorWorldSimNode node(nh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_sensor_world_sim_node 退出: %s", e.what());
        return 1;
    }
    return 0;
}
