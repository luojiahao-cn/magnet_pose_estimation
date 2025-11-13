#include <mag_device_magnet/magnet_node.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mag_device_magnet
{
namespace
{
geometry_msgs::Pose poseFromXyzRpy(const std::vector<double> &xyz, const std::vector<double> &rpy)
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

std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

} // namespace

FrameConfig loadFrameConfig(const mag_core_utils::param::StructReader &root)
{
    if (!root.has("frame"))
    {
        throw std::runtime_error("缺少 frame 配置");
    }
    const auto frame = root.childStruct("frame");
    FrameConfig cfg;
    cfg.parent_frame = frame.requireString("parent");
    cfg.child_frame = frame.requireString("child");
    return cfg;
}

TopicConfig loadTopicConfig(const mag_core_utils::param::StructReader &root, TopicConfig defaults)
{
    if (root.has("topics"))
    {
        const auto topics = root.childStruct("topics");
        defaults.pose_topic = topics.optionalString("pose", defaults.pose_topic);
    }
    return defaults;
}

MotionConfig loadMotionConfig(const mag_core_utils::param::StructReader &root, MotionConfig defaults)
{
    if (root.has("motion"))
    {
        const auto motion = root.childStruct("motion");
        defaults.update_rate_hz = motion.optionalNumber("update_rate_hz", defaults.update_rate_hz);
        defaults.magnetic_strength = motion.optionalNumber("magnetic_strength", defaults.magnetic_strength);
    }
    if (defaults.update_rate_hz <= 0.0)
    {
        throw std::runtime_error("motion/update_rate_hz 需为正数");
    }
    return defaults;
}

TfConfig loadTfConfig(const mag_core_utils::param::StructReader &root, TfConfig defaults)
{
    if (root.has("tf"))
    {
        const auto tf = root.childStruct("tf");
        defaults.enable = tf.optionalBool("enable", defaults.enable);
        defaults.use_static_tf = tf.optionalBool("static", defaults.use_static_tf);
    }
    return defaults;
}

TrajectoryConfig loadTrajectoryConfig(const mag_core_utils::param::StructReader &root)
{
    if (!root.has("trajectory"))
    {
        throw std::runtime_error("缺少 trajectory 配置");
    }
    const auto trajectory = root.childStruct("trajectory");
    auto type = toLower(trajectory.requireString("type"));

    TrajectoryConfig cfg;
    if (type == "static")
    {
        const auto pose_node = trajectory.childStruct("pose");
        const auto xyz = pose_node.requireVector3("xyz");
        const auto rpy = pose_node.requireVector3("rpy");
        cfg.type = TrajectoryType::Static;
        cfg.static_config.pose = poseFromXyzRpy(xyz, rpy);
    }
    else if (type == "circular")
    {
        cfg.type = TrajectoryType::Circular;
        const auto center = trajectory.requireVector3("center");
        cfg.circular_config.center.x = center[0];
        cfg.circular_config.center.y = center[1];
        cfg.circular_config.center.z = center[2];
        cfg.circular_config.radius = trajectory.optionalNumber("radius", cfg.circular_config.radius);
        cfg.circular_config.angular_speed = trajectory.optionalNumber("angular_speed", cfg.circular_config.angular_speed);
        cfg.circular_config.phase = trajectory.optionalNumber("phase", cfg.circular_config.phase);
        if (cfg.circular_config.radius <= 0.0)
        {
            throw std::runtime_error("trajectory/radius 需为正数");
        }
        if (trajectory.has("orientation_rpy"))
        {
            const auto rpy = trajectory.requireVector3("orientation_rpy");
            tf2::Quaternion q;
            q.setRPY(rpy[0], rpy[1], rpy[2]);
            cfg.circular_config.orientation = tf2::toMsg(q);
        }
        else
        {
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, 0.0);
            cfg.circular_config.orientation = tf2::toMsg(q);
        }
    }
    else
    {
        throw std::runtime_error("不支持的 trajectory/type: " + type);
    }

    return cfg;
}

MagnetNode::MagnetNode(ros::NodeHandle nh,
                       ros::NodeHandle pnh,
                       FrameConfig frame_config,
                       MotionConfig motion_config,
                       TopicConfig topic_config,
                       TfConfig tf_config,
                       TrajectoryConfig trajectory_config)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      frame_config_(std::move(frame_config)),
      motion_config_(motion_config),
      topic_config_(std::move(topic_config)),
      tf_config_(tf_config),
      trajectory_config_(std::move(trajectory_config))
{
    setupPublisher();
}

void MagnetNode::start()
{
    // 记录开始时间，驱动时间相关轨迹
    start_time_ = ros::Time::now();

    // 立即推一次数据，避免初始空白
    onUpdate(ros::TimerEvent());

    // 按配置频率定时更新
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / motion_config_.update_rate_hz),
                                    &MagnetNode::onUpdate,
                                    this);
}

void MagnetNode::setupPublisher()
{
    if (!topic_config_.pose_topic.empty())
    {
        pose_pub_ = nh_.advertise<mag_core_msgs::MagnetPose>(topic_config_.pose_topic, 10);
    }
}

void MagnetNode::onUpdate(const ros::TimerEvent &)
{
    const ros::Time stamp = ros::Time::now();
    const double elapsed = std::max(0.0, (stamp - start_time_).toSec());
    const auto sample = sampleTrajectory(elapsed);

    if (tf_config_.enable)
    {
        const bool static_tf = tf_config_.use_static_tf && trajectory_config_.type == TrajectoryType::Static;
        if (static_tf)
        {
            if (!static_sent_)
            {
                publishTransform(stamp, sample.pose, true);
                static_sent_ = true;
            }
        }
        else
        {
            publishTransform(stamp, sample.pose, false);
        }
    }

    publishPose(stamp, sample.pose);
}

TrajectorySample MagnetNode::sampleTrajectory(double elapsed_seconds) const
{
    switch (trajectory_config_.type)
    {
    case TrajectoryType::Static:
        return sampleStatic();
    case TrajectoryType::Circular:
        return sampleCircular(elapsed_seconds);
    default:
        throw std::runtime_error("未知的轨迹类型");
    }
}

TrajectorySample MagnetNode::sampleStatic() const
{
    // 静态轨迹直接返回固定姿态
    TrajectorySample sample;
    sample.pose = trajectory_config_.static_config.pose;
    return sample;
}

TrajectorySample MagnetNode::sampleCircular(double elapsed_seconds) const
{
    // 圆轨迹：二维圆周 + 固定姿态
    const auto &cfg = trajectory_config_.circular_config;
    const double angle = cfg.phase + cfg.angular_speed * elapsed_seconds;

    TrajectorySample sample;
    sample.pose.position.x = cfg.center.x + cfg.radius * std::cos(angle);
    sample.pose.position.y = cfg.center.y + cfg.radius * std::sin(angle);
    sample.pose.position.z = cfg.center.z;
    sample.pose.orientation = cfg.orientation;
    return sample;
}

void MagnetNode::publishPose(const ros::Time &stamp, const geometry_msgs::Pose &pose)
{
    if (!pose_pub_)
    {
        return;
    }
    mag_core_msgs::MagnetPose msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_config_.parent_frame;
    msg.position = pose.position;
    msg.orientation = pose.orientation;
    msg.magnetic_strength = motion_config_.magnetic_strength;
    pose_pub_.publish(msg);
}

void MagnetNode::publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose, bool static_tf)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_config_.parent_frame;
    tf_msg.child_frame_id = frame_config_.child_frame;
    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;

    if (static_tf)
    {
        static_broadcaster_.sendTransform(tf_msg);
    }
    else
    {
        dynamic_broadcaster_.sendTransform(tf_msg);
    }
}

} // namespace mag_device_magnet
