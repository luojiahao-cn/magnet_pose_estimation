#include <mag_device_magnet/magnet_node.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mag_device_magnet
{
namespace
{
geometry_msgs::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

} // namespace

MagnetNode::MagnetNode(ros::NodeHandle nh,
                       ros::NodeHandle pnh,
                       FrameConfig frame_config,
                       MotionConfig motion_config,
                       TopicConfig topic_config,
                       TfConfig tf_config,
                                             TrajectoryConfig trajectory_config,
                                             OrientationConfig orientation_config)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      frame_config_(std::move(frame_config)),
      motion_config_(motion_config),
      topic_config_(std::move(topic_config)),
      tf_config_(tf_config),
            trajectory_config_(std::move(trajectory_config)),
            orientation_config_(std::move(orientation_config))
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
    TrajectorySample sample = sampleTrajectory(elapsed);
    sample.pose.orientation = sampleOrientation(elapsed, sample.pose.orientation);

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
    case TrajectoryType::Rectangular:
        return sampleRectangular(elapsed_seconds);
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

TrajectorySample MagnetNode::sampleRectangular(double elapsed_seconds) const
{
    const auto &cfg = trajectory_config_.rectangular_config;
    const double width = cfg.width;
    const double height = cfg.height;
    const double left = cfg.center.x - width / 2.0;
    const double right = cfg.center.x + width / 2.0;
    const double bottom = cfg.center.y - height / 2.0;
    const double top = cfg.center.y + height / 2.0;

    const double distance = cfg.velocity * elapsed_seconds;
    const double progress = std::fmod(distance, cfg.perimeter);

    TrajectorySample sample;
    double remaining = progress;
    if (remaining <= width)
    {
        const double ratio = remaining / width;
        sample.pose.position.x = left + ratio * width;
        sample.pose.position.y = bottom;
    }
    else if (remaining <= width + height)
    {
        remaining -= width;
        const double ratio = remaining / height;
        sample.pose.position.x = right;
        sample.pose.position.y = bottom + ratio * height;
    }
    else if (remaining <= 2 * width + height)
    {
        remaining -= (width + height);
        const double ratio = remaining / width;
        sample.pose.position.x = right - ratio * width;
        sample.pose.position.y = top;
    }
    else
    {
        remaining -= (2 * width + height);
        const double ratio = remaining / height;
        sample.pose.position.x = left;
        sample.pose.position.y = top - ratio * height;
    }
    sample.pose.position.z = cfg.z;
    sample.pose.orientation = cfg.orientation;
    return sample;
}

geometry_msgs::Quaternion MagnetNode::sampleOrientation(double elapsed_seconds,
                                                         const geometry_msgs::Quaternion &base_orientation) const
{
    switch (orientation_config_.mode)
    {
    case OrientationMode::FromTrajectory:
        return base_orientation;
    case OrientationMode::Fixed:
        return orientation_config_.fixed_orientation;
    case OrientationMode::Spin:
    {
        const double angle = orientation_config_.angular_velocity * elapsed_seconds;
        double roll = orientation_config_.initial_roll + (orientation_config_.axis_x ? angle : 0.0);
        double pitch = orientation_config_.initial_pitch + (orientation_config_.axis_y ? angle : 0.0);
        double yaw = orientation_config_.initial_yaw + (orientation_config_.axis_z ? angle : 0.0);
        return quaternionFromRpy(roll, pitch, yaw);
    }
    default:
        return base_orientation;
    }
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
