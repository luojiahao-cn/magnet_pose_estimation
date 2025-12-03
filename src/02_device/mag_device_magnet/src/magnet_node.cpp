/**
 * @file magnet_node.cpp
 * @brief 磁体设备节点实现
 * 
 * 实现磁体设备 ROS 节点的核心功能，包括轨迹生成、姿态采样和数据发布
 */

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
/**
 * @brief 从 RPY 欧拉角创建四元数
 * 
 * @param roll 滚转角（弧度）
 * @param pitch 俯仰角（弧度）
 * @param yaw 偏航角（弧度）
 * @return geometry_msgs::Quaternion 四元数消息
 */
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
    // 记录开始时间，用于驱动时间相关的轨迹
    start_time_ = ros::Time::now();

    // 立即发布一次数据，避免初始空白期
    onUpdate(ros::TimerEvent());

    // 按配置的频率创建定时器进行周期性更新
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / motion_config_.update_rate_hz),
                                    &MagnetNode::onUpdate,
                                    this);
}

void MagnetNode::setupPublisher()
{
    // 如果配置了话题名称，则创建发布器
    if (!topic_config_.pose_topic.empty())
    {
        pose_pub_ = nh_.advertise<mag_core_msgs::MagnetPose>(topic_config_.pose_topic, 10);
    }
}

void MagnetNode::onUpdate(const ros::TimerEvent &)
{
    const ros::Time stamp = ros::Time::now();
    const double elapsed = std::max(0.0, (stamp - start_time_).toSec());
    
    // 采样轨迹得到位置和基础姿态
    TrajectorySample sample = sampleTrajectory(elapsed);
    
    // 根据姿态配置模式采样最终姿态
    sample.pose.orientation = sampleOrientation(elapsed, sample.pose.orientation);

    // 如果启用了 TF 发布
    if (tf_config_.enable)
    {
        // 判断是否使用静态 TF（仅当配置要求且轨迹为静态时）
        const bool static_tf = tf_config_.use_static_tf && trajectory_config_.type == TrajectoryType::Static;
        if (static_tf)
        {
            // 静态 TF 只需发送一次
            if (!static_sent_)
            {
                publishTransform(stamp, sample.pose, true);
                static_sent_ = true;
            }
        }
        else
        {
            // 动态 TF 每次更新都发布
            publishTransform(stamp, sample.pose, false);
        }
    }

    // 发布姿态消息
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
    // 静态轨迹直接返回配置中的固定姿态
    TrajectorySample sample;
    sample.pose = trajectory_config_.static_config.pose;
    return sample;
}

TrajectorySample MagnetNode::sampleCircular(double elapsed_seconds) const
{
    // 圆形轨迹：在二维平面上做圆周运动，姿态固定
    const auto &cfg = trajectory_config_.circular_config;
    
    // 计算当前角度（初始相位 + 角速度 * 时间）
    const double angle = cfg.phase + cfg.angular_speed * elapsed_seconds;

    TrajectorySample sample;
    // 根据角度计算圆周上的位置
    sample.pose.position.x = cfg.center.x + cfg.radius * std::cos(angle);
    sample.pose.position.y = cfg.center.y + cfg.radius * std::sin(angle);
    sample.pose.position.z = cfg.center.z;
    sample.pose.orientation = cfg.orientation;
    return sample;
}

TrajectorySample MagnetNode::sampleRectangular(double elapsed_seconds) const
{
    const auto &cfg = trajectory_config_.rectangular_config;
    
    // 计算矩形边界
    const double width = cfg.width;
    const double height = cfg.height;
    const double left = cfg.center.x - width / 2.0;
    const double right = cfg.center.x + width / 2.0;
    const double bottom = cfg.center.y - height / 2.0;
    const double top = cfg.center.y + height / 2.0;

    // 计算已走过的距离和当前在周长上的位置
    const double distance = cfg.velocity * elapsed_seconds;
    const double progress = std::fmod(distance, cfg.perimeter);

    TrajectorySample sample;
    double remaining = progress;
    
    // 根据当前位置判断在矩形的哪条边上
    // 第一条边：从 (left, bottom) 到 (right, bottom)
    if (remaining <= width)
    {
        const double ratio = remaining / width;
        sample.pose.position.x = left + ratio * width;
        sample.pose.position.y = bottom;
    }
    // 第二条边：从 (right, bottom) 到 (right, top)
    else if (remaining <= width + height)
    {
        remaining -= width;
        const double ratio = remaining / height;
        sample.pose.position.x = right;
        sample.pose.position.y = bottom + ratio * height;
    }
    // 第三条边：从 (right, top) 到 (left, top)
    else if (remaining <= 2 * width + height)
    {
        remaining -= (width + height);
        const double ratio = remaining / width;
        sample.pose.position.x = right - ratio * width;
        sample.pose.position.y = top;
    }
    // 第四条边：从 (left, top) 到 (left, bottom)
    else
    {
        remaining -= (2 * width + height);
        const double ratio = remaining / height;
        sample.pose.position.x = left;
        sample.pose.position.y = top - ratio * height;
    }
    sample.pose.position.z = cfg.center.z;
    sample.pose.orientation = cfg.orientation;
    return sample;
}

geometry_msgs::Quaternion MagnetNode::sampleOrientation(double elapsed_seconds,
                                                         const geometry_msgs::Quaternion &base_orientation) const
{
    switch (orientation_config_.mode)
    {
    case OrientationMode::FromTrajectory:
        // 使用轨迹中定义的姿态
        return base_orientation;
    case OrientationMode::Fixed:
        // 返回固定的姿态
        return orientation_config_.fixed_orientation;
    case OrientationMode::Spin:
    {
        // 旋转模式：根据角速度和已过时间计算当前姿态
        const double angle = orientation_config_.angular_velocity * elapsed_seconds;
        
        // 根据配置的旋转轴，在相应轴上叠加旋转角度
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
    // 如果发布器未初始化，直接返回
    if (!pose_pub_)
    {
        return;
    }
    
    // 构造并发布姿态消息
    mag_core_msgs::MagnetPose msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_config_.parent_frame;
    msg.position = pose.position;
    msg.orientation = pose.orientation;
    msg.magnetic_strength = motion_config_.magnetic_strength;
    msg.confidence = 1.0;  // 真实位姿，置信度为1.0（完全可信）
    pose_pub_.publish(msg);
}

void MagnetNode::publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose, bool static_tf)
{
    // 构造 TF 变换消息
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_config_.parent_frame;
    tf_msg.child_frame_id = frame_config_.child_frame;
    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;

    // 根据类型选择静态或动态 TF 广播器
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
