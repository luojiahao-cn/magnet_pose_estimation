/**
 * @file array_motion_node.cpp
 * @brief 传感器阵列运动控制节点实现
 * 
 * 实现传感器阵列运动控制 ROS 节点的核心功能，包括轨迹生成和 TF 发布
 */

#include <mag_sensor_movable_app/array_motion_node.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mag_sensor_movable_app
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

ArrayMotionNode::ArrayMotionNode(ros::NodeHandle nh,
                                 ros::NodeHandle pnh,
                                 FrameConfig frame_config,
                                 MotionConfig motion_config,
                                 TrajectoryConfig trajectory_config,
                                 OrientationConfig orientation_config)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      frame_config_(std::move(frame_config)),
      motion_config_(motion_config),
      trajectory_config_(std::move(trajectory_config)),
      orientation_config_(orientation_config)
{
}

void ArrayMotionNode::start()
{
    // 记录开始时间，用于驱动时间相关的轨迹
    start_time_ = ros::Time::now();

    // 立即发布一次数据，避免初始空白期
    onUpdate(ros::TimerEvent());

    // 按配置的频率创建定时器进行周期性更新
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / motion_config_.update_rate_hz),
                                    &ArrayMotionNode::onUpdate,
                                    this);
}

void ArrayMotionNode::onUpdate(const ros::TimerEvent &)
{
    const ros::Time stamp = ros::Time::now();
    const double elapsed = std::max(0.0, (stamp - start_time_).toSec());

    // 采样轨迹得到位置和基础姿态
    TrajectorySample sample = sampleTrajectory(elapsed);

    // 应用姿态配置（如果配置了自旋，则在运动的同时叠加自旋）
    sample.pose.orientation = sampleOrientation(elapsed, sample.pose.orientation);

    // 发布 TF 变换
    publishTransform(stamp, sample.pose);
}

TrajectorySample ArrayMotionNode::sampleTrajectory(double elapsed_seconds) const
{
    switch (trajectory_config_.type)
    {
    case TrajectoryType::Static:
        return sampleStatic();
    case TrajectoryType::Circular:
        return sampleCircular(elapsed_seconds);
    case TrajectoryType::Rectangular:
        return sampleRectangular(elapsed_seconds);
    case TrajectoryType::Linear:
        return sampleLinear(elapsed_seconds);
    case TrajectoryType::Spin:
        return sampleSpin(elapsed_seconds);
    default:
        throw std::runtime_error("未知的轨迹类型");
    }
}

TrajectorySample ArrayMotionNode::sampleStatic() const
{
    // 静态轨迹直接返回配置中的固定姿态
    TrajectorySample sample;
    sample.pose = trajectory_config_.static_config.pose;
    return sample;
}

TrajectorySample ArrayMotionNode::sampleCircular(double elapsed_seconds) const
{
    // 圆形轨迹：在二维平面上做圆周运动
    const auto &cfg = trajectory_config_.circular_config;

    // 计算当前角度（初始角度 + 角速度 * 时间）
    const double angle = cfg.initial_angle + cfg.angular_velocity * elapsed_seconds;

    TrajectorySample sample;
    // 根据角度计算圆周上的位置
    sample.pose.position.x = cfg.center.x + cfg.radius * std::cos(angle);
    sample.pose.position.y = cfg.center.y + cfg.radius * std::sin(angle);
    sample.pose.position.z = cfg.center.z;

    // 根据姿态模式设置姿态
    if (cfg.orientation_mode == "normal_to_path")
    {
        // 姿态垂直于路径方向（指向圆心）
        const double yaw = angle + M_PI;  // 指向圆心
        sample.pose.orientation = quaternionFromRpy(0.0, 0.0, yaw);
    }
    else
    {
        // 固定姿态
        sample.pose.orientation = cfg.fixed_orientation;
    }

    return sample;
}

TrajectorySample ArrayMotionNode::sampleRectangular(double elapsed_seconds) const
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
    sample.pose.orientation = cfg.fixed_orientation;
    return sample;
}

TrajectorySample ArrayMotionNode::sampleLinear(double elapsed_seconds) const
{
    const auto &cfg = trajectory_config_.linear_config;

    // 计算已走过的距离
    double distance = cfg.velocity * elapsed_seconds;

    TrajectorySample sample;

    if (cfg.repeat)
    {
        // 往返运动
        const double total_distance = 2.0 * cfg.distance;  // 往返总距离
        const double progress = std::fmod(distance, total_distance);

        if (progress <= cfg.distance)
        {
            // 正向运动：从起点到终点
            const double ratio = progress / cfg.distance;
            sample.pose.position.x = cfg.start.x + ratio * (cfg.end.x - cfg.start.x);
            sample.pose.position.y = cfg.start.y + ratio * (cfg.end.y - cfg.start.y);
            sample.pose.position.z = cfg.start.z + ratio * (cfg.end.z - cfg.start.z);
        }
        else
        {
            // 反向运动：从终点到起点
            const double ratio = (progress - cfg.distance) / cfg.distance;
            sample.pose.position.x = cfg.end.x - ratio * (cfg.end.x - cfg.start.x);
            sample.pose.position.y = cfg.end.y - ratio * (cfg.end.y - cfg.start.y);
            sample.pose.position.z = cfg.end.z - ratio * (cfg.end.z - cfg.start.z);
        }
    }
    else
    {
        // 单次运动：到达终点后停止
        const double ratio = std::min(1.0, distance / cfg.distance);
        sample.pose.position.x = cfg.start.x + ratio * (cfg.end.x - cfg.start.x);
        sample.pose.position.y = cfg.start.y + ratio * (cfg.end.y - cfg.start.y);
        sample.pose.position.z = cfg.start.z + ratio * (cfg.end.z - cfg.start.z);
    }

    sample.pose.orientation = cfg.fixed_orientation;
    return sample;
}

TrajectorySample ArrayMotionNode::sampleSpin(double elapsed_seconds) const
{
    const auto &cfg = trajectory_config_.spin_config;

    TrajectorySample sample;
    // 位置固定
    sample.pose.position = cfg.position;

    // 根据角速度和已过时间计算当前姿态
    const double angle = cfg.angular_velocity * elapsed_seconds;
    
    // 根据配置的旋转轴，在相应轴上叠加旋转角度
    double roll = cfg.initial_roll + (cfg.axis_x ? angle : 0.0);
    double pitch = cfg.initial_pitch + (cfg.axis_y ? angle : 0.0);
    double yaw = cfg.initial_yaw + (cfg.axis_z ? angle : 0.0);
    
    sample.pose.orientation = quaternionFromRpy(roll, pitch, yaw);
    return sample;
}

geometry_msgs::Quaternion ArrayMotionNode::sampleOrientation(double elapsed_seconds,
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
        
        // 确定初始姿态：如果配置了 initial_rpy，则使用配置值；否则从基础姿态中提取
        double initial_roll, initial_pitch, initial_yaw;
        if (orientation_config_.initial_roll != 0.0 || 
            orientation_config_.initial_pitch != 0.0 || 
            orientation_config_.initial_yaw != 0.0)
        {
            // 使用配置的初始 RPY
            initial_roll = orientation_config_.initial_roll;
            initial_pitch = orientation_config_.initial_pitch;
            initial_yaw = orientation_config_.initial_yaw;
        }
        else
        {
            // 从基础姿态中提取 RPY
            tf2::Quaternion base_q;
            tf2::fromMsg(base_orientation, base_q);
            tf2::Matrix3x3(base_q).getRPY(initial_roll, initial_pitch, initial_yaw);
        }
        
        // 根据配置的旋转轴，在初始姿态的相应轴上叠加旋转角度
        double roll = initial_roll + (orientation_config_.axis_x ? angle : 0.0);
        double pitch = initial_pitch + (orientation_config_.axis_y ? angle : 0.0);
        double yaw = initial_yaw + (orientation_config_.axis_z ? angle : 0.0);
        
        return quaternionFromRpy(roll, pitch, yaw);
    }
    default:
        return base_orientation;
    }
}

void ArrayMotionNode::publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose)
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

    // 发布动态 TF
    tf_broadcaster_.sendTransform(tf_msg);
}

} // namespace mag_sensor_movable_app

