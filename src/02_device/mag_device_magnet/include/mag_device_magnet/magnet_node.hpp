#pragma once

#include <mag_core_utils/param_reader.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <mag_core_msgs/MagnetPose.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/timer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace mag_device_magnet
{

// TF 帧配置
struct FrameConfig
{
    std::string parent_frame;
    std::string child_frame;
};

// 真值姿态话题配置
struct TopicConfig
{
    std::string pose_topic = "/magnetic/pose_true";
};

// 运行频率与磁体强度
struct MotionConfig
{
    double update_rate_hz = 60.0;
    double magnetic_strength = 1.0;
};

// TF 发布开关
struct TfConfig
{
    bool enable = true;
    bool use_static_tf = false;
};

// 支持的轨迹模式
enum class TrajectoryType
{
    Static,
    Circular
};

// 静止轨迹配置
struct StaticTrajectoryConfig
{
    geometry_msgs::Pose pose;
};

// 圆轨迹配置
struct CircularTrajectoryConfig
{
    geometry_msgs::Point center;
    double radius = 0.05;
    double angular_speed = 0.5;
    double phase = 0.0;
    geometry_msgs::Quaternion orientation;
};

// 综合轨迹配置
struct TrajectoryConfig
{
    TrajectoryType type = TrajectoryType::Static;
    StaticTrajectoryConfig static_config;
    CircularTrajectoryConfig circular_config;
};

// 轨迹采样输出
struct TrajectorySample
{
    geometry_msgs::Pose pose;
};

FrameConfig loadFrameConfig(const mag_core_utils::param::StructReader &root);
TopicConfig loadTopicConfig(const mag_core_utils::param::StructReader &root, TopicConfig defaults = {});
MotionConfig loadMotionConfig(const mag_core_utils::param::StructReader &root, MotionConfig defaults = {});
TfConfig loadTfConfig(const mag_core_utils::param::StructReader &root, TfConfig defaults = {});
TrajectoryConfig loadTrajectoryConfig(const mag_core_utils::param::StructReader &root);

class MagnetNode
{
public:
    MagnetNode(ros::NodeHandle nh,
               ros::NodeHandle pnh,
               FrameConfig frame_config,
               MotionConfig motion_config,
               TopicConfig topic_config,
               TfConfig tf_config,
               TrajectoryConfig trajectory_config);

    void start();

private:
    void setupPublisher();
    void onUpdate(const ros::TimerEvent &event);

    TrajectorySample sampleTrajectory(double elapsed_seconds) const;
    TrajectorySample sampleStatic() const;
    TrajectorySample sampleCircular(double elapsed_seconds) const;

    void publishPose(const ros::Time &stamp, const geometry_msgs::Pose &pose);
    void publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose, bool static_tf);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    FrameConfig frame_config_;
    MotionConfig motion_config_;
    TopicConfig topic_config_;
    TfConfig tf_config_;
    TrajectoryConfig trajectory_config_;

    ros::Publisher pose_pub_;
    ros::Timer update_timer_;
    ros::Time start_time_;

    tf2_ros::TransformBroadcaster dynamic_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    bool static_sent_ = false;
};

} // namespace mag_device_magnet
