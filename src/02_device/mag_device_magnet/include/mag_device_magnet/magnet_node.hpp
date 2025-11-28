/**
 * @file magnet_node.hpp
 * @brief 磁体设备节点头文件
 * 
 * 定义磁体设备 ROS 节点的核心类和配置结构
 */

#pragma once

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

/**
 * @brief TF 坐标系配置
 */
struct FrameConfig
{
    std::string parent_frame;  ///< 父坐标系名称
    std::string child_frame;   ///< 子坐标系名称
};

/**
 * @brief ROS 话题配置
 */
struct TopicConfig
{
    std::string pose_topic = "/magnetic/pose_true";  ///< 姿态真值发布话题
};

/**
 * @brief 运动参数配置
 */
struct MotionConfig
{
    double update_rate_hz = 60.0;      ///< 更新频率（Hz）
    double magnetic_strength = 1.0;    ///< 磁体强度
};

/**
 * @brief TF 发布配置
 */
struct TfConfig
{
    bool enable = true;         ///< 是否启用 TF 发布
    bool use_static_tf = false; ///< 是否使用静态 TF（仅适用于静态轨迹）
};

/**
 * @brief 轨迹类型枚举
 */
enum class TrajectoryType
{
    Static,      ///< 静态轨迹
    Circular,    ///< 圆形轨迹
    Rectangular  ///< 矩形轨迹
};

/**
 * @brief 静态轨迹配置
 */
struct StaticTrajectoryConfig
{
    geometry_msgs::Pose pose;  ///< 固定姿态
};

/**
 * @brief 圆形轨迹配置
 */
struct CircularTrajectoryConfig
{
    CircularTrajectoryConfig()
    {
        orientation.w = 1.0;
    }
    geometry_msgs::Point center;      ///< 圆心位置
    double radius = 0.05;              ///< 半径（米）
    double angular_speed = 0.5;       ///< 角速度（弧度/秒）
    double phase = 0.0;               ///< 初始相位（弧度）
    geometry_msgs::Quaternion orientation;  ///< 姿态四元数
};

/**
 * @brief 矩形轨迹配置
 */
struct RectangularTrajectoryConfig
{
    RectangularTrajectoryConfig()
    {
        orientation.w = 1.0;
    }
    double width = 0.02;              ///< 宽度（米）
    double height = 0.02;             ///< 高度（米）
    geometry_msgs::Point center;       ///< 中心位置（包含 x, y, z）
    double velocity = 0.01;            ///< 线速度（米/秒）
    geometry_msgs::Quaternion orientation;  ///< 默认姿态四元数（当未单独指定时使用）
    double perimeter = 0.0;            ///< 周长（自动计算）
};

/**
 * @brief 轨迹配置
 */
struct TrajectoryConfig
{
    TrajectoryType type = TrajectoryType::Static;  ///< 轨迹类型
    StaticTrajectoryConfig static_config;          ///< 静态轨迹配置
    CircularTrajectoryConfig circular_config;       ///< 圆形轨迹配置
    RectangularTrajectoryConfig rectangular_config; ///< 矩形轨迹配置
};

/**
 * @brief 轨迹采样输出
 */
struct TrajectorySample
{
    geometry_msgs::Pose pose;  ///< 采样得到的姿态
};

/**
 * @brief 姿态模式枚举
 */
enum class OrientationMode
{
    FromTrajectory,  ///< 使用轨迹中的姿态
    Fixed,           ///< 固定姿态
    Spin             ///< 旋转姿态
};

/**
 * @brief 姿态配置
 */
struct OrientationConfig
{
    OrientationConfig()
    {
        fixed_orientation.w = 1.0;
    }
    OrientationMode mode = OrientationMode::FromTrajectory;  ///< 姿态模式
    geometry_msgs::Quaternion fixed_orientation;             ///< 固定姿态四元数
    double initial_roll = 0.0;      ///< 初始滚转角（弧度）
    double initial_pitch = 0.0;      ///< 初始俯仰角（弧度）
    double initial_yaw = 0.0;       ///< 初始偏航角（弧度）
    double angular_velocity = 0.0;  ///< 角速度（弧度/秒）
    bool axis_x = false;            ///< 是否绕 X 轴旋转
    bool axis_y = false;            ///< 是否绕 Y 轴旋转
    bool axis_z = false;            ///< 是否绕 Z 轴旋转
};

/**
 * @brief 磁体设备 ROS 节点
 * 
 * 负责生成磁体的轨迹姿态并发布到 ROS 话题和 TF 树
 */
class MagnetNode
{
public:
    /**
     * @brief 构造函数
     * 
     * @param nh ROS 节点句柄
     * @param pnh 私有节点句柄
     * @param frame_config 坐标系配置
     * @param motion_config 运动参数配置
     * @param topic_config 话题配置
     * @param tf_config TF 发布配置
     * @param trajectory_config 轨迹配置
     * @param orientation_config 姿态配置
     */
    MagnetNode(ros::NodeHandle nh,
               ros::NodeHandle pnh,
               FrameConfig frame_config,
               MotionConfig motion_config,
               TopicConfig topic_config,
               TfConfig tf_config,
               TrajectoryConfig trajectory_config,
               OrientationConfig orientation_config);

    /**
     * @brief 启动节点
     * 
     * 开始定时更新并发布姿态数据
     */
    void start();

private:
    /**
     * @brief 设置发布器
     */
    void setupPublisher();
    
    /**
     * @brief 定时更新回调
     * 
     * @param event 定时器事件
     */
    void onUpdate(const ros::TimerEvent &event);

    /**
     * @brief 采样轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 轨迹采样结果
     */
    TrajectorySample sampleTrajectory(double elapsed_seconds) const;
    
    /**
     * @brief 采样静态轨迹
     * 
     * @return TrajectorySample 静态轨迹采样结果
     */
    TrajectorySample sampleStatic() const;
    
    /**
     * @brief 采样圆形轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 圆形轨迹采样结果
     */
    TrajectorySample sampleCircular(double elapsed_seconds) const;
    
    /**
     * @brief 采样矩形轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 矩形轨迹采样结果
     */
    TrajectorySample sampleRectangular(double elapsed_seconds) const;
    
    /**
     * @brief 采样姿态
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @param base_orientation 基础姿态（来自轨迹）
     * @return geometry_msgs::Quaternion 采样得到的姿态四元数
     */
    geometry_msgs::Quaternion sampleOrientation(double elapsed_seconds,
                                                const geometry_msgs::Quaternion &base_orientation) const;

    /**
     * @brief 发布姿态消息
     * 
     * @param stamp 时间戳
     * @param pose 姿态
     */
    void publishPose(const ros::Time &stamp, const geometry_msgs::Pose &pose);
    
    /**
     * @brief 发布 TF 变换
     * 
     * @param stamp 时间戳
     * @param pose 姿态
     * @param static_tf 是否为静态 TF
     */
    void publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose, bool static_tf);

    ros::NodeHandle nh_;              ///< ROS 节点句柄
    ros::NodeHandle pnh_;             ///< 私有节点句柄

    FrameConfig frame_config_;        ///< 坐标系配置
    MotionConfig motion_config_;      ///< 运动参数配置
    TopicConfig topic_config_;        ///< 话题配置
    TfConfig tf_config_;              ///< TF 发布配置
    TrajectoryConfig trajectory_config_;      ///< 轨迹配置
    OrientationConfig orientation_config_;    ///< 姿态配置

    ros::Publisher pose_pub_;         ///< 姿态发布器
    ros::Timer update_timer_;         ///< 更新定时器
    ros::Time start_time_;            ///< 启动时间

    tf2_ros::TransformBroadcaster dynamic_broadcaster_;   ///< 动态 TF 广播器
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;  ///< 静态 TF 广播器
    bool static_sent_ = false;        ///< 静态 TF 是否已发送
};

} // namespace mag_device_magnet
