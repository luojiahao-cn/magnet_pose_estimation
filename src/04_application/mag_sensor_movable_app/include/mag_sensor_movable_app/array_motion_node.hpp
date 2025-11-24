/**
 * @file array_motion_node.hpp
 * @brief 传感器阵列运动控制节点头文件
 * 
 * 定义传感器阵列运动控制 ROS 节点的核心类和配置结构
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace mag_sensor_movable_app
{

/**
 * @brief TF 坐标系配置
 */
struct FrameConfig
{
    std::string parent_frame;  ///< 父坐标系名称
    std::string child_frame;  ///< 子坐标系名称
};

/**
 * @brief 运动参数配置
 */
struct MotionConfig
{
    double update_rate_hz = 50.0;  ///< 更新频率（Hz）
};

/**
 * @brief 轨迹类型枚举
 */
enum class TrajectoryType
{
    Static,      ///< 静态轨迹
    Circular,    ///< 圆形轨迹
    Rectangular, ///< 矩形轨迹
    Linear,      ///< 直线轨迹
    Spin         ///< 自旋轨迹（位置固定，姿态旋转）
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
        fixed_orientation.w = 1.0;
    }
    geometry_msgs::Point center;      ///< 圆心位置
    double radius = 0.02;             ///< 半径（米）
    double angular_velocity = 0.5;    ///< 角速度（弧度/秒）
    double initial_angle = 0.0;       ///< 初始角度（弧度）
    std::string orientation_mode = "fixed";  ///< 姿态模式：fixed, normal_to_path
    geometry_msgs::Quaternion fixed_orientation;  ///< 固定姿态四元数
};

/**
 * @brief 矩形轨迹配置
 */
struct RectangularTrajectoryConfig
{
    RectangularTrajectoryConfig()
    {
        fixed_orientation.w = 1.0;
    }
    geometry_msgs::Point center;       ///< 中心位置
    double width = 0.04;               ///< 宽度（米）
    double height = 0.04;              ///< 高度（米）
    double velocity = 0.02;             ///< 线速度（米/秒）
    std::string orientation_mode = "fixed";  ///< 姿态模式：fixed
    geometry_msgs::Quaternion fixed_orientation;  ///< 固定姿态四元数
    double perimeter = 0.0;            ///< 周长（自动计算）
};

/**
 * @brief 直线轨迹配置
 */
struct LinearTrajectoryConfig
{
    LinearTrajectoryConfig()
    {
        fixed_orientation.w = 1.0;
    }
    geometry_msgs::Point start;        ///< 起点位置
    geometry_msgs::Point end;          ///< 终点位置
    double velocity = 0.02;             ///< 线速度（米/秒）
    std::string orientation_mode = "fixed";  ///< 姿态模式：fixed
    geometry_msgs::Quaternion fixed_orientation;  ///< 固定姿态四元数
    bool repeat = true;                ///< 是否往返运动
    double distance = 0.0;              ///< 距离（自动计算）
};

/**
 * @brief 自旋轨迹配置
 */
struct SpinTrajectoryConfig
{
    SpinTrajectoryConfig()
    {
        initial_orientation.w = 1.0;
    }
    geometry_msgs::Point position;     ///< 固定位置
    double initial_roll = 0.0;         ///< 初始滚转角（弧度）
    double initial_pitch = 0.0;        ///< 初始俯仰角（弧度）
    double initial_yaw = 0.0;          ///< 初始偏航角（弧度）
    geometry_msgs::Quaternion initial_orientation;  ///< 初始姿态四元数
    double angular_velocity = 1.0;     ///< 角速度（弧度/秒）
    bool axis_x = false;                ///< 是否绕 X 轴旋转
    bool axis_y = false;                ///< 是否绕 Y 轴旋转
    bool axis_z = false;                ///< 是否绕 Z 轴旋转
};

/**
 * @brief 姿态模式枚举
 */
enum class OrientationMode
{
    FromTrajectory,  ///< 使用轨迹中的姿态
    Fixed,           ///< 固定姿态
    Spin             ///< 旋转姿态（在运动的同时自旋）
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
 * @brief 轨迹配置
 */
struct TrajectoryConfig
{
    TrajectoryType type = TrajectoryType::Static;  ///< 轨迹类型
    StaticTrajectoryConfig static_config;          ///< 静态轨迹配置
    CircularTrajectoryConfig circular_config;     ///< 圆形轨迹配置
    RectangularTrajectoryConfig rectangular_config;  ///< 矩形轨迹配置
    LinearTrajectoryConfig linear_config;         ///< 直线轨迹配置
    SpinTrajectoryConfig spin_config;             ///< 自旋轨迹配置
    OrientationConfig orientation_config;         ///< 姿态配置（用于在运动时叠加自旋）
};

/**
 * @brief 轨迹采样输出
 */
struct TrajectorySample
{
    geometry_msgs::Pose pose;  ///< 采样得到的姿态
};

/**
 * @brief 传感器阵列运动控制节点
 * 
 * 负责根据配置的轨迹类型生成传感器阵列的运动轨迹，并发布动态 TF 变换
 */
class ArrayMotionNode
{
public:
    /**
     * @brief 构造函数
     * 
     * @param nh ROS 节点句柄
     * @param pnh 私有节点句柄
     * @param frame_config 坐标系配置
     * @param motion_config 运动参数配置
     * @param trajectory_config 轨迹配置
     */
    ArrayMotionNode(ros::NodeHandle nh,
                    ros::NodeHandle pnh,
                    FrameConfig frame_config,
                    MotionConfig motion_config,
                    TrajectoryConfig trajectory_config,
                    OrientationConfig orientation_config);

    /**
     * @brief 启动节点
     * 
     * 开始定时器循环，定期更新并发布 TF 变换
     */
    void start();

private:
    /**
     * @brief 定时器回调函数
     * 
     * @param event 定时器事件
     */
    void onUpdate(const ros::TimerEvent &event);

    /**
     * @brief 采样轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 采样结果
     */
    TrajectorySample sampleTrajectory(double elapsed_seconds) const;

    /**
     * @brief 采样静态轨迹
     * 
     * @return TrajectorySample 静态姿态
     */
    TrajectorySample sampleStatic() const;

    /**
     * @brief 采样圆形轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 采样结果
     */
    TrajectorySample sampleCircular(double elapsed_seconds) const;

    /**
     * @brief 采样矩形轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 采样结果
     */
    TrajectorySample sampleRectangular(double elapsed_seconds) const;

    /**
     * @brief 采样直线轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 采样结果
     */
    TrajectorySample sampleLinear(double elapsed_seconds) const;

    /**
     * @brief 采样自旋轨迹
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @return TrajectorySample 采样结果
     */
    TrajectorySample sampleSpin(double elapsed_seconds) const;

    /**
     * @brief 采样姿态（应用姿态配置）
     * 
     * @param elapsed_seconds 已过时间（秒）
     * @param base_orientation 基础姿态（来自轨迹）
     * @return geometry_msgs::Quaternion 采样得到的姿态四元数
     */
    geometry_msgs::Quaternion sampleOrientation(double elapsed_seconds,
                                                const geometry_msgs::Quaternion &base_orientation) const;

    /**
     * @brief 发布 TF 变换
     * 
     * @param stamp 时间戳
     * @param pose 姿态
     */
    void publishTransform(const ros::Time &stamp, const geometry_msgs::Pose &pose);

    ros::NodeHandle nh_;              ///< ROS 节点句柄
    ros::NodeHandle pnh_;             ///< 私有节点句柄

    FrameConfig frame_config_;        ///< 坐标系配置
    MotionConfig motion_config_;      ///< 运动参数配置
    TrajectoryConfig trajectory_config_;  ///< 轨迹配置
    OrientationConfig orientation_config_;  ///< 姿态配置

    ros::Timer update_timer_;         ///< 更新定时器
    ros::Time start_time_;            ///< 启动时间

    tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< TF 广播器
};

} // namespace mag_sensor_movable_app

