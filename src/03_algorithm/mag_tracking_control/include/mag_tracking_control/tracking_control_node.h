#pragma once

#include "mag_tracking_control/tracking_control_strategy_base.h"
#include "mag_tracking_control/strategy_fixed_offset.h"
#include "mag_tracking_control/strategy_adaptive_distance.h"

#include <mag_core_msgs/MagnetPose.h>
#include <mag_device_arm/SetEndEffectorPose.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace mag_tracking_control {

/**
 * @brief 跟踪控制节点配置
 */
struct TrackingControlConfig {
    std::string strategy_type;              ///< 策略类型：fixed_offset, adaptive_distance
    std::string sensor_arm_name;            ///< 传感器机械臂名称
    std::string magnet_pose_topic;          ///< 磁铁位姿估计话题
    std::string target_pose_topic;          ///< 目标位姿发布话题
    std::string arm_service_name;           ///< 机械臂服务名称
    std::string sensor_frame;               ///< 传感器阵列坐标系名称
    std::string magnet_frame;               ///< 磁铁坐标系名称
    double update_rate;                     ///< 控制循环频率 (Hz)
    bool enable_execution;                  ///< 是否实际执行运动
    double velocity_scaling;                ///< 速度缩放因子
    double acceleration_scaling;            ///< 加速度缩放因子
    
    // 固定偏移策略参数
    Eigen::Vector3d fixed_offset;
    double max_movement_per_step;
    
    // 自适应距离策略参数
    double target_field_strength;
    double min_field_strength;
    double max_field_strength;
    double adjustment_gain;
};

/**
 * @brief 跟踪控制 ROS 节点
 * 
 * 负责：
 * 1. 订阅磁铁位姿估计结果
 * 2. 获取当前传感器阵列位姿（通过TF）
 * 3. 使用策略算法计算目标位姿
 * 4. 调用机械臂服务执行运动
 */
class TrackingControlNode {
public:
    TrackingControlNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    
    /**
     * @brief 运行控制循环
     */
    void run();

private:
    /**
     * @brief 从参数服务器加载配置
     */
    void loadParameters();
    
    /**
     * @brief 创建策略实例
     */
    std::unique_ptr<TrackingControlStrategyBase> createStrategy(const std::string &type);
    
    /**
     * @brief 磁铁位姿回调
     */
    void magnetPoseCallback(const mag_core_msgs::MagnetPose::ConstPtr &msg);
    
    /**
     * @brief 控制循环主函数
     */
    void controlLoop(const ros::TimerEvent &event);
    
    /**
     * @brief 获取当前传感器位姿（通过TF）
     */
    bool getCurrentSensorPose(geometry_msgs::Pose &pose);
    
    /**
     * @brief 执行传感器位姿运动
     */
    bool executePose(const geometry_msgs::Pose &target_pose);

    ros::NodeHandle nh_;                    ///< 全局节点句柄
    ros::NodeHandle pnh_;                   ///< 私有节点句柄
    
    TrackingControlConfig config_;            ///< 节点配置
    std::unique_ptr<TrackingControlStrategyBase> strategy_;  ///< 控制策略
    
    ros::Subscriber magnet_pose_sub_;       ///< 磁铁位姿订阅者
    ros::Publisher target_pose_pub_;        ///< 目标位姿发布者（用于可视化）
    ros::Timer control_timer_;              ///< 控制循环定时器
    
    ros::ServiceClient arm_service_client_; ///< 机械臂服务客户端
    
    tf2_ros::Buffer tf_buffer_;             ///< TF缓冲
    tf2_ros::TransformListener tf_listener_; ///< TF监听器
    
    geometry_msgs::Pose current_magnet_pose_; ///< 当前磁铁位姿
    ros::Time last_magnet_pose_time_;        ///< 上次收到磁铁位姿的时间
    bool has_magnet_pose_;                   ///< 是否已收到磁铁位姿
    
    std::vector<Eigen::Vector3d> last_field_strengths_;  ///< 上次的磁场强度数据
};

}  // namespace mag_tracking_control

