#ifndef MAGNET_POSE_ESTIMATOR_HPP
#define MAGNET_POSE_ESTIMATOR_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>
#include <std_srvs/Empty.h>

namespace magnetic_pose_estimation {

/**
 * @brief 磁铁位置估计器类
 * 
 * 该类通过测量多个传感器的磁场数据，估计磁铁的位置、方向和强度。
 * 使用Gauss-Newton优化算法，最小化测量磁场与预测磁场之间的误差。
 */
class MagnetPoseEstimator {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    MagnetPoseEstimator(ros::NodeHandle& nh);

    /**
     * @brief 析构函数
     */
    ~MagnetPoseEstimator() = default;

private:
    /**
     * @brief 从参数服务器加载配置参数
     */
    void loadParameters();

    /**
     * @brief 磁场数据回调函数
     * @param msg 磁场测量消息
     */
    void magneticFieldCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg);

    /**
     * @brief 估计磁铁位置和参数
     * 使用Gauss-Newton优化算法最小化测量与预测之间的误差
     */
    void estimateMagnetPose();

    /**
     * @brief 计算雅可比矩阵
     * @param sensor_positions 传感器位置矩阵
     * @param position 磁铁位置
     * @param direction 磁铁方向
     * @param strength 磁铁强度
     * @return 雅可比矩阵
     */
    Eigen::MatrixXd calculateJacobian(
        const Eigen::MatrixXd& sensor_positions,
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction,
        double strength);

    /**
     * @brief 发布磁铁位姿消息
     * @param position 磁铁位置
     * @param direction 磁铁方向
     * @param strength 磁铁强度
     */
    void publishMagnetPose(
        const Eigen::Vector3d& position,
        const Eigen::Vector3d& direction,
        double strength);

    /**
     * @brief 重置参数为初始值
     */
    void resetToInitialParameters();

    /**
     * @brief 重置服务回调函数，将参数重置为初始值
     */
    bool resetServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    // ROS通信相关
    ros::NodeHandle& nh_;
    ros::Publisher magnet_pose_pub_;
    ros::Subscriber magnetic_field_sub_;
    ros::ServiceServer reset_localization_service_;

    // 测量数据存储
    std::map<int, magnetic_pose_estimation::MagneticField> measurements_;
    
    // 当前参数
    Eigen::Vector3d current_position_;
    Eigen::Vector3d magnetic_direction_;
    double magnet_strength_;
    
    // 初始参数（作为备份）
    Eigen::Vector3d initial_position_;
    Eigen::Vector3d initial_direction_;
    double initial_strength_;
    
    // 优化控制参数
    double max_position_change_;
    double max_error_threshold_;
    double min_improvement_;
    int max_iterations_;
    double convergence_threshold_;
    double lambda_damping_;

    bool optimize_strength_ = false;
    double strength_delta_;
    double strength_min_;
    double strength_max_;
};

} // namespace magnetic_pose_estimation

#endif // MAGNET_POSE_ESTIMATOR_HPP