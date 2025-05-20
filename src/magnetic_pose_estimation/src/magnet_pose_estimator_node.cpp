#include <ros/ros.h>
#include <string>
#include "magnetic_pose_estimation/magnet_pose_estimator_optimization.hpp"
#include "magnetic_pose_estimation/magnet_pose_estimator_kalman.hpp"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnet_pose_estimator");
    ros::NodeHandle nh("~");

    std::string estimator_type;
    nh.param<std::string>("estimator_type", estimator_type, "kalman");

    std::unique_ptr<magnetic_pose_estimation::BaseMagnetPoseEstimator> estimator;

    if (estimator_type == "optimization") {
        ROS_INFO("使用优化算法进行磁场姿态估计");
        estimator.reset(new magnetic_pose_estimation::OptimizationMagnetPoseEstimator(nh));
    } else if (estimator_type == "nn") {
        ROS_INFO("使用神经网络进行磁场姿态估计");
        // estimator.reset(new magnetic_pose_estimation::NNMagnetPoseEstimator(nh));
    } else if (estimator_type == "kalman") {
        ROS_INFO("使用卡尔曼滤波器进行磁场姿态估计");
        estimator.reset(new magnetic_pose_estimation::KalmanMagnetPoseEstimator(nh));
    } else {
        ROS_ERROR("未知的estimator_type: %s", estimator_type.c_str());
        return 1;
    }

    ros::spin();
    return 0;
}