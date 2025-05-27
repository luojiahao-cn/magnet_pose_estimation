#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <map>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/magnet_pose_estimator_base.hpp>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <chrono>

namespace magnetic_pose_estimation
{

    class OptimizationMagnetPoseEstimator : public BaseMagnetPoseEstimator
    {
    public:
        explicit OptimizationMagnetPoseEstimator(ros::NodeHandle &nh);

        void magneticFieldCallback(const MagneticField::ConstPtr &msg) override;
        bool resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) override;

    private:
        void loadParameters();
        void estimateMagnetPose();

        // 数值微分法计算雅可比矩阵
        Eigen::MatrixXd calculateJacobian(const Eigen::MatrixXd &sensor_positions,
                                          const Eigen::Vector3d &position,
                                          const Eigen::Vector3d &direction,
                                          double strength);

        // 发布磁铁位姿消息
        void publishMagnetPose(const Eigen::Vector3d &position,
                               const Eigen::Vector3d &direction,
                               double strength);

        // 重置参数为初始值
        void resetToInitialParameters();

        // 构建测量矩阵和传感器位置矩阵
        void buildMeasurementMatrices(Eigen::MatrixXd &sensor_positions, Eigen::MatrixXd &measured_fields);

        // 初始化状态向量
        void initializeStateVector(Eigen::VectorXd &state, Eigen::VectorXd &best_state, int state_dim);

        // 优化单步
        double optimizeStep(const Eigen::MatrixXd &sensor_positions,
                            const Eigen::MatrixXd &measured_fields,
                            Eigen::VectorXd &state,
                            Eigen::VectorXd &best_state,
                            double &best_error,
                            double &initial_error,
                            int iter,
                            int state_dim);

        ros::NodeHandle nh_;
        ros::Publisher magnet_pose_pub_;
        ros::Subscriber magnetic_field_sub_;
        ros::ServiceServer reset_localization_service_;

        std::map<int, MagneticField> measurements_;

        Eigen::Vector3d initial_position_;
        Eigen::Vector3d initial_direction_;
        double initial_strength_;
        double strength_delta_;
        bool optimize_strength_;
        double strength_min_, strength_max_;

        Eigen::Vector3d current_position_;
        Eigen::Vector3d magnetic_direction_;
        double magnet_strength_;

        double max_position_change_;
        double max_error_threshold_;
        double min_improvement_;
        int max_iterations_;
        double convergence_threshold_;
        double lambda_damping_;
    };

} // namespace magnetic_pose_estimation