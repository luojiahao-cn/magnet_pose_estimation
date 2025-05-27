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

namespace magnetic_pose_estimation {

struct MagnetFieldResidual {
    MagnetFieldResidual(const Eigen::Vector3d &sensor_pos, const Eigen::Vector3d &measured_field, double strength)
        : sensor_pos_(sensor_pos), measured_field_(measured_field), strength_(strength) {}

    template <typename T>
    bool operator()(const T *const position, const T *const direction, T *residuals) const
    {
        Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
        Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
        dir.normalize();
        Eigen::Matrix<T, 3, 1> pred_field = MagneticFieldCalculator::calculateMagneticFieldT<T>(sensor_pos_.cast<T>(), pos, dir, T(strength_));
        for (int i = 0; i < 3; ++i)
            residuals[i] = pred_field(i) - T(measured_field_(i));
        return true;
    }

    const Eigen::Vector3d sensor_pos_;
    const Eigen::Vector3d measured_field_;
    const double strength_;
};

struct MagnetFieldResidualWithStrength {
    MagnetFieldResidualWithStrength(const Eigen::Vector3d &sensor_pos, const Eigen::Vector3d &measured_field)
        : sensor_pos_(sensor_pos), measured_field_(measured_field) {}

    template <typename T>
    bool operator()(const T *const position, const T *const direction, const T *const strength, T *residuals) const
    {
        Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
        Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
        dir.normalize();
        Eigen::Matrix<T, 3, 1> pred_field = MagneticFieldCalculator::calculateMagneticFieldT<T>(
            sensor_pos_.cast<T>(), pos, dir, *strength);
        for (int i = 0; i < 3; ++i)
            residuals[i] = pred_field(i) - T(measured_field_(i));
        return true;
    }

    const Eigen::Vector3d sensor_pos_;
    const Eigen::Vector3d measured_field_;
};

}

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

        // 发布磁铁位姿消息
        void publishMagnetPose(const Eigen::Vector3d &position,
                               const Eigen::Vector3d &direction,
                               double strength);

        // 重置参数为初始值
        void resetToInitialParameters();

        // 构建测量矩阵和传感器位置矩阵
        void buildMeasurementMatrices(Eigen::MatrixXd &sensor_positions, Eigen::MatrixXd &measured_fields);

        // 新的Ceres优化配置参数
        int max_iterations_;
        double function_tolerance_;
        double gradient_tolerance_;
        double parameter_tolerance_;
        int num_threads_;
        bool minimizer_progress_to_stdout_;

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
    };

} // namespace magnetic_pose_estimation