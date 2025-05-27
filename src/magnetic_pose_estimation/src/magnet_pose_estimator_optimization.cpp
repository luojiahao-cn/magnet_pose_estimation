#include <magnetic_pose_estimation/magnet_pose_estimator_optimization.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace magnetic_pose_estimation
{

    /**
     * @brief 构造函数，初始化磁铁位姿估计器
     * @param nh ROS节点句柄
     *
     * 加载传感器配置、参数，初始化发布器和订阅器
     */
    OptimizationMagnetPoseEstimator::OptimizationMagnetPoseEstimator(ros::NodeHandle &nh)
        : nh_(nh)
    {
        // 加载传感器配置
        if (!SensorConfig::getInstance().loadConfig(nh_))
        {
            ROS_ERROR("无法加载传感器配置");
            return;
        }
        loadParameters();

        // 初始化发布器和订阅器
        magnet_pose_pub_ = nh_.advertise<MagnetPose>("/magnet_pose/predicted", 10);
        magnetic_field_sub_ = nh_.subscribe("/magnetic_field/processed", 25,
                                            &OptimizationMagnetPoseEstimator::magneticFieldCallback, this);
        reset_localization_service_ = nh_.advertiseService("/magnet_pose/reset_localization",
                                                           &OptimizationMagnetPoseEstimator::resetServiceCallback, this);

        ROS_INFO("磁铁位置估计器（最优化）已初始化");
    }

    /**
     * @brief 加载参数，包括磁铁初始参数和优化相关参数
     */
    void OptimizationMagnetPoseEstimator::loadParameters()
    {
        // 加载磁铁初始参数
        std::vector<double> position_vec, direction_vec;
        nh_.param("estimator_config/magnet/position", position_vec, std::vector<double>{0.01, 0.01, 0.05});
        nh_.param("estimator_config/magnet/direction", direction_vec, std::vector<double>{0, 0, 1});
        nh_.param<double>("estimator_config/magnet/strength", initial_strength_, 2.0);
        nh_.param<double>("estimator_config/magnet/strength_delta", strength_delta_, 0.0);

        // 判断是否需要优化强度
        optimize_strength_ = (strength_delta_ != 0.0);
        strength_min_ = initial_strength_ - (optimize_strength_ ? strength_delta_ : 0.0);
        strength_max_ = initial_strength_ + (optimize_strength_ ? strength_delta_ : 0.0);

        // 设置初始参数
        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]).normalized();
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;

        // 加载优化参数
        nh_.param<double>("estimator_config/optimization/max_position_change", max_position_change_, 0.5);
        // nh_.param<double>("estimator_config/optimization/max_error_threshold", max_error_threshold_, 10.0);
        // nh_.param<double>("estimator_config/optimization/min_improvement", min_improvement_, 0.1);
        nh_.param<int>("estimator_config/optimization/max_iterations", max_iterations_, 20);
        nh_.param<double>("estimator_config/optimization/convergence_threshold", convergence_threshold_, 1e-6);
        nh_.param<double>("estimator_config/optimization/lambda_damping", lambda_damping_, 1e5);

        ROS_INFO("参数加载完成");
    }

    /**
     * @brief 磁场数据回调，收集所有传感器的磁场测量，收集齐后触发位姿估计
     * @param msg 磁场消息
     */
    void OptimizationMagnetPoseEstimator::magneticFieldCallback(const MagneticField::ConstPtr &msg)
    {
        // 收集每个传感器的磁场测量
        Eigen::Vector3d raw_field(msg->mag_x, msg->mag_y, msg->mag_z);
        MagneticField corrected_msg = *msg;
        corrected_msg.mag_x = raw_field.x();
        corrected_msg.mag_y = raw_field.y();
        corrected_msg.mag_z = raw_field.z();
        measurements_[msg->sensor_id] = corrected_msg;

        // 收集到全部传感器数据后，进行位姿估计
        if (measurements_.size() >= SensorConfig::getInstance().getSensorCount())
        {
            estimateMagnetPose();
            measurements_.clear();
        }
    }

    /**
     * @brief 重置服务回调，将估计参数重置为初始值
     * @return 服务调用是否成功
     */
    bool OptimizationMagnetPoseEstimator::resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        resetToInitialParameters();
        ROS_INFO("已将参数重置为初始值");
        return true;
    }

    /**
     * @brief 估计磁铁位姿（位置、方向、强度），核心优化算法
     */
    void OptimizationMagnetPoseEstimator::estimateMagnetPose()
    {
        // 记录优化开始时间
        auto t_start = std::chrono::steady_clock::now();

        // 1. 构建测量矩阵和传感器位置矩阵
        Eigen::MatrixXd sensor_positions, measured_fields;
        buildMeasurementMatrices(sensor_positions, measured_fields);

        // 2. 备份当前参数
        Eigen::Vector3d backup_position = current_position_;
        Eigen::Vector3d backup_direction = magnetic_direction_;

        // 3. 构建状态向量
        Eigen::VectorXd state, best_state;
        int state_dim = optimize_strength_ ? 7 : 6;
        initializeStateVector(state, best_state, state_dim);

        double initial_error = 0.0, last_error = std::numeric_limits<double>::max(), best_error = last_error;
        bool optimization_failed = false;

        // 4. Gauss-Newton/Levenberg-Marquardt迭代优化
        for (int iter = 0; iter < max_iterations_; ++iter)
        {
            double error = optimizeStep(sensor_positions, measured_fields, state, best_state, best_error, initial_error, iter, state_dim);
            if (std::abs(error - last_error) < convergence_threshold_)
                break;
            if (std::isnan(error) || std::isinf(error))
            {
                ROS_ERROR("优化过程中出现NaN或Inf，恢复到初始参数");
                optimization_failed = true;
                break;
            }
            last_error = error;
        }

        // 5. 判断优化是否成功
        if (!optimization_failed)
        {
            current_position_ = best_state.segment<3>(0);
            magnetic_direction_ = best_state.segment<3>(3).normalized();
            if (optimize_strength_)
                magnet_strength_ = best_state(6);
        }
        else
        {
            current_position_ = backup_position;
            magnetic_direction_ = backup_direction;
        }

        // 6. 发布估计结果
        publishMagnetPose(current_position_, magnetic_direction_, magnet_strength_);
    }

    /**
     * @brief 构建测量矩阵和传感器位置矩阵
     */
    void OptimizationMagnetPoseEstimator::buildMeasurementMatrices(Eigen::MatrixXd &sensor_positions, Eigen::MatrixXd &measured_fields)
    {
        int n = measurements_.size();
        sensor_positions.resize(n, 3);
        measured_fields.resize(n, 3);

        int i = 0;
        for (const auto &measurement : measurements_)
        {
            const auto &msg = measurement.second;
            sensor_positions.row(i) << msg.sensor_pose.position.x, msg.sensor_pose.position.y, msg.sensor_pose.position.z;
            measured_fields.row(i) << msg.mag_x, msg.mag_y, msg.mag_z;
            ++i;
        }
    }

    /**
     * @brief 初始化状态向量
     */
    void OptimizationMagnetPoseEstimator::initializeStateVector(Eigen::VectorXd &state, Eigen::VectorXd &best_state, int state_dim)
    {
        state.resize(state_dim);
        state.segment<3>(0) = current_position_;
        state.segment<3>(3) = magnetic_direction_;
        if (optimize_strength_)
            state(6) = magnet_strength_;
        best_state = state;
    }

    /**
     * @brief 优化单步，返回当前误差
     */
    double OptimizationMagnetPoseEstimator::optimizeStep(
        const Eigen::MatrixXd &sensor_positions,
        const Eigen::MatrixXd &measured_fields,
        Eigen::VectorXd &state,
        Eigen::VectorXd &best_state,
        double &best_error,
        double &initial_error,
        int iter,
        int state_dim)
    {
        // 1. 从状态向量提取参数
        Eigen::Vector3d position = state.segment<3>(0);
        Eigen::Vector3d direction = state.segment<3>(3).normalized();
        double strength = optimize_strength_ ? state(6) : magnet_strength_;

        // 2. 计算预测磁场
        Eigen::MatrixXd predicted_fields = MagneticFieldCalculator::calculateMagneticField(
            sensor_positions, position, direction, strength);

        // 3. 计算残差（测量-预测）
        Eigen::MatrixXd residuals = measured_fields - predicted_fields;
        Eigen::VectorXd residuals_vector = Eigen::Map<Eigen::VectorXd>(residuals.data(), residuals.size());

        double error = residuals_vector.norm() / std::sqrt(residuals_vector.size()); // 均方根误差

        if (iter == 0)
            initial_error = error;
        if (error < best_error)
        {
            best_error = error;
            best_state = state;
        }

        // 4. 计算雅可比矩阵
        Eigen::MatrixXd J = calculateJacobian(sensor_positions, position, direction, strength);

        // 5. Levenberg-Marquardt步长
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
        Eigen::VectorXd delta = (J.transpose() * J + lambda_damping_ * I).ldlt().solve(J.transpose() * residuals_vector);

        // 6. 步长限制
        if (delta.segment<3>(0).norm() > max_position_change_)
        {
            ROS_WARN("位置增量过大 (%.3f m)，限制优化步长", delta.segment<3>(0).norm());
            delta *= max_position_change_ / delta.segment<3>(0).norm();
        }

        // 7. 更新状态向量
        state += delta;
        state.segment<3>(3).normalize(); // 方向归一化
        if (optimize_strength_)
        {
            state(6) = std::max(strength_min_, std::min(state(6), strength_max_));
            if (state(6) < 0)
                state(6) = 0.01;
        }

        return error;
    }

    /**
     * @brief 将估计参数重置为初始值
     */
    void OptimizationMagnetPoseEstimator::resetToInitialParameters()
    {
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;
        ROS_INFO("参数已重置为初始值");
    }

    /**
     * @brief 发布磁铁位姿消息
     * @param position 估计位置
     * @param direction 估计方向
     * @param strength 估计强度
     */
    void OptimizationMagnetPoseEstimator::publishMagnetPose(const Eigen::Vector3d &position,
                                                            const Eigen::Vector3d &direction,
                                                            double strength)
    {
        MagnetPose pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";
        pose_msg.position.x = position.x();
        pose_msg.position.y = position.y();
        pose_msg.position.z = position.z();

        // 方向向量转四元数（z轴到direction的旋转）
        tf2::Vector3 z_axis(0, 0, 1);
        tf2::Vector3 direction_vec(direction.x(), direction.y(), direction.z());
        tf2::Vector3 rotation_axis = z_axis.cross(direction_vec);
        double rotation_angle = std::acos(z_axis.dot(direction_vec));
        if (rotation_axis.length() < 1e-6)
        {
            if (direction.z() > 0)
            {
                pose_msg.orientation.w = 1.0;
                pose_msg.orientation.x = 0.0;
                pose_msg.orientation.y = 0.0;
                pose_msg.orientation.z = 0.0;
            }
            else
            {
                pose_msg.orientation.w = 0.0;
                pose_msg.orientation.x = 1.0;
                pose_msg.orientation.y = 0.0;
                pose_msg.orientation.z = 0.0;
            }
        }
        else
        {
            rotation_axis.normalize();
            tf2::Quaternion q(rotation_axis, rotation_angle);
            q.normalize();
            pose_msg.orientation = tf2::toMsg(q);
        }
        pose_msg.magnetic_strength = strength;
        magnet_pose_pub_.publish(pose_msg);
    }

    /**
     * @brief 数值微分法计算雅可比矩阵
     * @param sensor_positions 传感器位置矩阵
     * @param position 当前磁铁位置
     * @param direction 当前磁铁方向
     * @param strength 当前磁铁强度
     * @return 雅可比矩阵
     */
    Eigen::MatrixXd OptimizationMagnetPoseEstimator::calculateJacobian(
        const Eigen::MatrixXd &sensor_positions,
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &direction,
        double strength)
    {
        // 数值微分法计算雅可比矩阵
        const double delta = 1e-6;
        const int num_sensors = sensor_positions.rows();
        int param_dim = optimize_strength_ ? 7 : 6;
        Eigen::MatrixXd J(num_sensors * 3, param_dim);

        // 对位置参数求偏导
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d pos_plus = position;
            pos_plus(i) += delta;
            Eigen::MatrixXd field_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, pos_plus, direction, strength);
            Eigen::MatrixXd field_center = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength);
            Eigen::MatrixXd difference = field_plus - field_center;
            J.col(i) = Eigen::Map<Eigen::VectorXd>(difference.data(), difference.size()) / delta;
        }

        // 对方向参数求偏导
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d dir_plus = direction;
            dir_plus(i) += delta;
            dir_plus.normalize();
            Eigen::MatrixXd field_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, dir_plus, strength);
            Eigen::MatrixXd field_center = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength);
            Eigen::MatrixXd difference = field_plus - field_center;
            J.col(i + 3) = Eigen::Map<Eigen::VectorXd>(difference.data(), difference.size()) / delta;
        }

        // 对强度参数求偏导
        if (optimize_strength_)
        {
            Eigen::MatrixXd field_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength + delta);
            Eigen::MatrixXd field_center = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength);
            Eigen::MatrixXd difference = field_plus - field_center;
            J.col(6) = Eigen::Map<Eigen::VectorXd>(difference.data(), difference.size()) / delta;
        }
        return J;
    }

} // namespace magnetic_pose_estimation