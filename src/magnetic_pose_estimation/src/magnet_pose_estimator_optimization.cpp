#include <magnetic_pose_estimation/magnet_pose_estimator_optimization.hpp>

namespace magnetic_pose_estimation
{

    OptimizationMagnetPoseEstimator::OptimizationMagnetPoseEstimator(ros::NodeHandle &nh)
        : nh_(ros::NodeHandle())
    {
        if (!SensorConfig::getInstance().loadConfig(nh_))
        {
            ROS_ERROR("无法加载传感器配置");
            return;
        }
        loadParameters();

        magnet_pose_pub_ = nh_.advertise<MagnetPose>("/magnet_pose/predicted", 10);
        magnetic_field_sub_ = nh_.subscribe("/magnetic_field/processed", 25,
                                            &OptimizationMagnetPoseEstimator::magneticFieldCallback, this);
        reset_localization_service_ = nh_.advertiseService("/magnet_pose/reset_localization",
                                                           &OptimizationMagnetPoseEstimator::resetServiceCallback, this);

        ROS_INFO("磁铁位置估计器（最优化）已初始化");
    }

    void OptimizationMagnetPoseEstimator::loadParameters()
    {
        std::vector<double> position_vec;
        std::vector<double> direction_vec;

        // 加载磁铁初始参数
        nh_.param("estimator_config/magnet/position", position_vec, std::vector<double>{0.01, 0.01, 0.05});
        nh_.param("estimator_config/magnet/direction", direction_vec, std::vector<double>{0, 0, 1});
        nh_.param<double>("estimator_config/magnet/strength", initial_strength_, 2.0);
        nh_.param<double>("estimator_config/magnet/strength_delta", strength_delta_, 0.0);

        // 判断是否需要优化strength
        if (strength_delta_ == 0.0)
        {
            optimize_strength_ = false;
            strength_min_ = initial_strength_;
            strength_max_ = initial_strength_;
        }
        else
        {
            optimize_strength_ = true;
            strength_min_ = initial_strength_ - strength_delta_;
            strength_max_ = initial_strength_ + strength_delta_;
        }

        // 设置初始参数
        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]);
        initial_direction_.normalize();
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;

        // 加载优化参数
        nh_.param<double>("estimator_config/optimization/max_position_change", max_position_change_, 0.5);
        nh_.param<double>("estimator_config/optimization/max_error_threshold", max_error_threshold_, 10.0);
        nh_.param<double>("estimator_config/optimization/min_improvement", min_improvement_, 0.1);
        nh_.param<int>("estimator_config/optimization/max_iterations", max_iterations_, 20);
        nh_.param<double>("estimator_config/optimization/convergence_threshold", convergence_threshold_, 1e-6);
        nh_.param<double>("estimator_config/optimization/lambda_damping", lambda_damping_, 1e5);

        ROS_INFO("预测参数已加载 - 位置: [%.3f, %.3f, %.3f], 方向: [%.3f, %.3f, %.3f], 强度: %.3f, 允许波动: %.3f, 优化: %s, 范围: [%.3f, %.3f]",
                 initial_position_[0], initial_position_[1], initial_position_[2],
                 initial_direction_[0], initial_direction_[1], initial_direction_[2],
                 initial_strength_, strength_delta_,
                 optimize_strength_ ? "是" : "否",
                 strength_min_, strength_max_);
        ROS_INFO("优化参数已加载 - 最大位置变化: %.3f, 最大误差阈值: %.3f, 最小改善: %.1f%%, 最大迭代次数: %d, 收敛阈值: %.1e, 阻尼系数: %.1e",
                 max_position_change_, max_error_threshold_, min_improvement_ * 100, max_iterations_, convergence_threshold_, lambda_damping_);
        ROS_INFO("参数加载完成");
    }

    void OptimizationMagnetPoseEstimator::magneticFieldCallback(const MagneticField::ConstPtr &msg)
    {
        Eigen::Vector3d raw_field(msg->mag_x, msg->mag_y, msg->mag_z);

        MagneticField corrected_msg = *msg;
        corrected_msg.mag_x = raw_field.x();
        corrected_msg.mag_y = raw_field.y();
        corrected_msg.mag_z = raw_field.z();

        // 收集测量数据
        measurements_[msg->sensor_id] = corrected_msg;

        // 当收集到所有传感器的数据后，进行磁铁位姿估计
        if (measurements_.size() >= SensorConfig::getInstance().getSensorCount())
        {
            estimateMagnetPose();
            measurements_.clear(); // 清空测量数据，准备下一轮估计
        }
    }

    bool OptimizationMagnetPoseEstimator::resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        resetToInitialParameters();
        ROS_INFO("已将参数重置为初始值");
        return true;
    }

    void OptimizationMagnetPoseEstimator::estimateMagnetPose()
    {
        // 记录优化开始时间
        auto t_start = std::chrono::steady_clock::now();

        static int total_count = 0;
        static auto first_time = t_start;

        total_count++;
        auto duration = std::chrono::duration<double>(t_start - first_time).count();
        double avg_freq = (duration > 0.0) ? (total_count / duration) : 0.0;

        // 构建测量矩阵和传感器位置矩阵
        int n = measurements_.size();
        Eigen::MatrixXd sensor_positions(n, 3);
        Eigen::MatrixXd measured_fields(n, 3);

        int i = 0;
        for (const auto &measurement : measurements_)
        {
            const auto &msg = measurement.second;
            sensor_positions(i, 0) = msg.sensor_pose.position.x;
            sensor_positions(i, 1) = msg.sensor_pose.position.y;
            sensor_positions(i, 2) = msg.sensor_pose.position.z;

            measured_fields(i, 0) = msg.mag_x;
            measured_fields(i, 1) = msg.mag_y;
            measured_fields(i, 2) = msg.mag_z;
            i++;
        }

        // 备份当前参数，以便在优化失败时恢复
        Eigen::Vector3d backup_position = current_position_;
        Eigen::Vector3d backup_direction = magnetic_direction_;

        // 构建状态向量
        int state_dim = optimize_strength_ ? 7 : 6;
        Eigen::VectorXd state(state_dim);
        state.segment<3>(0) = current_position_;
        state.segment<3>(3) = magnetic_direction_;
        if (optimize_strength_)
            state(6) = magnet_strength_;

        // Gauss-Newton迭代
        double initial_error = 0.0;
        double last_error = std::numeric_limits<double>::max();
        double best_error = last_error;
        Eigen::VectorXd best_state = state;
        bool optimization_failed = false;

        for (int iter = 0; iter < max_iterations_; ++iter)
        {
            // 从状态向量中提取参数
            Eigen::Vector3d position = state.segment<3>(0);
            Eigen::Vector3d direction = state.segment<3>(3);
            direction.normalize();
            double strength = optimize_strength_ ? state(6) : magnet_strength_;

            // 计算预测的磁场
            Eigen::MatrixXd predicted_fields = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength);

            // 计算残差
            Eigen::MatrixXd residuals = measured_fields - predicted_fields;
            Eigen::VectorXd residuals_vector = Eigen::Map<Eigen::VectorXd>(residuals.data(), residuals.size());

            double error = residuals_vector.norm() / std::sqrt(residuals_vector.size()); // 均方根误差

            // 记录初始误差
            if (iter == 0)
            {
                initial_error = error;
            }

            // 记录最佳状态
            if (error < best_error)
            {
                best_error = error;
                best_state = state;
            }

            // 计算Jacobian矩阵
            Eigen::MatrixXd J = calculateJacobian(sensor_positions, position, direction, strength);

            // 使用带阻尼的最小二乘法计算增量
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
            Eigen::VectorXd delta = (J.transpose() * J + lambda_damping_ * I).ldlt().solve(J.transpose() * residuals_vector);

            // 检查增量是否合理
            if (delta.segment<3>(0).norm() > max_position_change_)
            {
                // 位置变化过大，说明优化可能不稳定
                ROS_WARN("位置增量过大 (%.3f m)，限制优化步长", delta.segment<3>(0).norm());
                delta *= max_position_change_ / delta.segment<3>(0).norm();
            }

            // 更新状态向量
            state += delta;

            // 强制归一化方向向量
            state.segment<3>(3).normalize();

            if (optimize_strength_)
                state(6) = std::max(strength_min_, std::min(state(6), strength_max_));

            if (optimize_strength_ && state(6) < 0)
                state(6) = 0.01; // 强度不能为负

            // 检查收敛
            if (std::abs(error - last_error) < convergence_threshold_)
            {
                break;
            }

            // 检查异常情况
            if (std::isnan(error) || std::isinf(error))
            {
                ROS_ERROR("优化过程中出现NaN或Inf，恢复到初始参数");
                optimization_failed = true;
                break;
            }

            last_error = error;
        }

        // 判断优化是否成功
        if (!optimization_failed)
        {
            // 更新当前参数为最佳状态
            current_position_ = best_state.segment<3>(0);
            magnetic_direction_ = best_state.segment<3>(3).normalized();
            if (optimize_strength_)
                magnet_strength_ = best_state(6);

            // 检查优化结果是否显著改善了误差
            double error_improvement = (initial_error - best_error) / initial_error;

            // 检查结果是否合理
            bool position_reasonable = true;
            for (int i = 0; i < 3; ++i)
            {
                if (std::abs(current_position_(i)) > 1.0)
                { // 有效范围假设为±1米
                    position_reasonable = false;
                    break;
                }
            }

            // 如果优化结果不合理或误差没有显著改善，恢复到之前的参数
            // if (!position_reasonable || best_error > max_error_threshold_ || error_improvement < min_improvement_) {
            //     ROS_WARN_THROTTLE(0.1, "优化结果不满足要求（误差:%.3f，改善:%.1f%%），恢复到之前的参数",
            //                  best_error, error_improvement * 100);
            //     current_position_ = backup_position;
            //     magnetic_direction_ = backup_direction;
            // } else {
            //     ROS_INFO_THROTTLE(2.0, "优化成功，误差从%.3f减小到%.3f（改善:%.1f%%）",
            //                  initial_error, best_error, error_improvement * 100);
            // }
        }
        else
        {
            // 优化失败，恢复到之前的参数
            ROS_WARN_THROTTLE(0.1, "优化失败，恢复到之前的参数");
            current_position_ = backup_position;
            magnetic_direction_ = backup_direction;
        }

        ROS_INFO_THROTTLE(1.0,
                          "磁铁位姿优化平均频率: %5.2f Hz | 估计磁铁参数 - 位置: [%6.3f, %6.3f, %6.3f], 方向: [%6.3f, %6.3f, %6.3f], 强度: %6.3f",
                          avg_freq,
                          current_position_[0], current_position_[1], current_position_[2],
                          magnetic_direction_[0], magnetic_direction_[1], magnetic_direction_[2],
                          magnet_strength_);

        publishMagnetPose(current_position_, magnetic_direction_, magnet_strength_);

        // 记录优化结束时间并输出耗时
        // auto t_end = std::chrono::steady_clock::now();
        // double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        // ROS_INFO("本轮优化耗时: %.2f ms", elapsed_ms);
    }

    void OptimizationMagnetPoseEstimator::resetToInitialParameters()
    {
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;
        ROS_INFO("参数已重置为初始值");
    }

    void OptimizationMagnetPoseEstimator::publishMagnetPose(const Eigen::Vector3d &position,
                                                            const Eigen::Vector3d &direction,
                                                            double strength)
    {
        MagnetPose pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";

        // 设置位置
        pose_msg.position.x = position.x();
        pose_msg.position.y = position.y();
        pose_msg.position.z = position.z();

        // 设置方向 - 从方向向量计算四元数
        tf2::Vector3 z_axis(0, 0, 1); // 默认z轴
        tf2::Vector3 direction_vec(direction.x(), direction.y(), direction.z());

        // 计算旋转轴和角度
        tf2::Vector3 rotation_axis = z_axis.cross(direction_vec);
        double rotation_angle = std::acos(z_axis.dot(direction_vec));

        // 如果方向向量与z轴平行，则不需要旋转
        if (rotation_axis.length() < 1e-6)
        {
            if (direction.z() > 0)
            {
                // 与z轴同向
                pose_msg.orientation.w = 1.0;
                pose_msg.orientation.x = 0.0;
                pose_msg.orientation.y = 0.0;
                pose_msg.orientation.z = 0.0;
            }
            else
            {
                // 与z轴反向
                pose_msg.orientation.w = 0.0;
                pose_msg.orientation.x = 1.0;
                pose_msg.orientation.y = 0.0;
                pose_msg.orientation.z = 0.0;
            }
        }
        else
        {
            // 一般情况，从轴角表示计算四元数
            rotation_axis.normalize();
            tf2::Quaternion q(rotation_axis, rotation_angle);
            q.normalize();
            pose_msg.orientation = tf2::toMsg(q);
        }

        // 设置磁铁强度
        pose_msg.magnetic_strength = strength;

        magnet_pose_pub_.publish(pose_msg);
    }

    Eigen::MatrixXd OptimizationMagnetPoseEstimator::calculateJacobian(
        const Eigen::MatrixXd &sensor_positions,
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &direction,
        double strength)
    {
        // 雅可比矩阵计算
        const double delta = 1e-6;
        const int num_sensors = sensor_positions.rows();
        int param_dim = optimize_strength_ ? 7 : 6;
        Eigen::MatrixXd J(num_sensors * 3, param_dim);

        // 计算位置的雅可比矩阵（前3列）
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

        // 计算方向的雅可比矩阵（后3列）
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d dir_plus = direction;
            dir_plus(i) += delta;
            dir_plus.normalize(); // 保持为单位向量

            Eigen::MatrixXd field_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, dir_plus, strength);

            Eigen::MatrixXd field_center = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, direction, strength);

            Eigen::MatrixXd difference = field_plus - field_center;
            J.col(i + 3) = Eigen::Map<Eigen::VectorXd>(difference.data(), difference.size()) / delta;
        }

        // 如果需要优化strength，计算第7列
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