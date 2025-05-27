#include <magnetic_pose_estimation/magnet_pose_estimator_optimization.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ceres/ceres.h>

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
        strength_min_ = std::max(0.0, initial_strength_ - (optimize_strength_ ? strength_delta_ : 0.0));
        strength_max_ = initial_strength_ + (optimize_strength_ ? strength_delta_ : 0.0);

        // 设置初始参数
        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]).normalized();
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;

        // 加载优化参数
        nh_.param<int>("estimator_config/optimization/max_iterations", max_iterations_, 50);
        nh_.param<double>("estimator_config/optimization/function_tolerance", function_tolerance_, 1e-8);
        nh_.param<double>("estimator_config/optimization/gradient_tolerance", gradient_tolerance_, 1e-10);
        nh_.param<double>("estimator_config/optimization/parameter_tolerance", parameter_tolerance_, 1e-8);
        nh_.param<int>("estimator_config/optimization/num_threads", num_threads_, 1);
        nh_.param<bool>("estimator_config/optimization/minimizer_progress_to_stdout", minimizer_progress_to_stdout_, false);

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
        // 参数数组，Ceres会直接优化这些数组的值
        double position[3] = {current_position_.x(), current_position_.y(), current_position_.z()};
        double direction[3] = {magnetic_direction_.x(), magnetic_direction_.y(), magnetic_direction_.z()};
        double strength = magnet_strength_; // 可选优化参数

        ceres::Problem problem;
        for (const auto &m : measurements_)
        {
            Eigen::Vector3d sensor_pos(m.second.sensor_pose.position.x, m.second.sensor_pose.position.y, m.second.sensor_pose.position.z);
            Eigen::Vector3d measured_field(m.second.mag_x, m.second.mag_y, m.second.mag_z);

            if (optimize_strength_) {
                // 优化位置、方向和强度
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<MagnetFieldResidualWithStrength, 3, 3, 3, 1>(
                        new MagnetFieldResidualWithStrength(sensor_pos, measured_field)),
                    nullptr,
                    position, direction, &strength);
            } else {
                // 只优化位置和方向，强度固定
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<MagnetFieldResidual, 3, 3, 3>(
                        new MagnetFieldResidual(sensor_pos, measured_field, magnet_strength_)),
                    nullptr,
                    position, direction);
            }
        }

        // 对方向参数添加归一化约束，使其始终为单位向量
        problem.SetParameterization(direction, new ceres::HomogeneousVectorParameterization(3));

        // 如果优化强度，可加边界约束
        if (optimize_strength_) {
            problem.SetParameterLowerBound(&strength, 0, strength_min_);
            problem.SetParameterUpperBound(&strength, 0, strength_max_);
        }

        // 配置Ceres求解器参数（使用新的配置）
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = max_iterations_;
        options.function_tolerance = function_tolerance_;
        options.gradient_tolerance = gradient_tolerance_;
        options.parameter_tolerance = parameter_tolerance_;
        options.num_threads = num_threads_;
        options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;
        ceres::Solver::Summary summary;

        // 执行优化
        ceres::Solve(options, &problem, &summary);

        // 优化完成后，更新估计的磁铁位置、方向和强度
        current_position_ = Eigen::Vector3d(position[0], position[1], position[2]);
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
        if (optimize_strength_) {
            magnet_strength_ = strength;
        }

        // 发布估计结果
        publishMagnetPose(current_position_, magnetic_direction_, magnet_strength_);
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

} // namespace magnetic_pose_estimation