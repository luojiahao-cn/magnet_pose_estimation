#include <magnetic_pose_estimation/magnet_pose_estimator_kalman.hpp>
#include <ceres/ceres.h>
#include <ceres/jet.h>

namespace magnetic_pose_estimation
{
    KalmanMagnetPoseEstimator::KalmanMagnetPoseEstimator(ros::NodeHandle &nh)
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
                                            &KalmanMagnetPoseEstimator::magneticFieldCallback, this);
        reset_localization_service_ = nh_.advertiseService("/magnet_pose/reset_localization",
                                                           &KalmanMagnetPoseEstimator::resetServiceCallback, this);

        ROS_INFO("磁铁位置估计器（卡尔曼滤波）已初始化");
    }

    void KalmanMagnetPoseEstimator::loadParameters()
    {
        std::vector<double> position_vec, direction_vec;
        nh_.param("estimator_config/magnet/position", position_vec, std::vector<double>{0.01, 0.01, 0.05});
        nh_.param("estimator_config/magnet/direction", direction_vec, std::vector<double>{0, 0, 1});
        nh_.param<double>("estimator_config/magnet/strength", initial_strength_, 2.0);

        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]);
        initial_direction_.normalize();

        // 状态向量：[a, b, c, theta, phi] - 5维磁体位姿
        // a, b, c: 位置坐标
        // theta, phi: 磁极方向角（球坐标）
        state_dim_ = 5;
        state_ = Eigen::VectorXd::Zero(state_dim_);

        // 设置初始位置
        state_.segment<3>(0) = initial_position_;

        // 将笛卡尔坐标方向转换为球坐标角度
        double theta = std::acos(std::max(-1.0, std::min(1.0, initial_direction_(2))));  // 极角 [0, π]
        double phi = std::atan2(initial_direction_(1), initial_direction_(0));  // 方位角 [-π, π]
        state_(3) = theta;
        state_(4) = phi;

        // 从配置文件加载卡尔曼滤波器参数
        double position_process_noise, angle_process_noise, measurement_noise, initial_covariance;
        
        nh_.param<double>("estimator_config/kalman/position_process_noise", position_process_noise, 1e-7);
        // 角度过程噪声相应减小
        nh_.param<double>("estimator_config/kalman/angle_process_noise", angle_process_noise, 1e-8);
        
        nh_.param<double>("estimator_config/kalman/measurement_noise", measurement_noise, 1e-2);
        nh_.param<double>("estimator_config/kalman/initial_covariance", initial_covariance, 0.01);

        // 初始化协方差矩阵 P
        P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * initial_covariance;

        // 过程噪声矩阵 Q（随机游走模型）- 分别设置位置和角度噪声
        Q_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
        Q_.diagonal().segment<3>(0).setConstant(position_process_noise);  // 位置噪声（亚毫米级）
        Q_.diagonal().segment<2>(3).setConstant(angle_process_noise);     // 角度噪声

        // 测量噪声矩阵 R
        int sensor_count = SensorConfig::getInstance().getSensorCount();
        R_ = Eigen::MatrixXd::Identity(sensor_count * 3, sensor_count * 3) * measurement_noise;

        ROS_INFO("卡尔曼滤波器初始状态: 位置=[%.4f, %.4f, %.4f], 角度=[%.4f, %.4f], 强度=%.4f",
                 state_(0), state_(1), state_(2), state_(3), state_(4), initial_strength_);
        ROS_INFO("卡尔曼滤波参数已加载 - 传感器数量: %d, 状态维度: %d", 
                 sensor_count, state_dim_);
        ROS_INFO("过程噪声: 位置=%.2e (%.4fmm std), 角度=%.2e", 
                 position_process_noise, sqrt(position_process_noise) * 1000, angle_process_noise);
    }

    void KalmanMagnetPoseEstimator::magneticFieldCallback(const MagneticField::ConstPtr &msg)
    {
        measurements_[msg->sensor_id] = *msg;

        if (measurements_.size() >= SensorConfig::getInstance().getSensorCount())
        {
            // 构建测量向量
            int n = measurements_.size();
            Eigen::VectorXd measurement(n * 3);
            int i = 0;
            for (const auto &m : measurements_)
            {
                measurement.segment<3>(i * 3) = Eigen::Vector3d(m.second.mag_x, m.second.mag_y, m.second.mag_z);
                i++;
            }
            predict();
            update(measurement);
            publishMagnetPose();
            measurements_.clear();
        }
    }

    void KalmanMagnetPoseEstimator::predict()
    {
        // 随机游走模型：x_{k|k-1} = x_{k-1|k-1} + w_k
        // 状态预测：状态保持不变（随机游走）
        // state_ = state_; // 状态不变
        
        // 协方差预测：P = P + Q
        P_ = P_ + Q_;
        
        ROS_DEBUG("随机游走预测完成");
    }

    void KalmanMagnetPoseEstimator::update(const Eigen::VectorXd &measurement)
    {
        // 将球坐标角度转换为笛卡尔方向向量
        double theta = state_(3);
        double phi = state_(4);
        Eigen::Vector3d direction(std::sin(theta) * std::cos(phi),
                                 std::sin(theta) * std::sin(phi),
                                 std::cos(theta));

        // 构建传感器位置矩阵
        int n = SensorConfig::getInstance().getSensorCount();
        Eigen::MatrixXd sensor_positions(n, 3);
        int i = 0;
        for (const auto &m : measurements_)
        {
            sensor_positions.row(i) << m.second.sensor_pose.position.x,
                m.second.sensor_pose.position.y,
                m.second.sensor_pose.position.z;
            i++;
        }
        
        // 使用现有的磁场计算器得到预测值
        Eigen::MatrixXd predicted_fields = MagneticFieldCalculator::calculateMagneticField(
            sensor_positions,
            state_.segment<3>(0),  // 位置
            direction,             // 方向
            initial_strength_);    // 强度（固定值）
        Eigen::VectorXd predicted = Eigen::Map<Eigen::VectorXd>(predicted_fields.data(), predicted_fields.size());

        Eigen::VectorXd y = measurement - predicted;

        // 使用数值差分计算雅可比矩阵
        Eigen::MatrixXd H(n * 3, state_dim_);
        const double delta = 1e-6;
        
        ROS_DEBUG("开始计算雅可比矩阵，状态维度: %d, 测量维度: %d", state_dim_, n * 3);
        
        for (int j = 0; j < state_dim_; ++j)
        {
            // 计算状态的扰动
            Eigen::VectorXd state_plus = state_;
            state_plus(j) += delta;
            
            // 将扰动后的球坐标角度转换为笛卡尔方向向量
            double theta_plus = state_plus(3);
            double phi_plus = state_plus(4);
            Eigen::Vector3d direction_plus(std::sin(theta_plus) * std::cos(phi_plus),
                                          std::sin(theta_plus) * std::sin(phi_plus),
                                          std::cos(theta_plus));
            
            // 计算扰动后的预测磁场
            Eigen::MatrixXd pred_plus = MagneticFieldCalculator::calculateMagneticField(
                sensor_positions,
                state_plus.segment<3>(0),  // 位置
                direction_plus,            // 方向  
                initial_strength_);        // 强度
            
            // 计算雅可比矩阵的第j列 (∂h/∂x_j)
            Eigen::VectorXd pred_plus_vec = Eigen::Map<Eigen::VectorXd>(pred_plus.data(), pred_plus.size());
            H.col(j) = (pred_plus_vec - predicted) / delta;
        }
        
        ROS_DEBUG("数值差分雅可比矩阵计算完成");
        // 卡尔曼滤波更新
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        state_ = state_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;

        // 角度约束
        state_(3) = std::max(0.0, std::min(M_PI, state_(3)));      // theta ∈ [0, π]
        if (state_(4) > M_PI) state_(4) -= 2.0 * M_PI;             // phi ∈ [-π, π]
        if (state_(4) < -M_PI) state_(4) += 2.0 * M_PI;
        
        ROS_DEBUG("状态更新完成 - 位置: [%.4f, %.4f, %.4f], 角度: [%.4f, %.4f]",
                  state_(0), state_(1), state_(2), state_(3), state_(4));
    }

    void KalmanMagnetPoseEstimator::publishMagnetPose()
    {
        MagnetPose pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";
        pose_msg.position.x = state_(0);
        pose_msg.position.y = state_(1);
        pose_msg.position.z = state_(2);

        // 将球坐标角度转换为笛卡尔方向向量
        double theta = state_(3);
        double phi = state_(4);
        Eigen::Vector3d direction(std::sin(theta) * std::cos(phi),
                                 std::sin(theta) * std::sin(phi),
                                 std::cos(theta));

        // 方向转四元数
        tf2::Vector3 z_axis(0, 0, 1);
        tf2::Vector3 direction_vec(direction(0), direction(1), direction(2));
        tf2::Vector3 rotation_axis = z_axis.cross(direction_vec);
        double rotation_angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(direction_vec))));
        
        if (rotation_axis.length() < 1e-6)
        {
            if (direction_vec.z() > 0)
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
        
        pose_msg.magnetic_strength = initial_strength_;  // 使用固定强度


        magnet_pose_pub_.publish(pose_msg);
    }

    bool KalmanMagnetPoseEstimator::resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        // 尝试从仿真环境获取实际位姿
        bool use_simulation_pose = false;

        // 等待仿真位姿话题
        ROS_INFO("尝试从仿真环境获取实际磁铁位姿...");
        boost::shared_ptr<MagnetPose const> sim_pose_msg = 
            ros::topic::waitForMessage<MagnetPose>("/magnet_pose/simulation", nh_, ros::Duration(2.0));
        
        if (sim_pose_msg)
        {
            // 使用仿真的实际位姿初始化
            state_.segment<3>(0) = Eigen::Vector3d(sim_pose_msg->position.x, 
                                                    sim_pose_msg->position.y, 
                                                    sim_pose_msg->position.z);
            
            // 从四元数转换为方向向量，再转换为球坐标角度
            tf2::Quaternion q;
            tf2::fromMsg(sim_pose_msg->orientation, q);
            tf2::Matrix3x3 rotation_matrix(q);
            
            // 提取Z轴方向（磁极方向）
            tf2::Vector3 z_direction = rotation_matrix.getColumn(2);
            Eigen::Vector3d direction(z_direction.x(), z_direction.y(), z_direction.z());
            direction.normalize();
            
            // 转换为球坐标角度
            double theta = std::acos(std::max(-1.0, std::min(1.0, direction(2))));  // 极角 [0, π]
            double phi = std::atan2(direction(1), direction(0));  // 方位角 [-π, π]
            state_(3) = theta;
            state_(4) = phi;
            
            // 更新磁铁强度（如果仿真提供）
            if (sim_pose_msg->magnetic_strength > 0)
            {
                initial_strength_ = sim_pose_msg->magnetic_strength;
            }
            
            use_simulation_pose = true;
            ROS_INFO("使用仿真位姿初始化 - 位置: [%.4f, %.4f, %.4f], 方向: [%.3f, %.3f, %.3f], 强度: %.2f", 
                        state_(0), state_(1), state_(2), 
                        direction(0), direction(1), direction(2), initial_strength_);
        }
        else
        {
            ROS_WARN("无法获取仿真位姿，使用配置文件的初始值");
        }
        
        
        // 如果没有成功获取仿真位姿，使用配置文件的初始值
        if (!use_simulation_pose)
        {
            // 重置位置
            state_.segment<3>(0) = initial_position_;
            
            // 重置角度（从方向向量转换为球坐标）
            double theta = std::acos(std::max(-1.0, std::min(1.0, initial_direction_(2))));
            double phi = std::atan2(initial_direction_(1), initial_direction_(0));
            state_(3) = theta;
            state_(4) = phi;
            
            ROS_INFO("使用配置文件初始值 - 位置: [%.4f, %.4f, %.4f], 角度: [%.4f, %.4f]",
                     state_(0), state_(1), state_(2), state_(3), state_(4));
        }
        
        // 重置协方差矩阵
        double initial_covariance;
        nh_.param<double>("estimator_config/kalman/initial_covariance", initial_covariance, 0.01);
        P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * initial_covariance;
        
        // 清空测量缓存
        measurements_.clear();
        
        ROS_INFO("卡尔曼滤波器已重置 - 最终状态: 位置=[%.4f, %.4f, %.4f], 角度=[%.4f, %.4f]",
                 state_(0), state_(1), state_(2), state_(3), state_(4));
        return true;
    }

} // namespace magnetic_pose_estimation