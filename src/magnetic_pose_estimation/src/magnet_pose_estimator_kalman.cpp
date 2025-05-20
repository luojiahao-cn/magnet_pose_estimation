#include <magnetic_pose_estimation/magnet_pose_estimator_kalman.hpp>

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

    // 状态向量：[x, y, z, dx, dy, dz, strength]
    state_dim_ = 7;
    state_ = Eigen::VectorXd::Zero(state_dim_);
    state_.segment<3>(0) = initial_position_;
    state_.segment<3>(3) = initial_direction_;
    state_(6) = initial_strength_;

    P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01;
    Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1e-4;
    R_ = Eigen::MatrixXd::Identity(SensorConfig::getInstance().getSensorCount() * 3,
                                   SensorConfig::getInstance().getSensorCount() * 3) * 1e-2;

    ROS_INFO("卡尔曼滤波参数已加载");
}

void KalmanMagnetPoseEstimator::magneticFieldCallback(const MagneticField::ConstPtr &msg)
{
    measurements_[msg->sensor_id] = *msg;

    if (measurements_.size() >= SensorConfig::getInstance().getSensorCount()) {
        // 构建测量向量
        int n = measurements_.size();
        Eigen::VectorXd measurement(n * 3);
        int i = 0;
        for (const auto &m : measurements_) {
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
    // 简单预测：假设状态不变
    // 可根据需要添加运动模型
    // state_ = F * state_;
    // P_ = F * P_ * F.transpose() + Q_;
    P_ = P_ + Q_;
}

void KalmanMagnetPoseEstimator::update(const Eigen::VectorXd& measurement)
{
    // 观测模型：由当前state_通过磁场模型预测
    int n = SensorConfig::getInstance().getSensorCount();
    Eigen::MatrixXd sensor_positions(n, 3);
    int i = 0;
    for (const auto &m : measurements_) {
        sensor_positions.row(i) << m.second.sensor_pose.position.x,
                                   m.second.sensor_pose.position.y,
                                   m.second.sensor_pose.position.z;
        i++;
    }
    Eigen::MatrixXd predicted_fields = MagneticFieldCalculator::calculateMagneticField(
        sensor_positions,
        state_.segment<3>(0),
        state_.segment<3>(3),
        state_(6)
    );
    Eigen::VectorXd predicted = Eigen::Map<Eigen::VectorXd>(predicted_fields.data(), predicted_fields.size());

    Eigen::VectorXd y = measurement - predicted; 

    // 观测矩阵H用数值雅可比近似
    Eigen::MatrixXd H(n * 3, state_dim_);
    const double delta = 1e-6;
    for (int j = 0; j < state_dim_; ++j) {
        Eigen::VectorXd state_plus = state_;
        state_plus(j) += delta;
        Eigen::MatrixXd pred_plus = MagneticFieldCalculator::calculateMagneticField(
            sensor_positions,
            state_plus.segment<3>(0),
            state_plus.segment<3>(3),
            state_plus(6)
        );
        H.col(j) = (Eigen::Map<Eigen::VectorXd>(pred_plus.data(), pred_plus.size()) - predicted) / delta;
    }

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    state_ = state_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;

    // 方向归一化
    state_.segment<3>(3).normalize();
}

void KalmanMagnetPoseEstimator::publishMagnetPose()
{
    MagnetPose pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.position.x = state_(0);
    pose_msg.position.y = state_(1);
    pose_msg.position.z = state_(2);

    // 方向转四元数
    tf2::Vector3 z_axis(0, 0, 1);
    tf2::Vector3 direction_vec(state_(3), state_(4), state_(5));
    tf2::Vector3 rotation_axis = z_axis.cross(direction_vec);
    double rotation_angle = std::acos(z_axis.dot(direction_vec));
    if (rotation_axis.length() < 1e-6) {
        if (direction_vec.z() > 0) {
            pose_msg.orientation.w = 1.0;
            pose_msg.orientation.x = 0.0;
            pose_msg.orientation.y = 0.0;
            pose_msg.orientation.z = 0.0;
        } else {
            pose_msg.orientation.w = 0.0;
            pose_msg.orientation.x = 1.0;
            pose_msg.orientation.y = 0.0;
            pose_msg.orientation.z = 0.0;
        }
    } else {
        rotation_axis.normalize();
        tf2::Quaternion q(rotation_axis, rotation_angle);
        q.normalize();
        pose_msg.orientation = tf2::toMsg(q);
    }
    pose_msg.magnetic_strength = state_(6);

    magnet_pose_pub_.publish(pose_msg);
}

bool KalmanMagnetPoseEstimator::resetServiceCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
    state_.segment<3>(0) = initial_position_;
    state_.segment<3>(3) = initial_direction_;
    state_(6) = initial_strength_;
    P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01;
    ROS_INFO("卡尔曼滤波器已重置到初始状态");
    return true;
}

} // namespace magnetic_pose_estimation