#include "mag_pose_estimator/ekf_estimator.h"

#include <Eigen/Geometry>
#include <ros/ros.h>

namespace mag_pose_estimator {

namespace {
/**
 * @brief 归一化四元数
 */
Eigen::Quaterniond normalize(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond copy = q;
  copy.normalize();
  return copy;
}

/**
 * @brief 归一化向量
 * 如果向量长度过小，返回单位 X 向量作为默认值
 */
Eigen::Vector3d normalizeVec(const Eigen::Vector3d &v) {
  double norm = v.norm();
  if (norm < 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  return v / norm;
}
}  // 匿名命名空间

/**
 * @brief 构造函数
 * 初始化状态向量和协方差矩阵
 * 状态向量：[位置(0-2), 四元数(x,y,z,w -> 3-6), 偏置(7-9)]
 */
EKFEstimator::EKFEstimator() {
  state_.setZero();
  state_(6) = 1.0;  // 四元数 w 分量初始化为 1（单位四元数）
  covariance_.setIdentity();
}

/**
 * @brief 初始化估计器
 * 设置初始协方差矩阵（对角元素为 0.1）
 */
void EKFEstimator::initialize() {
  covariance_.setIdentity();
  covariance_ *= 0.1;
  last_update_time_ = ros::Time(0);
  initialized_ = true;
}

/**
 * @brief 更新估计器状态
 * @param mag 磁场测量数据（单位：mT，毫特斯拉）
 * 
 * 处理流程：
 * 1. 根据时间间隔更新过程噪声（随机游走模型）
 * 2. 调用 correct() 进行观测更新
 */
void EKFEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }

  ros::Time stamp = mag.header.stamp;
  // 根据时间间隔累积过程噪声（随机游走模型）
  if (!stamp.isZero() && !last_update_time_.isZero()) {
    double dt = (stamp - last_update_time_).toSec();
    if (dt > 0.0) {
      double q_pos = config_.process_noise_position;
      double q_ori = config_.process_noise_orientation;
      // 位置过程噪声
      for (int i = 0; i < 3; ++i) {
        covariance_(i, i) += q_pos * dt;
      }
      // 姿态过程噪声
      for (int i = 0; i < 4; ++i) {
        covariance_(3 + i, 3 + i) += q_ori * dt;
      }
    }
  }
  last_update_time_ = stamp;

  Eigen::Vector3d measurement(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  correct(measurement);
}

/**
 * @brief 预测步骤
 * 当前实现为随机游走模型（无运动模型）
 * TODO: 可扩展融合 IMU 线加速度以改进位置预测
 */
void EKFEstimator::predict() {
  // 预留运动模型接口
  // TODO: 在此融合 IMU 线加速度以改进位置预测
}

/**
 * @brief 观测更新步骤
 * @param measurement 磁场测量向量 (mT)
 * 
 * EKF 更新流程：
 * 1. 从状态向量提取四元数并归一化
 * 2. 计算预测磁场方向：predicted = q * world_field
 * 3. 计算残差：residual = normalize(measurement) - normalize(predicted)
 * 4. 计算观测雅可比矩阵 H
 * 5. 计算卡尔曼增益 K
 * 6. 更新状态：state = state + K * residual
 * 7. 更新协方差：P = (I - K*H) * P
 * 8. 重新归一化四元数
 * 9. 更新偏置：bias += position_gain * residual
 */
void EKFEstimator::correct(const Eigen::Vector3d &measurement) {
  // 提取并归一化四元数（状态向量中顺序为 [x, y, z, w]）
  Eigen::Quaterniond q(state_(6), state_(3), state_(4), state_(5));
  q = normalize(q);
  state_(3) = q.x();
  state_(4) = q.y();
  state_(5) = q.z();
  state_(6) = q.w();

  // 计算预测磁场方向（通过旋转世界坐标系磁场向量）
  Eigen::Vector3d predicted = q * config_.world_field;
  
  // 计算归一化残差（只使用方向，不关心幅值）
  Eigen::Vector3d residual = normalizeVec(measurement) - normalizeVec(predicted);

  // 计算观测雅可比矩阵 H (3×10)
  Eigen::Matrix<double, 3, 10> H;
  H.setZero();

  // 姿态雅可比：表示四元数扰动对归一化磁场方向的影响
  Eigen::Matrix<double, 3, 4> dq_dq = orientationJacobian(q, config_.world_field);
  H.block<3, 4>(0, 3) = dq_dq;

  // 观测噪声协方差矩阵
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.measurement_noise;
  
  // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^(-1)
  Eigen::Matrix<double, 10, 3> K = covariance_ * H.transpose() * (H * covariance_ * H.transpose() + R).inverse();

  // 更新状态
  state_ += K * residual;
  
  // 更新协方差：P = (I - K*H) * P
  covariance_ = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * covariance_;

  // 重新归一化四元数
  Eigen::Quaterniond q_update(state_(6), state_(3), state_(4), state_(5));
  q_update.normalize();
  state_(3) = q_update.x();
  state_(4) = q_update.y();
  state_(5) = q_update.z();
  state_(6) = q_update.w();

  // 更新偏置（将残差的一部分注入偏置，允许建模磁场漂移）
  Eigen::Vector3d bias(state_(7), state_(8), state_(9));
  bias += config_.position_gain * residual;
  state_(7) = bias.x();
  state_(8) = bias.y();
  state_(9) = bias.z();
}

/**
 * @brief 计算姿态对观测的雅可比矩阵
 * @param q 当前姿态四元数
 * @param field_world 世界坐标系下的磁场向量 (mT)
 * @return 3×4 雅可比矩阵
 * 
 * 雅可比矩阵表示四元数扰动对归一化磁场方向的影响：
 *   J = [∂(normalize(R*q*field_world))/∂q]
 * 
 * 其中 R 是四元数 q 对应的旋转矩阵。
 */
Eigen::Matrix<double, 3, 4> EKFEstimator::orientationJacobian(const Eigen::Quaterniond &q,
                                                             const Eigen::Vector3d &field_world) const {
  Eigen::Matrix<double, 3, 4> J;
  J.setZero();
  
  // 四元数对应的旋转矩阵
  Eigen::Matrix3d R = q.toRotationMatrix();
  
  // 磁场向量的反对称矩阵（用于计算叉积）
  Eigen::Matrix<double, 3, 3> skew;
  skew << 0.0, -field_world.z(), field_world.y(),
      field_world.z(), 0.0, -field_world.x(),
      -field_world.y(), field_world.x(), 0.0;

  // 雅可比矩阵的前三列：对四元数虚部 [x, y, z] 的偏导
  J.block<3, 3>(0, 0) = -R * skew;
  
  // 雅可比矩阵的第四列：对四元数实部 w 的偏导
  J.col(3) = R * field_world;
  
  return J;
}

/**
 * @brief 获取当前估计的姿态
 * @return 包含位置和姿态的 Pose 消息
 */
geometry_msgs::Pose EKFEstimator::getPose() const {
  geometry_msgs::Pose pose;
  pose.position.x = state_(0);
  pose.position.y = state_(1);
  pose.position.z = state_(2);
  pose.orientation.x = state_(3);
  pose.orientation.y = state_(4);
  pose.orientation.z = state_(5);
  pose.orientation.w = state_(6);
  return pose;
}

}  // 命名空间 mag_pose_estimator
