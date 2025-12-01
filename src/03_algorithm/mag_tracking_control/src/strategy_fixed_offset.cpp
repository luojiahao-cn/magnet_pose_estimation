#include "mag_tracking_control/strategy_fixed_offset.h"

#include <cmath>
#include <algorithm>

namespace mag_tracking_control {

// 固定偏移策略构造函数
FixedOffsetStrategy::FixedOffsetStrategy()
    : offset_(0.0, 0.0, 0.05)
    , max_movement_per_step_(0.02)
{
}

// 初始化固定偏移策略
bool FixedOffsetStrategy::initialize(const Eigen::Vector3d &offset, double max_movement_per_step) {
    offset_ = offset;
    max_movement_per_step_ = max_movement_per_step;
    initialized_ = true;
    return true;
}

// 计算目标位姿
bool FixedOffsetStrategy::computeTargetPose(
    const TrackingControlInput &input,
    TrackingControlOutput &output
) {
    if (!initialized_) {
        output.is_valid = false;
        output.message = "策略未初始化";
        return false;
    }

    // Step 1: 计算理想目标位姿（磁铁位置 + 固定偏移）
    output.target_pose.position.x = input.magnet_pose.position.x + offset_.x();
    output.target_pose.position.y = input.magnet_pose.position.y + offset_.y();
    output.target_pose.position.z = input.magnet_pose.position.z + offset_.z();

    // Step 2: 计算姿态（可选：朝向磁铁）
    // 简化处理：保持当前姿态或使用固定姿态
    output.target_pose.orientation = input.current_sensor_pose.orientation;

    // Step 3: 限制移动距离（平滑运动）
    limitMovement(input.current_sensor_pose.position, output.target_pose.position);

    // Step 4: 计算质量评分（可选：基于距离）
    double distance = computeDistance(input.current_sensor_pose.position, output.target_pose.position);
    output.quality_score = 1.0 / (1.0 + distance);  // 距离越小，评分越高

    output.is_valid = true;
    output.message = "成功计算目标位姿";
    return true;
}

// 限制移动距离
void FixedOffsetStrategy::limitMovement(
    const geometry_msgs::Point &current_pos,
    geometry_msgs::Point &target_pos
) const {
    double distance = computeDistance(current_pos, target_pos);
    
    if (distance > max_movement_per_step_) {
        // 按比例缩放移动距离
        double scale = max_movement_per_step_ / distance;
        target_pos.x = current_pos.x + (target_pos.x - current_pos.x) * scale;
        target_pos.y = current_pos.y + (target_pos.y - current_pos.y) * scale;
        target_pos.z = current_pos.z + (target_pos.z - current_pos.z) * scale;
    }
}

// 计算距离
double FixedOffsetStrategy::computeDistance(
    const geometry_msgs::Point &a,
    const geometry_msgs::Point &b
) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace mag_tracking_control

