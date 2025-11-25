#include "mag_tracking_control/strategy_adaptive_distance.h"

#include <cmath>
#include <algorithm>

namespace mag_tracking_control {

AdaptiveDistanceStrategy::AdaptiveDistanceStrategy()
    : target_field_strength_(35.0)
    , min_field_strength_(20.0)
    , max_field_strength_(50.0)
    , adjustment_gain_(0.02)
    , max_movement_per_step_(0.02)
{
}

bool AdaptiveDistanceStrategy::initialize(
    double target_field_strength,
    double min_field_strength,
    double max_field_strength,
    double adjustment_gain,
    double max_movement_per_step
) {
    target_field_strength_ = target_field_strength;
    min_field_strength_ = min_field_strength;
    max_field_strength_ = max_field_strength;
    adjustment_gain_ = adjustment_gain;
    max_movement_per_step_ = max_movement_per_step;
    initialized_ = true;
    return true;
}

bool AdaptiveDistanceStrategy::computeTargetPose(
    const TrackingControlInput &input,
    TrackingControlOutput &output
) {
    if (!initialized_) {
        output.is_valid = false;
        output.message = "策略未初始化";
        return false;
    }

    if (input.field_strengths.empty()) {
        output.is_valid = false;
        output.message = "缺少磁场强度数据";
        return false;
    }

    // Step 1: 计算平均磁场强度
    double avg_field = computeAverageFieldStrength(input.field_strengths);

    // Step 2: 计算当前距离
    double current_distance = computeDistance(
        input.current_sensor_pose.position,
        input.magnet_pose.position
    );

    // Step 3: 根据磁场强度计算距离调整
    double distance_adjustment = 0.0;
    if (avg_field > max_field_strength_) {
        // 磁场太强，需要远离
        double error = avg_field - target_field_strength_;
        distance_adjustment = adjustment_gain_ * error / target_field_strength_;
    } else if (avg_field < min_field_strength_) {
        // 磁场太弱，需要靠近
        double error = target_field_strength_ - avg_field;
        distance_adjustment = -adjustment_gain_ * error / target_field_strength_;
    }

    // Step 4: 计算目标位置（沿着传感器到磁铁的方向）
    Eigen::Vector3d direction = computeDirectionToMagnet(
        input.current_sensor_pose.position,
        input.magnet_pose.position
    );

    // 新距离 = 当前距离 + 调整量
    double target_distance = current_distance + distance_adjustment;
    
    // 限制目标距离在合理范围内（避免过近或过远）
    double min_distance = 0.01;  // 最小距离 1cm
    double max_distance = 0.20;  // 最大距离 20cm
    target_distance = std::max(min_distance, std::min(max_distance, target_distance));

    // 从磁铁位置沿相反方向计算目标位置
    output.target_pose.position.x = input.magnet_pose.position.x - direction.x() * target_distance;
    output.target_pose.position.y = input.magnet_pose.position.y - direction.y() * target_distance;
    output.target_pose.position.z = input.magnet_pose.position.z - direction.z() * target_distance;

    // Step 5: 保持当前姿态
    output.target_pose.orientation = input.current_sensor_pose.orientation;

    // Step 6: 限制移动距离（平滑运动）
    limitMovement(input.current_sensor_pose.position, output.target_pose.position);

    // Step 7: 计算质量评分（基于磁场强度是否在目标范围）
    double field_error = std::abs(avg_field - target_field_strength_);
    output.quality_score = 1.0 / (1.0 + field_error / target_field_strength_);

    output.is_valid = true;
    output.message = "成功计算目标位姿，平均磁场强度: " + std::to_string(avg_field) + " mT";
    return true;
}

double AdaptiveDistanceStrategy::computeAverageFieldStrength(
    const std::vector<Eigen::Vector3d> &fields
) const {
    if (fields.empty()) {
        return 0.0;
    }

    double total_magnitude = 0.0;
    for (const auto &field : fields) {
        total_magnitude += field.norm();
    }
    return total_magnitude / fields.size();
}

Eigen::Vector3d AdaptiveDistanceStrategy::computeDirectionToMagnet(
    const geometry_msgs::Point &sensor_pos,
    const geometry_msgs::Point &magnet_pos
) const {
    Eigen::Vector3d direction(
        magnet_pos.x - sensor_pos.x,
        magnet_pos.y - sensor_pos.y,
        magnet_pos.z - sensor_pos.z
    );
    
    double norm = direction.norm();
    if (norm > 1e-6) {
        direction.normalize();
    } else {
        // 如果距离太近，使用默认方向（向上）
        direction = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    
    return direction;
}

void AdaptiveDistanceStrategy::limitMovement(
    const geometry_msgs::Point &current_pos,
    geometry_msgs::Point &target_pos
) const {
    double distance = computeDistance(current_pos, target_pos);
    
    if (distance > max_movement_per_step_) {
        double scale = max_movement_per_step_ / distance;
        target_pos.x = current_pos.x + (target_pos.x - current_pos.x) * scale;
        target_pos.y = current_pos.y + (target_pos.y - current_pos.y) * scale;
        target_pos.z = current_pos.z + (target_pos.z - current_pos.z) * scale;
    }
}

double AdaptiveDistanceStrategy::computeDistance(
    const geometry_msgs::Point &a,
    const geometry_msgs::Point &b
) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace mag_tracking_control

