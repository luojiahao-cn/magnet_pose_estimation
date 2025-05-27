#ifndef MAGNETIC_FIELD_CALCULATOR_HPP
#define MAGNETIC_FIELD_CALCULATOR_HPP

#include <Eigen/Dense>

namespace magnetic_pose_estimation
{

    class MagneticFieldCalculator
    {
    public:
        /**
         * @brief 计算多个传感器位置处的磁场
         * @param sensor_positions 传感器位置矩阵，每行表示一个传感器的位置
         * @param magnetic_position 磁偶极子的位置
         * @param magnetic_direction 磁偶极子的方向（单位向量）
         * @param magnetic_moment_size 磁矩大小
         * @return 各个传感器位置处的磁场向量矩阵
         */
        static Eigen::MatrixXd calculateMagneticField(
            const Eigen::Matrix<double, -1, 3> &sensor_positions,
            const Eigen::Vector3d &magnetic_position,
            const Eigen::Vector3d &magnetic_direction,
            double magnetic_moment_size);

        template <typename T>
        static Eigen::Matrix<T, 3, 1> calculateMagneticFieldT(
            const Eigen::Matrix<T, 3, 1> &sensor_pos,
            const Eigen::Matrix<T, 3, 1> &position,
            const Eigen::Matrix<T, 3, 1> &direction,
            T strength);
    };

} // namespace magnetic_pose_estimation

#endif // MAGNETIC_FIELD_CALCULATOR_HPP