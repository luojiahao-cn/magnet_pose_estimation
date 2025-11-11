#include <ceres/ceres.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mag_pose_estimation/mag_pose_estimator_optimization.hpp>

namespace mag_pose_estimation
{

    OptimizationMagnetPoseEstimator::OptimizationMagnetPoseEstimator(ros::NodeHandle &pnh) : pnh_(pnh)
    {
        loadParameters();
        ROS_INFO("磁铁位置估计器（优化）已初始化");
    }

    void OptimizationMagnetPoseEstimator::loadParameters()
    {
        std::vector<double> position_vec, direction_vec;
        if (!pnh_.getParam("estimator_config/magnet/position", position_vec) || position_vec.size() != 3)
        {
            throw std::runtime_error("缺少或非法参数: ~estimator_config/magnet/position[3]");
        }
        if (!pnh_.getParam("estimator_config/magnet/direction", direction_vec) || direction_vec.size() != 3)
        {
            throw std::runtime_error("缺少或非法参数: ~estimator_config/magnet/direction[3]");
        }
        if (!pnh_.getParam("estimator_config/magnet/strength", initial_strength_))
        {
            throw std::runtime_error("缺少参数: ~estimator_config/magnet/strength");
        }
        if (!pnh_.getParam("estimator_config/magnet/strength_delta", strength_delta_))
        {
            throw std::runtime_error("缺少参数: ~estimator_config/magnet/strength_delta");
        }

        optimize_strength_ = (strength_delta_ != 0.0);
        strength_min_ = std::max(0.0, initial_strength_ - (optimize_strength_ ? strength_delta_ : 0.0));
        strength_max_ = initial_strength_ + (optimize_strength_ ? strength_delta_ : 0.0);

        initial_position_ = Eigen::Vector3d(position_vec[0], position_vec[1], position_vec[2]);
        initial_direction_ = Eigen::Vector3d(direction_vec[0], direction_vec[1], direction_vec[2]).normalized();
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;

        if (!pnh_.getParam("estimator_config/optimization/max_iterations", max_iterations_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/max_iterations");
        if (!pnh_.getParam("estimator_config/optimization/function_tolerance", function_tolerance_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/function_tolerance");
        if (!pnh_.getParam("estimator_config/optimization/gradient_tolerance", gradient_tolerance_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/gradient_tolerance");
        if (!pnh_.getParam("estimator_config/optimization/parameter_tolerance", parameter_tolerance_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/parameter_tolerance");
        if (!pnh_.getParam("estimator_config/optimization/num_threads", num_threads_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/num_threads");
        if (!pnh_.getParam("estimator_config/optimization/minimizer_progress_to_stdout", minimizer_progress_to_stdout_))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/minimizer_progress_to_stdout");

        std::string linear_solver_str;
        if (!pnh_.getParam("estimator_config/optimization/linear_solver_type", linear_solver_str))
            throw std::runtime_error("缺少参数: ~estimator_config/optimization/linear_solver_type");
        if (linear_solver_str == "DENSE_QR")
            linear_solver_type_ = ceres::DENSE_QR;
        else
            linear_solver_type_ = ceres::DENSE_QR;

        ROS_INFO("基线优化参数加载完成");
    }

    void OptimizationMagnetPoseEstimator::reset()
    {
        resetToInitialParameters();
    }

    bool OptimizationMagnetPoseEstimator::estimate(const std::map<int, MagneticField> &measurements,
                                                   const std::map<int, geometry_msgs::Pose> &sensor_poses,
                                                   MagnetPose &out_pose,
                                                   double *out_error)
    {
        double position[3] = {current_position_.x(), current_position_.y(), current_position_.z()};
        double direction[3] = {magnetic_direction_.x(), magnetic_direction_.y(), magnetic_direction_.z()};
        double strength = magnet_strength_;

        ceres::Problem problem;

        for (const auto &m : measurements)
        {
            auto it = sensor_poses.find(m.first);
            if (it == sensor_poses.end()) {
                ROS_ERROR("[optimization] 传感器位置缺失 ID=%d", m.first);
                return false;
            }
            Eigen::Vector3d sensor_pos(it->second.position.x, it->second.position.y, it->second.position.z);
            Eigen::Vector3d measured_field(m.second.mag_x, m.second.mag_y, m.second.mag_z);

            if (optimize_strength_)
            {
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<MagnetFieldResidualWithStrength, 3, 3, 3, 1>(
                                             new MagnetFieldResidualWithStrength(sensor_pos, measured_field)),
                                         nullptr,
                                         position,
                                         direction,
                                         &strength);
            }
            else
            {
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<MagnetFieldResidual, 3, 3, 3>(
                                             new MagnetFieldResidual(sensor_pos, measured_field, magnet_strength_)),
                                         nullptr,
                                         position,
                                         direction);
            }
        }

        problem.SetParameterization(direction, new ceres::HomogeneousVectorParameterization(3));

        if (optimize_strength_)
        {
            problem.SetParameterLowerBound(&strength, 0, strength_min_);
            problem.SetParameterUpperBound(&strength, 0, strength_max_);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = linear_solver_type_;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.max_num_iterations = max_iterations_;
        options.function_tolerance = function_tolerance_;
        options.gradient_tolerance = gradient_tolerance_;
        options.parameter_tolerance = parameter_tolerance_;
        options.num_threads = num_threads_;
        options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (out_error)
            *out_error = summary.final_cost;

        current_position_ = Eigen::Vector3d(position[0], position[1], position[2]);
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
        if (optimize_strength_)
        {
            magnet_strength_ = strength;
        }

        // Fill out_pose message
        out_pose.header.stamp = ros::Time::now();
        out_pose.header.frame_id = "world";
        out_pose.position.x = current_position_.x();
        out_pose.position.y = current_position_.y();
        out_pose.position.z = current_position_.z();

        tf2::Vector3 z_axis(0, 0, 1);
        tf2::Vector3 dir_v(magnetic_direction_.x(), magnetic_direction_.y(), magnetic_direction_.z());
        tf2::Vector3 axis = z_axis.cross(dir_v);
        double angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(dir_v))));
        if (axis.length() < 1e-6)
        {
            if (dir_v.z() > 0)
            {
                out_pose.orientation.w = 1.0;
                out_pose.orientation.x = 0.0;
                out_pose.orientation.y = 0.0;
                out_pose.orientation.z = 0.0;
            }
            else
            {
                out_pose.orientation.w = 0.0;
                out_pose.orientation.x = 1.0;
                out_pose.orientation.y = 0.0;
                out_pose.orientation.z = 0.0;
            }
        }
        else
        {
            axis.normalize();
            tf2::Quaternion q(axis, angle);
            q.normalize();
            out_pose.orientation = tf2::toMsg(q);
        }
        out_pose.magnetic_strength = magnet_strength_;
        return true;
    }

    void OptimizationMagnetPoseEstimator::resetToInitialParameters()
    {
        current_position_ = initial_position_;
        magnetic_direction_ = initial_direction_;
        magnet_strength_ = initial_strength_;
    }

    // publish function removed; node handles publishing now

} // namespace mag_pose_estimation
