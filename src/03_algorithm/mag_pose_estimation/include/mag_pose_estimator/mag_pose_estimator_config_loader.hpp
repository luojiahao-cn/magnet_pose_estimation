#pragma once

#include <XmlRpcValue.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mag_pose_estimator
{

struct MagPoseEstimatorConfig
{
    // Frames
    std::string output_frame;

    // Topics
    std::string mag_topic;
    std::string pose_topic;

    // Params
    std::string estimator_type;
    int min_sensors;
    double tf_timeout;

    // Preprocessor
    bool enable_calibration;
    Eigen::Matrix3d soft_iron_matrix;
    Eigen::Vector3d hard_iron_offset;
    bool enable_filter;
    double low_pass_alpha;

    // Estimator
    double position_gain;
    double process_noise_position;
    double process_noise_orientation;
    double measurement_noise;
    int optimizer_iterations;
    double optimizer_damping;
    Eigen::Vector3d world_field;

    // Optimizer
    struct OptimizerParams
    {
        Eigen::Vector3d initial_position;
        Eigen::Vector3d initial_direction;
        double initial_strength;
        double strength_delta;
        bool optimize_strength;
        int max_iterations;
        double function_tolerance;
        double gradient_tolerance;
        double parameter_tolerance;
        int num_threads;
        bool minimizer_progress;
        std::string linear_solver;
    } optimizer;
};

MagPoseEstimatorConfig loadMagPoseEstimatorConfig(const XmlRpc::XmlRpcValue &root,
                                                   const std::string &context);

} // namespace mag_pose_estimator