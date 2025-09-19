#pragma once
#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <ros/ros.h>

#include <map>

namespace mag_pose_estimation
{

    // Alias ROS message types from new package name to this algorithm namespace
    using MagneticField = mag_sensor_node::MagSensorData;
    using MagnetPose = mag_sensor_node::MagnetPose;

    class BaseMagnetPoseEstimator
    {
    public:
        virtual ~BaseMagnetPoseEstimator() = default;
        // Reset internal state to initial parameters
        virtual void reset() = 0;
        // Run one estimation step on a batch of measurements (keyed by sensor_id)
        // Returns true on success and fills out_pose; out_error is optional (can be nullptr)
        virtual bool estimate(const std::map<int, MagneticField> &measurements, MagnetPose &out_pose, double *out_error) = 0;
    };

} // namespace mag_pose_estimation
