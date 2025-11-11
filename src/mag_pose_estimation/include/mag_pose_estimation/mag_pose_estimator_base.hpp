#pragma once
#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <ros/ros.h>

#include <map>

namespace mag_pose_estimation
{

    // Alias ROS message types from new package name to this algorithm namespace
    using MagneticField = magnet_msgs::MagSensorData;
    using MagnetPose = magnet_msgs::MagnetPose;

    class BaseMagnetPoseEstimator
    {
    public:
        virtual ~BaseMagnetPoseEstimator() = default;
        // Reset internal state to initial parameters
        virtual void reset() = 0;
        // Run one estimation step on a batch of measurements (keyed by sensor_id)
        // Returns true on success and fills out_pose; out_error is optional (can be nullptr)
        virtual bool estimate(const std::map<int, MagneticField> &measurements, const std::map<int, geometry_msgs::Pose> &sensor_poses, MagnetPose &out_pose, double *out_error) = 0;
    };

} // namespace mag_pose_estimation
