#pragma once
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>

namespace magnetic_pose_estimation {

class BaseMagnetPoseEstimator {
public:
    virtual ~BaseMagnetPoseEstimator() = default;
    virtual void magneticFieldCallback(const MagneticField::ConstPtr& msg) = 0;
    virtual bool resetServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) = 0;
};

} // namespace magnetic_pose_estimation