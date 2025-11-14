#pragma once

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>

namespace mag_pose_estimator {

struct EstimatorConfig {
  Eigen::Vector3d world_field;            // Magnetic field expressed in world frame (Tesla)
  double process_noise_position;          // Process noise for XYZ random walk
  double process_noise_orientation;       // Process noise for quaternion random walk
  double measurement_noise;               // Measurement noise variance for normalized magnetic vector
  double position_gain;                   // Gain that maps field magnitude error to position change
  int optimizer_iterations;               // Iterations for optimizer-based solvers
  double optimizer_damping;               // Damping for LM/Gauss-Newton step
};

class EstimatorBase {
public:
  virtual ~EstimatorBase() = default;

  virtual void initialize() = 0;
  virtual void update(const sensor_msgs::MagneticField &mag) = 0;
  virtual geometry_msgs::Pose getPose() const = 0;

  virtual std::string name() const = 0;

  inline void setConfig(const EstimatorConfig &config) {
    config_ = config;
  }

protected:
  EstimatorConfig config_;
  bool initialized_ = false;
};

}  // namespace mag_pose_estimator
