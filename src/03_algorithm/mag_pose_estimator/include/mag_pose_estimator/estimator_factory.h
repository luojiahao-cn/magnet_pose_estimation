#pragma once

#include <memory>
#include <string>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

std::unique_ptr<EstimatorBase> createEstimator(const std::string &type);

}  // namespace mag_pose_estimator
