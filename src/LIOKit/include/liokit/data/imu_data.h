#pragma once

#include "liokit/data/quaternion.h"
#include "liokit/data/vector3.h"

namespace liokit {

struct ImuData {
  double timestamp;  // sec
  Quaternion orientation;
  Vector3 v_acc;  // linear acceleration
  Vector3 w;      // angular velocity
};

struct ImuCovarianceData {
  ImuData imu_data;
  double orientation_covariance[9];
  double v_acc_covariance[9];  // linear acceleration covariance
  double w_covariance[9];      // angular velocity covariance
};

}  // namespace liokit
