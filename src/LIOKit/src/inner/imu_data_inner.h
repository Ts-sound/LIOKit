#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <memory>

#include "liokit/data/imu_data.h"

namespace liokit {

struct ImuDataInner {
  double timestamp;      // sec
  gtsam::Vector3 acc;    // linear acceleration
  gtsam::Vector3 omega;  // angular velocity
};

typedef std::shared_ptr<ImuDataInner> ImuDataInnerPtr;
typedef std::shared_ptr<const ImuDataInner> ImuDataInnerConstPtr;

}