#pragma once

#include "liokit/data/quaternion.h"
#include "liokit/data/vector3.h"

namespace liokit {

struct OdometryData {
  double timestamp;  // sec
  Vector3 point;
  Quaternion orientation;

  Vector3 v;  // linear
  Vector3 w;  // angular
};

}  // namespace liokit
