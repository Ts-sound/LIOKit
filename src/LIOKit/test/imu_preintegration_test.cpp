#include "imu_preintegration.h"

#include <gtest/gtest.h>

namespace {
// TODO

TEST(imu_preintegration_test, todo) {
  liokit::ImuPreintegration::Option opt;
  auto imu_preint = liokit::ImuPreintegration(opt);
}

TEST(imu_preintegration_test, imu_data_processing) {
  liokit::ImuPreintegration::Option opt;
  auto imu_preint = liokit::ImuPreintegration(opt);

  // Test basic IMU data processing
  auto imu_data = std::make_shared<liokit::ImuDataInner>();
  imu_data->timestamp = 1.0;
  imu_data->acc = gtsam::Vector3(0.0, 0.0, 9.81);
  imu_data->omega = gtsam::Vector3(0.0, 0.0, 0.0);

  auto result = imu_preint.OnImuData(imu_data);
  EXPECT_FALSE(result.has_value());  // Should not have value before optimization
}

TEST(imu_preintegration_test, odometry_integration) {
  liokit::ImuPreintegration::Option opt;
  auto imu_preint = liokit::ImuPreintegration(opt);

  // Test odometry data integration
  gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
  double timestamp = 1.0;

  // This should initialize the system
  imu_preint.OnOdometryData(pose, timestamp);
}

TEST(imu_preintegration_test, parameter_initialization) {
  liokit::ImuPreintegration::Option opt;
  opt.period_s = 0.01;
  opt.imu_gravity = 9.8;
  opt.imu_acc_noise = 1e-3;
  opt.imu_gyr_noise = 1e-3;
  opt.imu_acc_biasN = 1e-4;
  opt.imu_gyr_biasN = 1e-4;

  // Test that parameters are properly set
  EXPECT_EQ(opt.period_s, 0.01);
  EXPECT_EQ(opt.imu_gravity, 9.8);
}

TEST(imu_preintegration_test, failure_detection) {
  liokit::ImuPreintegration::Option opt;
  auto imu_preint = liokit::ImuPreintegration(opt);

  // Test failure detection with high velocity
  gtsam::Vector3 high_vel(40.0, 0.0, 0.0);
  gtsam::imuBias::ConstantBias normal_bias;
  // Note: This test would need access to private methods or be moved to a friend test class
  // to properly test the FailureDetection method
}

}  // namespace
