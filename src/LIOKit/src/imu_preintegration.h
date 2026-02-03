#pragma once

#include <deque>
#include <mutex>

//
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

//

#include "inner/imu_data_inner.h"

namespace liokit {

class ImuPreintegration {
  public:
  struct Option {
    double period_s = 0.002;    // 1/HZ
    double imu_gravity = 9.81;  //
    double imu_acc_noise = 1e-4;
    double imu_gyr_noise = 1e-4;
    double imu_acc_biasN = 6.0e-5;
    double imu_gyr_biasN = 6.0e-5;
  };

  public:
  ImuPreintegration(const Option &option);

  public:
  void OnOdometryData(gtsam::Pose3 &pose, double timestamp);

  std::optional<gtsam::NavState> OnImuData(const ImuDataInnerConstPtr &imu_data);

  private:
  void ResetOptimization();
  void ResetParams();
  void ResetFactorGraph();

  void InitOptimization();

  bool FailureDetection(const gtsam::Vector3 &vel, const gtsam::imuBias::ConstantBias &bias);

  private:
  typedef std::unique_ptr<gtsam::PreintegratedImuMeasurements> ImuIntegratorUptr;
  typedef gtsam::noiseModel::Diagonal::shared_ptr NoisePtr;

  Option option_;

  // tf
  gtsam::Pose3 tf_imu2lidar_;
  // tramsform points from imu frame to lidar frame
  gtsam::Pose3 tf_lidar2imu_ ;

  private:
  NoisePtr prior_pose_noise_;
  NoisePtr prior_vel_noise_;
  NoisePtr prior_bias_noise_;
  NoisePtr correction_noise_;
  NoisePtr correction_noise2_;
  gtsam::Vector noise_model_between_bias_;

  ImuIntegratorUptr imu_integrator_optimisation_;
  ImuIntegratorUptr imu_integrator_;

  //

  std::deque<ImuDataInnerConstPtr> imu_queue_imu_;
  std::deque<ImuDataInnerConstPtr> imu_queue_optim_;

  int key_ = 1;

  gtsam::Pose3 pre_pose_;
  gtsam::Vector3 pre_vel_;
  gtsam::NavState pre_state_;
  gtsam::imuBias::ConstantBias pre_bias_;

  gtsam::NavState pre_odom_state_;
  gtsam::imuBias::ConstantBias pre_odom_bias_;

  bool done_first_optimized_ = false;
  bool system_initialized_ = false;
  double last_timestamp_imu_ = -1.;
  double last_timestamp_opt_ = -1.;

  gtsam::ISAM2 optimizer_;
  gtsam::NonlinearFactorGraph factor_graph_;
  gtsam::Values graph_values_;

  std::mutex mtx_;
};

}