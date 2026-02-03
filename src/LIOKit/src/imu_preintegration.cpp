#include "imu_preintegration.h"

#include "inner/utils.h"

namespace liokit {

using namespace ::gtsam::symbol_shorthand;

ImuPreintegration::ImuPreintegration(Option &option) : option_(option) {
  auto p = gtsam::PreintegrationParams::MakeSharedU(option_.imu_gravity);
  p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(option_.imu_acc_noise, 2);  // acc white noise in continuous
  p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(option_.imu_gyr_noise, 2);      // gyro white noise in continuous
  p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);  // error committed in integrating position from velocities
  gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

  imu_integrator_ =
      std::make_unique<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);  // setting up the IMU integration for IMU message
  imu_integrator_optimisation_ =
      std::make_unique<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);  // setting up the IMU integration for optimization

  //
  prior_pose_noise_ =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
  prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                // m/s
  prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
  correction_noise_ =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());    // rad,rad,rad,m, m, m
  correction_noise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());  // rad,rad,rad,m, m, m

  {
    auto &acc = option_.imu_acc_biasN;
    auto &gyr = option_.imu_acc_biasN;
    noise_model_between_bias_ = (gtsam::Vector(6) << acc, acc, acc, gyr, gyr, gyr).finished();
  }

  {  // TODO: get tf param
    double x, y, z;
    tf_imu2lidar_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-x, -y, -z));
    tf_lidar2imu_ = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(x, y, z));
  }
}

std::optional<gtsam::NavState> ImuPreintegration::OnImuData(const ImuDataInnerConstPtr &imu_data) {
  std::lock_guard<std::mutex> lock(mtx_);

  imu_queue_imu_.push_back(imu_data);
  imu_queue_optim_.push_back(imu_data);

  if (!done_first_optimized_) {
    return std::optional<gtsam::NavState>();
  }
  auto &timestamp = imu_data->timestamp;
  double dt = last_timestamp_imu_ < 0 ? option_.period_s : (timestamp - last_timestamp_imu_);
  last_timestamp_imu_ = timestamp;

  // integrate this single imu message
  imu_integrator_->integrateMeasurement(imu_data->acc, imu_data->omega, dt);

  // TODO: predict odometry
  auto cur_state = imu_integrator_->predict(pre_odom_state_, pre_odom_bias_);

  gtsam::Pose3 lidarPose = cur_state.pose().compose(tf_imu2lidar_);
  // cur_state.velocity() * ;
  return std::optional<gtsam::NavState>();
}

void ImuPreintegration::ResetOptimization() {
  gtsam::ISAM2Params optParameters;
  optParameters.relinearizeThreshold = 0.1;
  optParameters.relinearizeSkip = 1;
  optimizer_ = gtsam::ISAM2(optParameters);

  factor_graph_ = gtsam::NonlinearFactorGraph();
  graph_values_ = gtsam::Values();
}

void ImuPreintegration::ResetParams() {
  last_timestamp_imu_ = -1;
  done_first_optimized_ = false;
  system_initialized_ = false;
}

void ImuPreintegration::ResetFactorGraph() {
  // get updated noise before reset
  auto updated_pose_noise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_ - 1)));
  auto updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_ - 1)));
  auto updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_ - 1)));
  // reset graph
  ResetOptimization();
  // add pose
  gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), pre_pose_, updated_pose_noise);
  factor_graph_.add(priorPose);
  // add velocity
  gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), pre_vel_, updatedVelNoise);
  factor_graph_.add(priorVel);
  // add bias
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), pre_bias_, updatedBiasNoise);
  factor_graph_.add(priorBias);
  // add values
  graph_values_.insert(X(0), pre_pose_);
  graph_values_.insert(V(0), pre_vel_);
  graph_values_.insert(B(0), pre_bias_);
  // optimize once
  optimizer_.update(factor_graph_, graph_values_);
  factor_graph_.resize(0);
  graph_values_.clear();

  key_ = 1;
}

void ImuPreintegration::InitOptimization() {
  factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>  //
                    (X(0), pre_pose_, prior_pose_noise_));

  // initial velocity
  pre_vel_ = gtsam::Vector3(0, 0, 0);

  factor_graph_.add(gtsam::PriorFactor<gtsam::Vector3>  //
                    (V(0), pre_vel_, prior_vel_noise_));

  // initial bias
  pre_bias_ = gtsam::imuBias::ConstantBias();
  gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), pre_bias_, prior_bias_noise_);
  factor_graph_.add(priorBias);

  // add values
  graph_values_.insert(X(0), pre_pose_);
  graph_values_.insert(V(0), pre_vel_);
  graph_values_.insert(B(0), pre_bias_);

  // optimize once
  optimizer_.update(factor_graph_, graph_values_);
  factor_graph_.resize(0);
  graph_values_.clear();

  imu_integrator_optimisation_->resetIntegrationAndSetBias(pre_bias_);
  imu_integrator_->resetIntegrationAndSetBias(pre_bias_);

  key_ = 1;
}

bool ImuPreintegration::FailureDetection(const gtsam::Vector3 &vel, const gtsam::imuBias::ConstantBias &bias) {
  if (vel.norm() > 30) {
    // ROS_WARN("Large velocity, reset IMU-preintegration!");
    return true;
  }

  if (bias.accelerometer().norm() > 1.0 || bias.gyroscope().norm() > 1.0) {
    // ROS_WARN("Large bias, reset IMU-preintegration!");
    return true;
  }

  return false;
}

void ImuPreintegration::OnOdometryData(gtsam::Pose3 &pose, double timestamp) {
  std::lock_guard<std::mutex> lock(mtx_);

  // make sure we have imu data to integrate
  if (imu_queue_imu_.empty()) return;

  // 0. initialize system
  if (system_initialized_ == false) {
    ResetOptimization();

    // pop old IMU message
    utils::QueuePopOld(imu_queue_optim_, timestamp);

    pre_pose_ = pose.compose(tf_lidar2imu_);

    InitOptimization();
    system_initialized_ = true;
    return;
  }

  // reset graph for speed
  if (key_ == 100) {
    ResetFactorGraph();
  }

  // 1. integrate imu data and optimize
  utils::QueuePopOldWithHandle(imu_queue_optim_, timestamp, [this](const ImuDataInnerConstPtr &data) {
    double dt = (last_timestamp_opt_ < 0) ? option_.period_s : (data->timestamp - last_timestamp_opt_);
    last_timestamp_opt_ = data->timestamp;
    imu_integrator_optimisation_->integrateMeasurement(data->acc, data->omega, dt);
  });

  // add imu factor to graph

  auto &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imu_integrator_optimisation_);
  gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
  factor_graph_.add(imu_factor);
  // add imu bias between factor
  factor_graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_optimisation_->deltaTij()) * noise_model_between_bias_)));

  auto cur_pose = pose.compose(tf_lidar2imu_);
  gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), cur_pose, correction_noise_);
  factor_graph_.add(pose_factor);
  // insert predicted values
  gtsam::NavState propState_ = imu_integrator_optimisation_->predict(pre_state_, pre_bias_);
  graph_values_.insert(X(key_), propState_.pose());
  graph_values_.insert(V(key_), propState_.v());
  graph_values_.insert(B(key_), pre_bias_);
  // optimize
  optimizer_.update(factor_graph_, graph_values_);
  optimizer_.update();
  factor_graph_.resize(0);
  graph_values_.clear();
  // Overwrite the beginning of the preintegration for the next step.
  gtsam::Values result = optimizer_.calculateEstimate();
  pre_pose_ = result.at<gtsam::Pose3>(X(key_));
  pre_vel_ = result.at<gtsam::Vector3>(V(key_));
  pre_state_ = gtsam::NavState(pre_pose_, pre_vel_);
  pre_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key_));
  // Reset the optimization preintegration object.
  imu_integrator_optimisation_->resetIntegrationAndSetBias(pre_bias_);
  // check optimization
  if (FailureDetection(pre_vel_, pre_bias_)) {
    ResetParams();
    return;
  }

  // 2. after optiization, re-propagate imu odometry preintegration
  pre_odom_state_ = pre_state_;
  pre_odom_bias_ = pre_bias_;
  // first pop imu message older than current correction data
  utils::QueuePopOld(imu_queue_imu_, timestamp);

  // repropogate
  if (!imu_queue_imu_.empty()) {
    // reset bias use the newly optimized bias
    imu_integrator_->resetIntegrationAndSetBias(pre_odom_bias_);
    // integrate imu message from the beginning of this optimization
    double last_time = -1.;
    for (int i = 0; i < (int)imu_queue_imu_.size(); ++i) {
      auto &data = imu_queue_imu_.at(i);
      double dt = (last_time < 0) ? option_.period_s : (data->timestamp - last_time);
      last_time = data->timestamp;

      imu_integrator_->integrateMeasurement(data->acc, data->omega, dt);
    }
  }

  ++key_;
  done_first_optimized_ = true;
}

}  // namespace liokit
