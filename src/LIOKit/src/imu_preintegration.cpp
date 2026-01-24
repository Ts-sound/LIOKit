#include "imu_preintegration.h"

namespace liokit {
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
}

void ImuPreintegration::OnOdometryData(gtsam::Pose3 &pose, double timestamp) {}

gtsam::NavState ImuPreintegration::OnImuData(const gtsam::Vector3 &acc, const gtsam::Vector3 &omega, double timestamp) {
  // TODO: done_first_optimized_ == true
  double dt = last_timestamp_imu_ < 0 ? option_.period_ms : (timestamp - last_timestamp_imu_);
  last_timestamp_imu_ = timestamp;

  // integrate this single imu message
  imu_integrator_->integrateMeasurement(acc, omega, dt);

  // predict odometry
  return imu_integrator_->predict(pre_odom_state_, pre_odom_bias_);
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

// TODO:
void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  std::lock_guard<std::mutex> lock(mtx);

  double currentCorrectionTime = ROS_TIME(odomMsg);

  // make sure we have imu data to integrate
  if (imuQueOpt.empty()) return;

  float p_x = odomMsg->pose.pose.position.x;
  float p_y = odomMsg->pose.pose.position.y;
  float p_z = odomMsg->pose.pose.position.z;
  float r_x = odomMsg->pose.pose.orientation.x;
  float r_y = odomMsg->pose.pose.orientation.y;
  float r_z = odomMsg->pose.pose.orientation.z;
  float r_w = odomMsg->pose.pose.orientation.w;
  bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
  gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

  // 0. initialize system
  if (systemInitialized == false) {
    resetOptimization();

    // pop old IMU message
    while (!imuQueOpt.empty()) {
      if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
        lastImuT_opt = ROS_TIME(&imuQueOpt.front());
        imuQueOpt.pop_front();
      } else
        break;
    }
    // initial pose
    prevPose_ = lidarPose.compose(lidar2Imu);
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
    graphFactors.add(priorPose);
    // initial velocity
    prevVel_ = gtsam::Vector3(0, 0, 0);
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
    graphFactors.add(priorVel);
    // initial bias
    prevBias_ = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
    graphFactors.add(priorBias);
    // add values
    graphValues.insert(X(0), prevPose_);
    graphValues.insert(V(0), prevVel_);
    graphValues.insert(B(0), prevBias_);
    // optimize once
    optimizer.update(graphFactors, graphValues);
    graphFactors.resize(0);
    graphValues.clear();

    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    key = 1;
    systemInitialized = true;
    return;
  }

  // reset graph for speed
  if (key == 100) {
    // get updated noise before reset
    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise =
        gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
    // reset graph
    resetOptimization();
    // add pose
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
    graphFactors.add(priorPose);
    // add velocity
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
    graphFactors.add(priorVel);
    // add bias
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
    graphFactors.add(priorBias);
    // add values
    graphValues.insert(X(0), prevPose_);
    graphValues.insert(V(0), prevVel_);
    graphValues.insert(B(0), prevBias_);
    // optimize once
    optimizer.update(graphFactors, graphValues);
    graphFactors.resize(0);
    graphValues.clear();

    key = 1;
  }

  // 1. integrate imu data and optimize
  while (!imuQueOpt.empty()) {
    // pop and integrate imu data that is between two optimizations
    sensor_msgs::Imu *thisImu = &imuQueOpt.front();
    double imuTime = ROS_TIME(thisImu);
    if (imuTime < currentCorrectionTime - delta_t) {
      double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
      imuIntegratorOpt_->integrateMeasurement(
          gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
          gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

      lastImuT_opt = imuTime;
      imuQueOpt.pop_front();
    } else
      break;
  }
  // add imu factor to graph
  const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
  gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
  graphFactors.add(imu_factor);
  // add imu bias between factor
  graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
  // add pose factor
  gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
  gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
  graphFactors.add(pose_factor);
  // insert predicted values
  gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
  graphValues.insert(X(key), propState_.pose());
  graphValues.insert(V(key), propState_.v());
  graphValues.insert(B(key), prevBias_);
  // optimize
  optimizer.update(graphFactors, graphValues);
  optimizer.update();
  graphFactors.resize(0);
  graphValues.clear();
  // Overwrite the beginning of the preintegration for the next step.
  gtsam::Values result = optimizer.calculateEstimate();
  prevPose_ = result.at<gtsam::Pose3>(X(key));
  prevVel_ = result.at<gtsam::Vector3>(V(key));
  prevState_ = gtsam::NavState(prevPose_, prevVel_);
  prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
  // Reset the optimization preintegration object.
  imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
  // check optimization
  if (failureDetection(prevVel_, prevBias_)) {
    resetParams();
    return;
  }

  // 2. after optiization, re-propagate imu odometry preintegration
  prevStateOdom = prevState_;
  prevBiasOdom = prevBias_;
  // first pop imu message older than current correction data
  double lastImuQT = -1;
  while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
    lastImuQT = ROS_TIME(&imuQueImu.front());
    imuQueImu.pop_front();
  }
  // repropogate
  if (!imuQueImu.empty()) {
    // reset bias use the newly optimized bias
    imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
    // integrate imu message from the beginning of this optimization
    for (int i = 0; i < (int)imuQueImu.size(); ++i) {
      sensor_msgs::Imu *thisImu = &imuQueImu[i];
      double imuTime = ROS_TIME(thisImu);
      double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

      imuIntegratorImu_->integrateMeasurement(
          gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
          gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
      lastImuQT = imuTime;
    }
  }

  ++key;
  doneFirstOpt = true;
}

bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
  Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
  if (vel.norm() > 30) {
    ROS_WARN("Large velocity, reset IMU-preintegration!");
    return true;
  }

  Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
  Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
  if (ba.norm() > 1.0 || bg.norm() > 1.0) {
    ROS_WARN("Large bias, reset IMU-preintegration!");
    return true;
  }

  return false;
}

}  // namespace liokit
