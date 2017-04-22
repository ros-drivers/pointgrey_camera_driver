#include<pointgrey_camera_driver/TimeSyncEKF.h>

namespace time_sync{

 constexpr double TimeSyncEKF::kSigmaInitOffset;
 constexpr double TimeSyncEKF::kSigmaInitSkew;
 constexpr double TimeSyncEKF::kSigmaMeasurementTimeOffset;
 constexpr double TimeSyncEKF::kSigmaSkew;
 constexpr double TimeSyncEKF::kUpdateRate;


TimeSyncEKF::TimeSyncEKF():
    isInitialized_(false) {

}

TimeSyncEKF::~TimeSyncEKF(){

}

double TimeSyncEKF::getLocalTimestamp(double device_time) {
  if(!isInitialized_) {
    std::cout << "[WARN]: Timesync filter not initialized yet! Hack initializing now.";
    double local_time = ros::Time::now().toSec();
    initialize(device_time, local_time);
  }
  double dt = device_time - lastUpdateDeviceTime_;
  return device_time + x_(0) + dt * x_(1);
}

void TimeSyncEKF::print() {
  std::cout << "offset: " << x_(0) << "skew: " << x_(1) << "dt" << dt_ << std::endl;
  // std::cout << P_<< std::endl;
}

void TimeSyncEKF::reset() {
  isInitialized_ = 0;
}

void TimeSyncEKF::updateFilter(double device_time, double local_time) {
  if(!isInitialized_) {
    initialize(device_time, local_time);
    return;
  }

  double dt = device_time - lastUpdateDeviceTime_;

  if(dt < kUpdateRate)
    return;

  dt_=dt;
  Eigen::Matrix2d F;
  F << 1, dt_, 0, 1;

  // Prediction
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + dt_ * Q_;

  // Update
  double S = H_ * P_ * H_.transpose() + R_;

  Eigen::Vector2d K;
  K = P_ * H_.transpose() * (1 / S);

  double measurement_residual = local_time - device_time - H_ * x_;

  x_ = x_ + K * measurement_residual;
  P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;

  lastUpdateDeviceTime_ = device_time;

  //print();
}

void TimeSyncEKF::initialize(double device_time, double local_time){
  x_.setZero();
  x_[0] = local_time - device_time;

  P_.setZero();
  P_(0,0) = kSigmaInitOffset * kSigmaInitOffset;
  P_(1,1) = kSigmaInitSkew * kSigmaInitSkew;

  Q_.setZero();
  Q_(1,1) = kSigmaSkew * kSigmaSkew;

  R_ = kSigmaMeasurementTimeOffset * kSigmaMeasurementTimeOffset;

  H_.setZero();
  H_(0,0) = 1;

  lastUpdateDeviceTime_ = device_time;
  isInitialized_ = true;
}

}
