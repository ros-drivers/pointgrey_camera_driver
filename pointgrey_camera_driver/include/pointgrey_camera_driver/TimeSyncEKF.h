#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <ros/ros.h>

namespace time_sync {

class TimeSyncEKF
{

 public:
  static constexpr double kSigmaInitOffset = 2e-3;
  static constexpr double kSigmaInitSkew = 1e-3;
  static constexpr double kSigmaMeasurementTimeOffset = 2e-3;
  static constexpr double kSigmaSkew = 2e-6;
  static constexpr double kUpdateRate = 0.5;

  TimeSyncEKF();
  ~TimeSyncEKF();
  double getLocalTimestamp(double device_time);
  void updateFilter(double device_time, double local_time);
  void print();
  void reset();

 private:
  void initialize(double device_time, double local_time);
  bool isOutlier(double device_time, double local_time);

  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  double R_;
  Eigen::Matrix<double, 1, 2> H_;
  bool isInitialized_;
  double lastUpdateDeviceTime_;
  double dt_;

};

}
