/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rbdyn/BodySensor.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_plugin
{

struct ObstacleDetectionJerkEstimator : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~ObstacleDetectionJerkEstimator() override;

private:

  double dt_; // Time step
  double t_; // Time
  int internal_counter_; // Internal counter

  double k_; // Gyro bias correction gain
  double alpha_rot_; // complementary filter gain for rotation matrix estimation
  double alpha_jerk_; // complementary filter gain for jerk estimation
  std::string imuBodyName_; // Name of the IMU sensor
  
  Eigen::Vector3d accelero_;
  Eigen::Vector3d bias_gyro_; // Gyro bias estimate
  Eigen::Vector3d bias_gyro_dot_; // Gyro bias derivative
  Eigen::Vector3d gyro_;
  Eigen::Matrix3d R_; // Estimated rotation matrix
  Eigen::Matrix3d R_dot_; // Estimated rotation matrix derivative
  Eigen::Matrix3d R_rob_; // Rotation matrix of the robot (model based)

  Eigen::Vector3d jerk_; // Estimated jerk = R^T * \dot{a}
  Eigen::Vector3d jerk_dot_; // Estimated jerk derivative
  Eigen::Vector3d jerk_no_filtered_; // Estimated jerk without filtering
  Eigen::Vector3d omega_; // Angular velocity
  Eigen::Vector3d accelero_dot_; // Derivative of the accelerometer measurement

};

} // namespace mc_plugin
