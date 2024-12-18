/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rbdyn/BodySensor.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <string>
#include <mc_tvm/Robot.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FA.h>

namespace mc_plugin
{

struct ObstacleDetectionJerkEstimator : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  void addGui(mc_control::MCGlobalController & ctl);
  void addLog(mc_control::MCGlobalController & ctl);
  void addPlot(mc_control::MCGlobalController & ctl);
  void removePlot(mc_control::MCGlobalController & ctl);

  void jerkEstimationBase(mc_control::MCGlobalController & ctl);
  void jerkEstimationWithLinearVelocity(mc_control::MCGlobalController & ctl);
  void jerkEstimationWithoutModel(mc_control::MCGlobalController & ctl);
  void jerkEstimationFromQP(mc_control::MCGlobalController & ctl);

  std::string getEstimationType(void);
  void setEstimationType(std::string type);

  enum EstimationType
  {
    Base,
    WithLinearVelocity,
    WithoutModel,
    FromQP
  };

  ~ObstacleDetectionJerkEstimator() override;

private:
  std::vector<std::string> plots_;
  double dt_; // Time step
  double obstacle_threshold_; // Threshold to detect an obstacle
  int internal_counter_; // Internal counter
  std::string imuBodyName_; // Name of the IMU sensor
  std::string robotBodyName_;
  std::string bodySensor_name_ ;

  int estimationType_; // Type of jerk estimation: 0 -> base, 1 -> with linear velocity, 2 -> without model, 3 -> from QP

  // Gains
  double k_; // Gyro bias correction gain
  double k_vel_; // Gyro bias correction gain including the linear velocity
  double alpha_rot_; // Complementary filter gain for rotation matrix estimation
  double alpha_rot_vel_; // Complementary filter gain for rotation matrix estimation from the linear velocity estimation
  double alpha_v_; // Complementary filter gain for the linear velocity part of the rotation matrix estimation
  double alpha_jerk_; // Complementary filter gain for jerk estimation
  double alpha_jerk_vel_; // Complementary filter gain for jerk estimation including the linear velocity
  double alpha_acc_; // Complementary filter gain for acceleration estimation

  // Flags
  bool remove_plot_flag_;
  bool reset_plot_flag_;
  bool imu_not_yet_initialized_;
  bool collision_stop_activated_;
  bool obstacle_detected_; // Flag to indicate if an obstacle is detected
  bool obstacle_detection_has_changed_; // Flag to indicate if the obstacle detection has changed

  bool jerkEstimationBaseFlag_;
  bool jerkEstimationWithLinearVelocityFlag_;
  bool jerkEstimationWithoutModelFlag_;
  bool jerkEstimationFromQPFlag_;
  
  // IMU measurements
  Eigen::Vector3d accelero_;
  Eigen::Vector3d accelero_dot_; // Derivative of the accelerometer measurements
  Eigen::Vector3d accelero_dot_dot_; // Second derivative of the accelerometer measurements
  Eigen::Vector3d prev_accelero_;
  Eigen::Vector3d prev_accelero_dot_;
  Eigen::Vector3d gyro_;
  Eigen::Vector3d prev_gyro_;
  Eigen::Vector3d gyro_dot_;

  // Rotation matrix of the robot (model based)
  Eigen::Matrix3d R_rob_;
  Eigen::Quaternion<double> quat_R_rob_;

  // Linear velocity from the robot model
  Eigen::Vector3d v_encoders_;

  // No model estimation
  Eigen::Vector3d jerk_withoutModel_; // Estimated jerk = R^T * \dot{a}
  Eigen::Vector3d jerk_dot_withoutModel_; // Estimated jerk derivative
  Eigen::Vector3d jerk_withoutModel_noFiltration_; // Estimated jerk without filtering
  Eigen::Vector3d jerk_diff_baseNoModel_; // Difference between the jerk estimated with the robot model and without the model

  // Base estimation
  Eigen::Vector3d bias_gyro_base_; // Gyro bias estimate
  Eigen::Vector3d bias_gyro_dot_base_; // Gyro bias derivative
  Eigen::Matrix3d R_base_; // Estimated rotation matrix
  Eigen::Quaternion<double> quat_tilde_base_;
  Eigen::Quaternion<double> quat_R_base_;
  Eigen::Vector3d omega_base_; // Angular velocity
  Eigen::Vector3d jerk_base_; // Estimated jerk = R^T * \dot{a}
  Eigen::Vector3d jerk_dot_base_; // Estimated jerk derivative

  // Estimation from QP
  Eigen::Vector3d acc_qp_;
  Eigen::Vector3d jerk_qp_;
  Eigen::Vector3d jerk_diff_baseQp_; 
  

  // Estimation including the linear velocity
  Eigen::Vector3d bias_gyro_vel_; // Gyro bias derivative including the linear velocity
  Eigen::Vector3d bias_gyro_dot_vel_; // Gyro bias derivative including the linear velocity
  Eigen::Matrix3d R_vel_; // Estimated rotation matrix including the linear velocity
  Eigen::Quaternion<double> quat_tilde_vel_;
  Eigen::Quaternion<double> quat_R_vel_;
  Eigen::Vector3d omega_acceleroAndEncVel_;
  Eigen::Vector3d omega_vel_; // Angular velocity including the linear velocity
  Eigen::Vector3d jerk_vel_; // Estimated jerk including the linear velocity
  Eigen::Vector3d jerk_dot_vel_; // Estimated jerk derivative including the linear velocity
  Eigen::Vector3d v_vel_;
  Eigen::Vector3d v_dot_vel_;
  Eigen::Vector3d v_dot_accelero_; // Linear acceleration from the accelerometer measurements (Gravity removed)
  Eigen::Vector3d acc_vel_; // Linear acceleration including the linear velocity
  Eigen::Vector3d acc_dot_vel_; // Derivative of the linear acceleration including the linear velocity
  
};

} // namespace mc_plugin
