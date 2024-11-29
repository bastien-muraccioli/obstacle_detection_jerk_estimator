#include "ObstacleDetectionJerkEstimator.h"

#include <mc_control/GlobalPluginMacros.h>
  
#include <Eigen/src/Core/Matrix.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <mc_rtc/gui/plot.h>
// Make shorter names for types we will use a lot
using Color = mc_rtc::gui::Color;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;
using Range = mc_rtc::gui::plot::Range;
using Style = mc_rtc::gui::plot::Style;
using Side = mc_rtc::gui::plot::Side;

namespace mc_plugin
{

ObstacleDetectionJerkEstimator::~ObstacleDetectionJerkEstimator() = default;

void ObstacleDetectionJerkEstimator::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();

  dt_ = ctl.timestep();
  t_ = 0.0;
  internal_counter_ = 0;
  k_ = 1.0;
  alpha_rot_ = 0.5;
  alpha_jerk_ = 0.5;
  imuBodyName_ = "Accelerometer";
  if(config.has("k"))
  {
    k_ = config("k");
  }
  if(config.has("alpha_rot"))
  {
    alpha_rot_ = config("alpha_rot");
  }
    if(config.has("alpha_jerk"))
  {
    alpha_jerk_ = config("alpha_jerk");
  }
  if(config.has("imuBodyName"))
  {
    imuBodyName_.assign(config("imuBodyName"));
  }


  const auto & imu = robot.bodySensor(imuBodyName_);
  accelero_ = Eigen::Vector3d::Zero();
  accelero_dot_ = Eigen::Vector3d::Zero();
  gyro_ = Eigen::Vector3d::Zero();
  bias_gyro_ = Eigen::Vector3d::Zero();
  bias_gyro_dot_ = Eigen::Vector3d::Zero();
  R_dot_ = Eigen::Matrix3d::Zero();
  R_rob_ = Eigen::Matrix3d::Zero();
  jerk_ = Eigen::Vector3d::Zero();
  jerk_no_filtered_ = Eigen::Vector3d::Zero();
  jerk_dot_ = Eigen::Vector3d::Zero();
  omega_ = Eigen::Vector3d::Zero();

  // R_ = 
  R_rob_ = realRobot.bodySensor(imuBodyName_).orientation().toRotationMatrix().transpose();//realRobot.posW().rotation(); //realRobot.bodySensor(imu.parentBody()).orientation().toRotationMatrix().transpose();
  R_ = R_rob_;

  // Add log entries
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::gyro", [this]() { return gyro_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::bias_gyro", [this]() { return bias_gyro_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::bias_gyro_dot", [this]() { return bias_gyro_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk", [this]() { return jerk_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_dot", [this]() { return jerk_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_no_filtered", [this]() { return jerk_no_filtered_; });

  // Add GUI elements
  auto & gui = *ctl.controller().gui();

  gui.addPlot(
      "jerk(t)",
      mc_rtc::gui::plot::X(
          "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
      mc_rtc::gui::plot::Y(
          "jerk(t)", [this]() { return jerk_(0); }, mc_rtc::gui::Color::Red));
  
  gui.addPlot(
      "jerk no filtered(t)",
      mc_rtc::gui::plot::X(
          "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
      mc_rtc::gui::plot::Y(
          "jerk no filtered(t)", [this]() { return jerk_no_filtered_(0); }, mc_rtc::gui::Color::Blue));

  //jerk_dot_
  gui.addPlot(
    "jerk_dot(t)",
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "jerk_dot(t)", [this]() { return jerk_dot_(0); }, mc_rtc::gui::Color::Green));

  mc_rtc::log::info("ObstacleDetectionJerkEstimator::init called with configuration:\n{}", config.dump(true, true));
}

void ObstacleDetectionJerkEstimator::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("ObstacleDetectionJerkEstimator::reset called");
}

void ObstacleDetectionJerkEstimator::before(mc_control::MCGlobalController & controller)
{
  t_ += dt_;
  // mc_rtc::log::info("ObstacleDetectionJerkEstimator::before");
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();
  const auto & imu = robot.bodySensor(imuBodyName_);
  
  // Estimate the rotation matrix based on the gyro measurements
  gyro_ = imu.angularVelocity();

  R_rob_ = realRobot.bodySensor(imuBodyName_).orientation().toRotationMatrix().transpose();
  // R_rob_ = realRobot.posW().rotation();// realRobot.bodySensor(imu.parentBody()).orientation().toRotationMatrix().transpose();
  const Eigen::Matrix3d rot_error = (R_rob_ * R_.transpose() - R_rob_.transpose() * R_)/2;
  const Eigen::Vector3d rot_error_vec = stateObservation::kine::skewSymmetricToRotationVector(rot_error);

  bias_gyro_dot_ = -k_ * rot_error_vec;
  bias_gyro_ += bias_gyro_dot_*dt_;

  const Eigen::Vector3d gyro_filter = gyro_ + bias_gyro_ + alpha_rot_ * rot_error_vec;
  R_dot_ = R_ * stateObservation::kine::skewSymmetric(gyro_filter);
  R_ += R_dot_*dt_;

  // Estimate the jerk based on the accelerometer measurements
  const Eigen::Vector3d omega_prev = omega_;
  omega_ = gyro_ - bias_gyro_;
  const Eigen::Vector3d omega_dot = (omega_ - omega_prev)/dt_;


  const Eigen::Vector3d prev_accelero = accelero_;
  accelero_ = imu.linearAcceleration();
  const Eigen::Vector3d prev_accelero_dot = accelero_dot_;
  accelero_dot_ = (accelero_- prev_accelero)/dt_;
  const Eigen::Vector3d accelero_dot_dot = (accelero_dot_ - prev_accelero_dot)/dt_;

  jerk_no_filtered_ = stateObservation::kine::skewSymmetric(omega_) * accelero_ + accelero_dot_;

  jerk_dot_ = stateObservation::kine::skewSymmetric(omega_dot)*accelero_ + stateObservation::kine::skewSymmetric(omega_)*accelero_dot_ + accelero_dot_dot + alpha_jerk_*(jerk_no_filtered_ - jerk_);
  jerk_ += jerk_dot_*dt_;

  internal_counter_++;
}

void ObstacleDetectionJerkEstimator::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("ObstacleDetectionJerkEstimator::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ObstacleDetectionJerkEstimator::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ObstacleDetectionJerkEstimator", mc_plugin::ObstacleDetectionJerkEstimator)
