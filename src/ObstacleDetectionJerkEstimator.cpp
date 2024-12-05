#include "ObstacleDetectionJerkEstimator.h"

#include <mc_control/GlobalPluginMacros.h>
  
#include <Eigen/src/Core/Matrix.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <mc_control/mc_global_controller.h>
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

  imu_not_yet_initialized_ = true;

  ctl.controller().datastore().make_call("ObstacleDetectionJerkEstimator::ResetPlot", [this]() {
      this->reset_plot_flag_ = true;
    });

  ctl.controller().datastore().make_call("ObstacleDetectionJerkEstimator::RemovePlot", [this](){
      this->remove_plot_flag_ = true;
    });

  ctl.controller().datastore().make<bool>("Obstacle detected", false);
  obstacle_detected_ = false;
  obstacle_detection_has_changed_ = false;

  dt_ = ctl.timestep();
  internal_counter_ = 0;

  // Setup configuration
  obstacle_threshold_ = 10000.0;
  k_ = 1.0;
  alpha_rot_ = 0.5;
  alpha_jerk_ = 0.5;
  imuBodyName_ = "Accelerometer";
  if(config.has("obstacleThreshold"))
  {
    obstacle_threshold_ = config("obstacleThreshold");
  }
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
  
  accelero_dot_ = Eigen::Vector3d::Zero();
  bias_gyro_ = Eigen::Vector3d::Zero();
  bias_gyro_dot_ = Eigen::Vector3d::Zero();
  R_dot_ = Eigen::Matrix3d::Zero();
  // R_rob_ = Eigen::Matrix3d::Zero();
  jerk_ = Eigen::Vector3d::Zero();
  jerk_no_filtered_ = Eigen::Vector3d::Zero();
  jerk_dot_ = Eigen::Vector3d::Zero();
  jerk_biased_imu_ = Eigen::Vector3d::Zero();
  jerk_dot_biased_imu_ = Eigen::Vector3d::Zero();
  omega_ = Eigen::Vector3d::Zero();

  // R_ = 
  R_rob_ = realRobot.bodySensor(imuBodyName_).orientation().toRotationMatrix().transpose();//realRobot.posW().rotation(); //realRobot.bodySensor(imu.parentBody()).orientation().toRotationMatrix().transpose();
  R_ = R_rob_;
  quat_tilde_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_.transpose()));

  // removePlot(ctl);
  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("ObstacleDetectionJerkEstimator::init called with configuration:\n{}", config.dump(true, true));
}

void ObstacleDetectionJerkEstimator::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("ObstacleDetectionJerkEstimator::reset called");
}

void ObstacleDetectionJerkEstimator::before(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("ObstacleDetectionJerkEstimator::before");
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();
  const auto & imu = robot.bodySensor(imuBodyName_);

  if(imu.linearAcceleration().norm() != 0.0 && imu.angularVelocity().norm() != 0.0 && imu_not_yet_initialized_)
  {
    imu_not_yet_initialized_ = false;
    accelero_ = imu.linearAcceleration();
    gyro_ = imu.angularVelocity();
  }

  if(imu_not_yet_initialized_)
  {
    return;
  }
  
  // Estimate the rotation matrix based on the gyro measurements
  const Eigen::Vector3d prev_gyro = gyro_; 
  gyro_ = imu.angularVelocity();

  R_rob_ = realRobot.bodySensor(imuBodyName_).orientation().toRotationMatrix().transpose();
  quat_tilde_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_.transpose()));
  const Eigen::Matrix3d rot_error = (R_rob_ * R_.transpose() - R_rob_.transpose() * R_)/2;
  const Eigen::Vector3d rot_error_vec = stateObservation::kine::skewSymmetricToRotationVector(rot_error);

  bias_gyro_dot_ = -k_ * rot_error_vec;
  bias_gyro_ += bias_gyro_dot_*dt_;

  const Eigen::Vector3d omega_prev = omega_;
  omega_ = gyro_ - bias_gyro_;
  const Eigen::Vector3d omega_dot = (omega_ - omega_prev)/dt_;

  const Eigen::Vector3d gyro_filter = omega_ + alpha_rot_ * rot_error_vec;
  R_dot_ = R_ * stateObservation::kine::skewSymmetric(gyro_filter);
  // const Eigen::Matrix3d gyro_filtered = stateObservation::kine::skewSymmetric(gyro_filter*dt_);
  // const Eigen::Matrix3d gyro_filtered2 = gyro_filtered * gyro_filtered;
  // const double theta = stateObservation::kine::skewSymmetricToRotationVector(gyro_filtered).norm();
  // const double theta2 = theta*theta;

  // const Eigen::Matrix3d expGyro = Eigen::Matrix3d::Identity()
  //                             + (std::sin(theta)/theta) * gyro_filtered 
  //                             + ((1.0 - std::cos(theta))/theta2) * gyro_filtered2;
  // mc_rtc::log::info("theta: {}, gyro_filtered: {}, expGyro: {}", theta, gyro_filtered.norm(), expGyro.norm());
  // R_ = R_ * expGyro;
  R_ = R_ * stateObservation::kine::rotationVectorToRotationMatrix(gyro_filter*dt_);
  // Estimate the jerk based on the accelerometer measurements
  const Eigen::Vector3d prev_accelero = accelero_;
  accelero_ = imu.linearAcceleration();
  const Eigen::Vector3d prev_accelero_dot = accelero_dot_;
  accelero_dot_ = (accelero_- prev_accelero)/dt_;
  const Eigen::Vector3d accelero_dot_dot = (accelero_dot_ - prev_accelero_dot)/dt_;
  const Eigen::Vector3d gyro_dot = (gyro_ - prev_gyro)/dt_;

  jerk_no_filtered_ = stateObservation::kine::skewSymmetric(omega_) * accelero_ + accelero_dot_;

  jerk_dot_ = stateObservation::kine::skewSymmetric(gyro_dot)*accelero_ + stateObservation::kine::skewSymmetric(omega_)*accelero_dot_ + accelero_dot_dot + alpha_jerk_*(jerk_no_filtered_ - jerk_);
  jerk_ += jerk_dot_*dt_;


  // Estimate the jerk with only the IMU no robot model <=> gyro bias correction
  const Eigen::Vector3d jerk_no_filtered_bias_imu = stateObservation::kine::skewSymmetric(gyro_) * accelero_ + accelero_dot_;
  
  jerk_dot_biased_imu_ = stateObservation::kine::skewSymmetric(gyro_dot)*accelero_ + stateObservation::kine::skewSymmetric(gyro_)*accelero_dot_ + accelero_dot_dot + alpha_jerk_*(jerk_no_filtered_ - jerk_);
  jerk_biased_imu_ += jerk_dot_biased_imu_*dt_;

  // Check if an obstacle is detected
  if(jerk_.norm() > obstacle_threshold_)
  {
    obstacle_detected_ = true;
  }
  else
  {
    obstacle_detected_ = false;
  }

  if(obstacle_detected_ != obstacle_detection_has_changed_)
  {
    obstacle_detection_has_changed_ = obstacle_detected_;
    ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
  }

  internal_counter_++;
  if(reset_plot_flag_)
  {
    removePlot(ctl);
    addPlot(ctl);
    reset_plot_flag_ = false;
  }
  if(remove_plot_flag_)
  {
    removePlot(ctl);
    remove_plot_flag_ = false;
  }
}

void ObstacleDetectionJerkEstimator::after(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("ObstacleDetectionJerkEstimator::after");
}

void ObstacleDetectionJerkEstimator::addPlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  gui.addPlot(
      plots_[0],
      mc_rtc::gui::plot::X(
          "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
      mc_rtc::gui::plot::Y(
          "no filtered jerk(t)", [this]() { return jerk_no_filtered_.norm(); }, mc_rtc::gui::Color::Blue),
      mc_rtc::gui::plot::Y(
      "jerk(t)", [this]() { return jerk_.norm(); }, mc_rtc::gui::Color::Red),
       mc_rtc::gui::plot::Y(
      "jerk_biased_imu(t)", [this]() { return jerk_biased_imu_.norm(); }, mc_rtc::gui::Color::Yellow)
      );

  //jerk_dot_
  gui.addPlot(
    plots_[1],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "jerk_dot(t)", [this]() { return jerk_dot_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "jerk_dot_biased_imu(t)", [this]() { return jerk_dot_biased_imu_.norm(); }, mc_rtc::gui::Color::Yellow)
        );

  //omega
  gui.addPlot(
    plots_[2],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
    "gyro(t)", [this]() { return gyro_.norm(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
        "omega(t)", [this]() { return omega_.norm(); }, mc_rtc::gui::Color::Red)
    );

  //accelero
  gui.addPlot(
    plots_[3],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "accelero(t)", [this]() { return accelero_.norm(); }, mc_rtc::gui::Color::Blue)
    );

  //Rotation matrix
  gui.addPlot(
    plots_[4],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "R(t)", [this]() { return R_.norm(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
        "R_rob(t)", [this]() { return R_rob_.norm(); }, mc_rtc::gui::Color::Red)
    );

}

void ObstacleDetectionJerkEstimator::addGui(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  plots_ = {"jerk", "jerk_dot", "omega", "accelero", "R"};

  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
                                     mc_rtc::gui::NumberInput(
                                         "Obstacle Threshold", [this]() { return this->obstacle_threshold_; },
                                         [this](double threshold)
                                         {
                                           obstacle_threshold_ = threshold;
                                         }));

  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
                                     mc_rtc::gui::NumberInput(
                                         "Gyro Bias Gain", [this]() { return this->k_; },
                                         [this](double gain)
                                         {
                                           k_ = gain;
                                         }));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
                                     mc_rtc::gui::NumberInput(
                                         "Complementary Filter Jerk Gain", [this]() { return this->alpha_jerk_; },
                                         [this](double gain)
                                         {
                                           alpha_jerk_ = gain;
                                         }));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
                                     mc_rtc::gui::NumberInput(
                                         "Complementary Filter Rotation Gain", [this]() { return this->alpha_rot_; },
                                         [this](double gain)
                                         {
                                           alpha_rot_ = gain;
                                         }));

}

void ObstacleDetectionJerkEstimator::addLog(mc_control::MCGlobalController & ctl)
{
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::gyro", [this]() { return gyro_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::gyro_norm", [this]() { return gyro_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::biasGyro", [this]() { return bias_gyro_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::biasGyro_norm", [this]() { return bias_gyro_.norm();});
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::biasGyro_dot", [this]() { return bias_gyro_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::biasGyro_dot_norm", [this]() { return bias_gyro_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk", [this]() { return jerk_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_norm", [this]() { return jerk_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_dot", [this]() { return jerk_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_dot_norm", [this]() { return jerk_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_no_filtered", [this]() { return jerk_no_filtered_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_no_filtered_norm", [this]() { return jerk_no_filtered_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::obs_detected", [this]() { return obstacle_detected_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::omega", [this]() { return omega_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::omega_norm", [this]() { return omega_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::accelero", [this]() { return accelero_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::accelero_norm", [this]() { return accelero_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::accelero_dot", [this]() { return accelero_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::accelero_dot_norm", [this]() { return accelero_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::R", [this]() { return R_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::R_dot", [this]() { return R_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::R_rob", [this]() { return R_rob_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_biased_imu", [this]() { return jerk_biased_imu_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_biased_imu_norm", [this]() { return jerk_biased_imu_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_dot_biased_imu", [this]() { return jerk_dot_biased_imu_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::jerk_dot_biased_imu_norm", [this]() { return jerk_dot_biased_imu_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator::quat_tilde", [this]() { return quat_tilde_; });
}

mc_control::GlobalPlugin::GlobalPluginConfiguration ObstacleDetectionJerkEstimator::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void ObstacleDetectionJerkEstimator::removePlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  for(const auto & plot : plots_)
  {
    mc_rtc::log::info("Removing plot {}", plot);
    gui.removePlot(plot);
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ObstacleDetectionJerkEstimator", mc_plugin::ObstacleDetectionJerkEstimator)
