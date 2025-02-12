#include "ObstacleDetectionJerkEstimator.h"

#include <mc_control/GlobalPluginMacros.h>
  
#include <Eigen/src/Core/Matrix.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <string>

#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/plot.h>
#include <SpaceVecAlg/MotionVec.h>
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

  estimationType_ = EstimationType::Base;

  jointNumber_ = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
  
  // Flags
  imu_not_yet_initialized_ = true;
  remove_plot_flag_= false;
  reset_plot_flag_ = false;
  collision_stop_activated_ = true;
  obstacle_detected_= false;
  obstacle_detection_has_changed_ = false;

  jerkEstimationBaseFlag_ = true;
  jerkEstimationWithLinearVelocityFlag_ = true;
  jerkEstimationWithoutModelFlag_ = true;
  jerkEstimationFromQPFlag_ = true;
  jerkEstimationFlag_ = true;

  // Detection observer
  detection_jerk_base_ = false;
  detection_jerk_vel_ = false;
  detection_jerk_withoutModel_ = false;
  detection_jerk_qp_ = false;

  ctl.controller().datastore().make_call("ObstacleDetectionJerkEstimator::ResetPlot", [this]() {
      this->reset_plot_flag_ = true;
    });

  ctl.controller().datastore().make_call("ObstacleDetectionJerkEstimator::RemovePlot", [this](){
      this->remove_plot_flag_ = true;
    });

  ctl.controller().datastore().make<bool>("Obstacle detected", false);

  dt_ = ctl.timestep();
  internal_counter_ = 0;
  obstacle_threshold_ = 10000.0;

  // Gains
  k_ = 1.0;
  k_vel_ = 1.0;
  alpha_rot_ = 1.0;
  alpha_rot_vel_ = 1.0;
  alpha_v_ = 1.0;
  alpha_jerk_ = 1.0;
  alpha_jerk_vel_ = 1.0;
  alpha_acc_ = 1.0;
  
  imuBodyName_ = "Accelerometer";
  robotBodyName_ = "FT_sensor_mounting";

  if(config.has("obstacleThreshold"))
  {
    obstacle_threshold_ = config("obstacleThreshold");
  }
  if(config.has("collision_stop_activated"))
  {
    collision_stop_activated_ = config("collision_stop_activated");
  }
  if(config.has("k"))
  {
    k_ = config("k");
  }
  if(config.has("k_vel"))
  {
    k_vel_ = config("k_vel");
  }
  if(config.has("alpha_rot"))
  {
    alpha_rot_ = config("alpha_rot");
  }
  if(config.has("alpha_rot_vel"))
  {
    alpha_rot_vel_ = config("alpha_rot_vel");
  }
  if(config.has("alpha_v"))
  {
    alpha_v_ = config("alpha_v");
  }
    if(config.has("alpha_jerk"))
  {
    alpha_jerk_ = config("alpha_jerk");
  }
  if(config.has("alpha_jerk_vel"))
  {
    alpha_jerk_vel_ = config("alpha_jerk_vel");
  }
   if(config.has("alpha_acc"))
  {
    alpha_acc_ = config("alpha_acc");
  }
  
  if(config.has("imuBodyName"))
  {
    imuBodyName_.assign(config("imuBodyName"));
  }
  if(config.has("robotBodyName"))
  {
    robotBodyName_.assign(config("robotBodyName"));
  }
  if(config.has("estimationType"))
  {
    setEstimationType(config("estimationType"));
  }


  if(robot.hasBodySensor(imuBodyName_))
  {
    jerkEstimationInit(ctl);
    isIMU_ = true;
  }
  else {
    isIMU_ = false;
    mc_rtc::log::error("The IMU sensor {} does not exist in the robot", imuBodyName_);
  }

  // Zurlo estimation
  windowSize = 100;
  if(config.has("windowSize"))
  {
    windowSize = config("windowSize");
  }
  sensitivityThresholdNiblack = 8.0;
  if(config.has("sensitivityThresholdNiblack"))
  {
    sensitivityThresholdNiblack = config("sensitivityThresholdNiblack");
  }
  sensitivityThresholdCusum = 0.5;
  if(config.has("sensitivityThresholdCusum"))
  {
    sensitivityThresholdCusum = config("sensitivityThresholdCusum");
  }
  zurlo_.setValues(windowSize, jointNumber_, sensitivityThresholdNiblack, sensitivityThresholdCusum);

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
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  Eigen::VectorXd qdot(jointNumber_);
  rbd::paramToVector(realRobot.alpha(), qdot);

  obstacle_detected_ = false;
  if(isIMU_ && jerkEstimationFlag_)
  {
    jerkEstimation(ctl);
  }
  if(zurlo_.zurloEstimationFlag_&& qdot.any() > 0.1)
  {
    obstacle_detected_ = zurlo_.collisionDetection(ctl);
  }
  
  if(obstacle_detected_ != obstacle_detection_has_changed_)
  {
    obstacle_detection_has_changed_ = obstacle_detected_;
    ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
  }
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

  internal_counter_++;
  zurlo_.updateCounter(internal_counter_ * dt_);
}

void ObstacleDetectionJerkEstimator::after(mc_control::MCGlobalController & controller)
{
  
}

void ObstacleDetectionJerkEstimator::addPlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  double counter = static_cast<double>(internal_counter_) * dt_;
  gui.addPlot(
      plots_[0],
      mc_rtc::gui::plot::X(
          "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
      mc_rtc::gui::plot::Y(
          "no filtered jerk(t)", [this]() { return jerk_withoutModel_noFiltration_.norm(); }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
      "jerk(t)", [this]() { return jerk_base_.norm(); }, mc_rtc::gui::Color::Blue),
       mc_rtc::gui::plot::Y(
      "jerk_biased_imu(t)", [this]() { return jerk_withoutModel_.norm(); }, mc_rtc::gui::Color::Yellow),
      mc_rtc::gui::plot::Y(
        "jerk_vel(t)", [this]() { return jerk_vel_.norm(); }, mc_rtc::gui::Color::Green),
      mc_rtc::gui::plot::Y(
        "jerk_qp(t)", [this]() { return jerk_qp_.norm(); }, mc_rtc::gui::Color::Magenta)
      );

  //jerk_dot_
  gui.addPlot(
    plots_[1],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "jerk_dot(t)", [this]() { return jerk_dot_base_.norm(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
        "jerk_dot_biased_imu(t)", [this]() { return jerk_dot_withoutModel_.norm(); }, mc_rtc::gui::Color::Yellow)
        );

  //omega
  gui.addPlot(
    plots_[2],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
    "gyro(t)", [this]() { return gyro_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "omega(t)", [this]() { return omega_base_.norm(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
        "omega_vel(t)", [this]() { return omega_vel_.norm(); }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y(
        "omega_acceleroAndEncVel(t)", [this]() { return omega_acceleroAndEncVel_.norm(); }, mc_rtc::gui::Color::Cyan)
    );

  //accelero
  gui.addPlot(
    plots_[3],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "accelero(t)", [this]() { return accelero_.norm(); }, mc_rtc::gui::Color::Red)
    );

  //Rotation matrix
  gui.addPlot(
    plots_[4],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "R(t)", [this]() { return R_base_.norm(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y(
        "R_rob(t)", [this]() { return R_rob_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "R_vel(t)", [this]() { return R_vel_.norm(); }, mc_rtc::gui::Color::Green)
    );

  //Velocity
  gui.addPlot(
    plots_[5],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "v_rob(t)", [this]() { return v_encoders_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "v(t)", [this]() { return v_vel_.norm(); }, mc_rtc::gui::Color::Green)
    );

  //Acceleration
  gui.addPlot(
    plots_[6],
    mc_rtc::gui::plot::X(
        "t", [this]() { return static_cast<double>(internal_counter_) * dt_; }),
    mc_rtc::gui::plot::Y(
        "v_dot", [this]() { return v_dot_vel_.norm(); }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y(
        "v_dot_accelero(t)", [this]() { return v_dot_accelero_.norm(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y(
        "acc_vel(t)", [this]() { return acc_vel_.norm(); }, mc_rtc::gui::Color::Cyan),
    mc_rtc::gui::plot::Y(
        "acc_qp(t)", [this]() { return acc_qp_.norm(); }, mc_rtc::gui::Color::Magenta)
    );

  zurlo_.addPlot(plots_, counter, ctl);
}

void ObstacleDetectionJerkEstimator::addGui(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  plots_ = {"jerk", "jerk_dot", "omega", "accelero", "R", "velocity", "acceleration", "residual", "residual_current", "residual_energy"};

  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::NumberInput(
        "Obstacle Threshold", [this]() { return this->obstacle_threshold_; },
        [this](double threshold)
        {
          obstacle_threshold_ = threshold;
        }));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Collision Stop Activated", [this]() { return collision_stop_activated_; }, [this](){collision_stop_activated_ = !collision_stop_activated_;}
        ));
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
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::NumberInput(
        "Complementary Filter Acceleration Gain", [this]() { return this->alpha_acc_; },
        [this](double gain)
        {
          alpha_acc_ = gain;
        }));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::NumberInput(
        "Complementary Filter Rotation Velocity Gain", [this]() { return this->alpha_rot_vel_; },
        [this](double gain)
        {
          alpha_rot_vel_ = gain;
        })
    );
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Jerk Estimation for collision detection", [this]() { return jerkEstimationFlag_; }, [this](){jerkEstimationFlag_ = !jerkEstimationFlag_;}));

  zurlo_.addGui(ctl);

  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Jerk Estimation Base", [this]() { return jerkEstimationBaseFlag_;},[this](){jerkEstimationBaseFlag_ = !jerkEstimationBaseFlag_;}));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Jerk Estimation With Linear Velocity", [this]() { return jerkEstimationWithLinearVelocityFlag_; }, [this](){jerkEstimationWithLinearVelocityFlag_ = !jerkEstimationWithLinearVelocityFlag_;}));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Jerk Estimation Without Model", [this]() { return jerkEstimationWithoutModelFlag_; }, [this](){jerkEstimationWithoutModelFlag_ = !jerkEstimationWithoutModelFlag_;}));
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::Checkbox(
        "Jerk Estimation From QP", [this]() { return jerkEstimationFromQPFlag_; }, [this](){jerkEstimationFromQPFlag_ = !jerkEstimationFromQPFlag_;})
    );
  gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
    mc_rtc::gui::ComboInput(
      "Estimation type for collision detection (Works only if you checked Jerk Estimation)", {"Base", "With Linear Velocity", "Without Model", "From QP", "Zurlo"},
      [this]() {return getEstimationType();},
      [this](const std::string & t){setEstimationType(t);})
    );  
}

void ObstacleDetectionJerkEstimator::addLog(mc_control::MCGlobalController & ctl)
{
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero", [this]() { return accelero_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero_norm", [this]() { return accelero_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero_dot", [this]() { return accelero_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero_dot_norm", [this]() { return accelero_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero_dot_dot", [this]() { return accelero_dot_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_accelero_dot_dot_norm", [this]() { return accelero_dot_dot_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_gyro", [this]() { return gyro_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_gyro_norm", [this]() { return gyro_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_gyro_dot", [this]() { return gyro_dot_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_gyro_dot_norm", [this]() { return gyro_dot_.norm(); });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_robot", [this]() { return R_rob_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_robot_quaternion", [this]() { return quat_R_rob_; });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_encoders", [this]() { return v_encoders_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_encoders_norm", [this]() { return v_encoders_.norm(); });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel", [this]() { return jerk_withoutModel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel_norm", [this]() { return jerk_withoutModel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel_dot", [this]() { return jerk_dot_withoutModel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel_dot_norm", [this]() { return jerk_dot_withoutModel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel_noFiltration", [this]() { return jerk_withoutModel_noFiltration_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_withoutModel_noFiltration_norm", [this]() { return jerk_withoutModel_noFiltration_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_diff_baseNoModel", [this]() { return jerk_diff_baseNoModel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_diff_baseNoModel_norm", [this]() { return jerk_diff_baseNoModel_.norm(); });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_base", [this]() { return bias_gyro_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_base_norm", [this]() { return bias_gyro_base_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_base_dot", [this]() { return bias_gyro_dot_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_base_dot_norm", [this]() { return bias_gyro_dot_base_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_base", [this]() { return R_base_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_base_quaternion_error", [this]() { return quat_tilde_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_base_quaternion", [this]() { return quat_R_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_omega_base", [this]() { return omega_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_omega_base_norm", [this]() { return omega_base_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_base", [this]() { return jerk_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_base_norm", [this]() { return jerk_base_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_base_dot", [this]() { return jerk_dot_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_base_dot_norm", [this]() { return jerk_dot_base_.norm(); });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_qp", [this]() { return acc_qp_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_qp_norm", [this]() { return acc_qp_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_qp", [this]() { return jerk_qp_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_qp_norm", [this]() { return jerk_qp_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_diff_baseQp", [this]() { return jerk_diff_baseQp_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_diff_baseQp_norm", [this]() { return jerk_diff_baseQp_.norm(); });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_vel", [this]() { return bias_gyro_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_vel_norm", [this]() { return bias_gyro_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_vel_dot", [this]() { return bias_gyro_dot_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_biasGyro_vel_dot_norm", [this]() { return bias_gyro_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_vel", [this]() { return R_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_vel_quaternion_error", [this]() { return quat_tilde_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_R_vel_quaternion", [this]() { return quat_R_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_omega_vel", [this]() { return omega_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_omega_vel_norm", [this]() { return omega_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_vel", [this]() { return jerk_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_vel_norm", [this]() { return jerk_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_vel_dot", [this]() { return jerk_dot_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_jerk_vel_dot_norm", [this]() { return jerk_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_vel", [this]() { return v_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_vel_norm", [this]() { return v_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_vel_dot", [this]() { return v_dot_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_vel_dot_norm", [this]() { return v_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_dot_accelero", [this]() { return v_dot_accelero_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_v_dot_accelero_norm", [this]() { return v_dot_accelero_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_vel", [this]() { return acc_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_vel_norm", [this]() { return acc_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_vel_dot", [this]() { return acc_dot_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_acc_vel_dot_norm", [this]() { return acc_dot_vel_.norm(); });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_obstacleDetected_", [this]() { return obstacle_detected_; });

  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_jerk_base_", [this]() { return detection_jerk_base_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_jerk_vel_", [this]() { return detection_jerk_vel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_jerk_withoutModel_", [this]() { return detection_jerk_withoutModel_; });
  ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_jerk_qp_", [this]() { return detection_jerk_qp_; });

  zurlo_.addLog(ctl);
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

void ObstacleDetectionJerkEstimator::jerkEstimationInit(mc_control::MCGlobalController & ctl)
{
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();

  const auto & imu = robot.bodySensor(imuBodyName_);
  rbd::forwardKinematics(realRobot.mb(), realRobot.mbc());
  rbd::forwardVelocity(realRobot.mb(), realRobot.mbc());
  rbd::forwardAcceleration(realRobot.mb(), realRobot.mbc());

  // Initializing variables
  accelero_dot_ = Eigen::Vector3d::Zero();
  accelero_dot_dot_ = Eigen::Vector3d::Zero();
  prev_accelero_dot_ = Eigen::Vector3d::Zero();
  gyro_dot_ = Eigen::Vector3d::Zero();

  R_rob_ = realRobot.bodyPosW(robotBodyName_).rotation().transpose();
  quat_R_rob_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_));

  v_encoders_ = realRobot.bodyVelW(robotBodyName_).linear();

  // Initialize the estimation without the model
  jerk_withoutModel_ = Eigen::Vector3d::Zero();
  jerk_dot_withoutModel_ = Eigen::Vector3d::Zero();
  jerk_withoutModel_noFiltration_ = Eigen::Vector3d::Zero();
  jerk_diff_baseNoModel_ = Eigen::Vector3d::Zero();

  // Initialize the base estimation
  bias_gyro_base_ = Eigen::Vector3d::Zero();
  bias_gyro_dot_base_ = Eigen::Vector3d::Zero();
  R_base_ = R_rob_;
  quat_tilde_base_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_base_.transpose()));
  quat_R_base_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_base_));
  omega_base_ = Eigen::Vector3d::Zero();
  jerk_base_ = Eigen::Vector3d::Zero();
  jerk_dot_base_ = Eigen::Vector3d::Zero();

  // Initialize the estimation from the QP
  acc_qp_ = Eigen::Vector3d::Zero();
  jerk_qp_ = Eigen::Vector3d::Zero();
  jerk_diff_baseQp_ = Eigen::Vector3d::Zero();

  // Initialize the estimation including the linear velocity
  bias_gyro_vel_ = Eigen::Vector3d::Zero();
  bias_gyro_dot_vel_ = Eigen::Vector3d::Zero();
  R_vel_ = R_rob_;
  quat_tilde_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_vel_.transpose()));
  quat_R_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_vel_));
  omega_acceleroAndEncVel_ = Eigen::Vector3d::Zero();
  omega_vel_ = Eigen::Vector3d::Zero();
  jerk_vel_ = Eigen::Vector3d::Zero();
  jerk_dot_vel_ = Eigen::Vector3d::Zero();
  v_vel_ = Eigen::Vector3d::Zero();
  v_dot_vel_ = Eigen::Vector3d::Zero();
  v_dot_accelero_ = Eigen::Vector3d::Zero();
  acc_vel_ = Eigen::Vector3d::Zero();
  acc_dot_vel_ = Eigen::Vector3d::Zero();
}

void ObstacleDetectionJerkEstimator::jerkEstimation(mc_control::MCGlobalController & ctl)
{
  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot();
  if(!robot.hasBodySensor(imuBodyName_))
  {
    mc_rtc::log::error("[ObstacleDetectionJerkEstimator] Body sensor {} does not exist in the robot. Jerk Estimation is impossible", imuBodyName_);
    return;
  }
  const mc_rbdyn::BodySensor & imu = robot.bodySensor(imuBodyName_);

  rbd::forwardKinematics(realRobot.mb(), realRobot.mbc());
  rbd::forwardVelocity(realRobot.mb(), realRobot.mbc());
  rbd::forwardAcceleration(realRobot.mb(), realRobot.mbc());

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

  detection_jerk_base_ = false;
  detection_jerk_vel_ = false;
  detection_jerk_withoutModel_ = false;
  detection_jerk_qp_ = false;
  
  // IMU measurements
  prev_gyro_ = gyro_; 
  gyro_ = imu.angularVelocity();
  const Eigen::Vector3d gyro_dot = (gyro_ - prev_gyro_)/dt_;
  prev_accelero_ = accelero_;
  accelero_ = imu.linearAcceleration();
  prev_accelero_dot_ = accelero_dot_;
  accelero_dot_ = (accelero_- prev_accelero_)/dt_;
  accelero_dot_dot_ = (accelero_dot_ - prev_accelero_dot_)/dt_;


  // Rotation matrix model
  R_rob_ = realRobot.bodyPosW(robotBodyName_).rotation().transpose();
  quat_R_rob_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_));

  // Velocity from the robot model
  v_encoders_ = realRobot.bodyVelW(robotBodyName_).linear();

  if(jerkEstimationBaseFlag_)
  {
    jerkEstimationBase(ctl);
  }
  if(jerkEstimationWithLinearVelocityFlag_)
  {
    jerkEstimationWithLinearVelocity(ctl);
  }
  if(jerkEstimationWithoutModelFlag_)
  {
    jerkEstimationWithoutModel(ctl);
  }
  if(jerkEstimationFromQPFlag_)
  {
    jerkEstimationFromQP(ctl);
  }

  double jerk_norm = 0.0;

  switch (estimationType_) {
    case EstimationType::Base:
      jerk_norm = jerk_base_.norm();
      break;
    case EstimationType::WithLinearVelocity:
      jerk_norm = jerk_vel_.norm();
      break;
    case EstimationType::WithoutModel:
      jerk_norm = jerk_withoutModel_.norm();
      break;
    case EstimationType::FromQP:
      jerk_norm = jerk_qp_.norm();
      break;
  }

  // Check if an obstacle is detected
  if(jerk_norm > obstacle_threshold_ && collision_stop_activated_)
  {
      obstacle_detected_ = true;
      mc_rtc::log::info("Obstacle detected with jerk estimation");
  }
}

void ObstacleDetectionJerkEstimator::jerkEstimationBase(mc_control::MCGlobalController & ctl)
{
  // Estimate the gyro bias
  const Eigen::Matrix3d rot_error = (R_rob_ * R_base_.transpose() - R_rob_.transpose() * R_base_)/2;
  const Eigen::Vector3d rot_error_vec = stateObservation::kine::skewSymmetricToRotationVector(rot_error);
  bias_gyro_dot_base_ = -k_ * rot_error_vec;
  bias_gyro_base_ += bias_gyro_dot_base_*dt_;

  // Estimate the angular velocity: gyro with bias correction
  const Eigen::Vector3d omega_prev = omega_base_;
  omega_base_ = gyro_ - bias_gyro_base_;

  // Estimate the rotation matrix
  const Eigen::Vector3d filtered_omega = omega_base_ + alpha_rot_ * rot_error_vec;
  // R_dot_ = R_ * stateObservation::kine::skewSymmetric(filtered_omega); <=>
  R_base_ = R_base_ * stateObservation::kine::rotationVectorToRotationMatrix(filtered_omega*dt_);
  
  // Estimate the jerk
  const Eigen::Vector3d X = stateObservation::kine::skewSymmetric(omega_base_) * accelero_ + accelero_dot_;
  const Eigen::Vector3d X_dot = stateObservation::kine::skewSymmetric(gyro_dot_)*accelero_ + stateObservation::kine::skewSymmetric(omega_base_)*accelero_dot_ + accelero_dot_dot_;
  jerk_dot_base_ = X_dot + alpha_jerk_*(X - jerk_base_);
  jerk_base_ += jerk_dot_base_*dt_;

  // Need to be equal to identity matrix to make sure the rotation matrix is correctly estimated
  quat_tilde_base_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_base_.transpose()));
  // Quaternion of the rotation matrix for logs
  quat_R_base_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_base_));

  if(jerk_base_.norm() > obstacle_threshold_)
  {
    detection_jerk_base_ = true;
  }
}

void ObstacleDetectionJerkEstimator::jerkEstimationWithLinearVelocity(mc_control::MCGlobalController & ctl)
{
  // Estimate the gyro bias
  const Eigen::Matrix3d rot_error = (R_rob_ * R_vel_.transpose() - R_rob_.transpose() * R_vel_)/2;
  const Eigen::Vector3d rot_error_vec = stateObservation::kine::skewSymmetricToRotationVector(rot_error);
  bias_gyro_dot_vel_ = -k_vel_ * rot_error_vec;
  bias_gyro_vel_ += bias_gyro_dot_vel_*dt_;

  // Estimate the angular velocity: gyro with bias correction
  const Eigen::Vector3d omega_prev_vel = omega_vel_;
  omega_vel_ = gyro_ - bias_gyro_vel_;
  const Eigen::Vector3d omega_dot_vel = (omega_vel_ - omega_prev_vel)/dt_;

  // Estimate the rotation matrix
  omega_acceleroAndEncVel_ = acc_vel_.cross(R_vel_.transpose()*(v_encoders_-v_vel_)); //accelero_.cross(R_vel_.transpose()*(v_encoders_-v_vel_));
  const Eigen::Vector3d filtered_omega_vel = omega_vel_ + alpha_rot_vel_ * rot_error_vec + alpha_v_*omega_acceleroAndEncVel_;
  // R_dot_vel_ = R_vel_ * stateObservation::kine::skewSymmetric(filtered_omega_vel);
  R_vel_ = R_vel_ * stateObservation::kine::rotationVectorToRotationMatrix(filtered_omega_vel*dt_);

  // Estimate the acceleration from the velocity
  v_dot_accelero_ = Eigen::Vector3d(0,0,-9.81)+R_vel_*accelero_;
  v_dot_vel_= v_dot_accelero_ + alpha_acc_*(v_encoders_ - v_vel_);
  v_vel_ += v_dot_vel_*dt_;

  // Estimate the jerk
  Eigen::Vector3d X = stateObservation::kine::skewSymmetric(omega_vel_) * accelero_ + accelero_dot_;
  acc_dot_vel_ = R_vel_*X + alpha_jerk_vel_*(v_dot_vel_ - acc_vel_);
  acc_vel_ += acc_dot_vel_*dt_;
  jerk_vel_ = R_vel_.transpose()*acc_dot_vel_;

  // Need to be equal to identity matrix to make sure the rotation matrix is correctly estimated
  quat_tilde_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_rob_*R_vel_.transpose()));
  // Quaternion of the rotation matrix for logs
  quat_R_vel_ = stateObservation::kine::rotationVectorToQuaternion(stateObservation::kine::rotationMatrixToRotationVector(R_vel_));

  if(jerk_vel_.norm() > obstacle_threshold_)
  {
    detection_jerk_vel_ = true;
  }
}

void ObstacleDetectionJerkEstimator::jerkEstimationWithoutModel(mc_control::MCGlobalController & ctl)
{
  jerk_withoutModel_noFiltration_ = stateObservation::kine::skewSymmetric(gyro_) * accelero_ + accelero_dot_; // X
  const Eigen::Vector3d X_dot = stateObservation::kine::skewSymmetric(gyro_dot_)*accelero_ + stateObservation::kine::skewSymmetric(gyro_)*accelero_dot_ + accelero_dot_dot_;

  jerk_dot_withoutModel_ = X_dot + alpha_jerk_*(jerk_withoutModel_noFiltration_ - jerk_withoutModel_);
  jerk_withoutModel_ += jerk_dot_withoutModel_*dt_;

  jerk_diff_baseNoModel_ = jerk_base_ - jerk_withoutModel_;

  if(jerk_withoutModel_.norm() > obstacle_threshold_)
  {
    detection_jerk_withoutModel_ = true;
  }
}

void ObstacleDetectionJerkEstimator::jerkEstimationFromQP(mc_control::MCGlobalController & ctl)
{
  auto & realRobot = ctl.realRobot();
  Eigen::Vector3d acc_qp_new =
      R_rob_ * realRobot.bodyAccB(robotBodyName_).linear() +
      realRobot.bodyVelW(robotBodyName_).angular().cross(v_encoders_);
  Eigen::Vector3d acc_qp_dot = (acc_qp_new - acc_qp_)/dt_;
  acc_qp_ = acc_qp_new;

  jerk_qp_ = R_rob_.transpose()*acc_qp_dot;
  jerk_diff_baseQp_ = jerk_base_ - jerk_qp_;

  if(jerk_qp_.norm() > obstacle_threshold_)
  {
    detection_jerk_qp_ = true;
  }
}

void ObstacleDetectionJerkEstimator::setEstimationType(std::string t)
{
  if(t.compare("Base") == 0)
  {
    estimationType_ = EstimationType::Base;
  }
  else if(t.compare("With Linear Velocity") == 0)
  {
    estimationType_ = EstimationType::WithLinearVelocity;
  }
  else if(t.compare("Without Model") == 0)
  {
    estimationType_ = EstimationType::WithoutModel;
  }
  else if(t.compare("From QP") == 0)
  {
    estimationType_ = EstimationType::FromQP;
  }
  else
  {
    mc_rtc::log::error("[ObstacleDetectionJerkEstimator] The estimation type is not recognized");
  }
}

std::string ObstacleDetectionJerkEstimator::getEstimationType()
{
  switch (estimationType_) 
  {
    case EstimationType::Base:
      return "Base";
    case EstimationType::WithLinearVelocity:
      return "With Linear Velocity";
    case EstimationType::WithoutModel:
      return "Without Model";
    case EstimationType::FromQP:
      return "From QP";
    default:
      return "Base";
  }
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("ObstacleDetectionJerkEstimator", mc_plugin::ObstacleDetectionJerkEstimator)
