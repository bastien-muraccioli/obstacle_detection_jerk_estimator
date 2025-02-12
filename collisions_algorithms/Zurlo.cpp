#include "Zurlo.h"

Zurlo::Zurlo(void) {}

void Zurlo::setValues(int n, double k, int jointNumber) 
{
    windowSize = n;
    sensitivityThreshold = k;
    jointNumber_ = jointNumber;
    base_high_threshold = 0.0;
    base_low_threshold = 0.0;
    residual_high_threshold.setConstant(jointNumber, base_high_threshold);
    residual_low_threshold.setConstant(jointNumber, base_low_threshold);
    residual_current_high_threshold.setConstant(jointNumber, base_high_threshold);
    residual_current_low_threshold.setConstant(jointNumber, base_low_threshold);
    residual_energy_high_threshold = base_high_threshold;
    residual_energy_low_threshold = base_low_threshold;
    zurloEstimationFlag_ = false;
    zurloUse_residual_ = false;
    zurloUse_residual_current_ = true;
    residual_.setZero(jointNumber);
    residual_current_.setZero(jointNumber);
    residual_energy_ = 0.0;
    zurloEstimationControlFlag_ = false;
    zurloNiblackThreshold_residual_.setValues(windowSize, sensitivityThreshold, jointNumber);
    zurloNiblackThreshold_residual_current_.setValues(windowSize, sensitivityThreshold, jointNumber);
    zurloNiblackThreshold_residual_energy_.setValues(windowSize, sensitivityThreshold, 1);
    detection_zurlo_current_ = false;
    detection_zurlo_torque_ = false;
}

bool Zurlo::collisionDetection(mc_control::MCGlobalController & ctl)
{
    detection_zurlo_current_ = false;
    detection_zurlo_torque_ = false;
    if(ctl.controller().datastore().has("speed_residual"))
    {
        residual_ = ctl.controller().datastore().get<Eigen::VectorXd>("speed_residual");
    }
    else
    {
        mc_rtc::log::error("[ObstacleDetectionJerkEstimator] The speed_residual is not available in the datastore");
    }
    if(ctl.controller().datastore().has("current_residual"))
    {
        residual_current_ = ctl.controller().datastore().get<Eigen::VectorXd>("current_residual");
    }
    else
    {
        mc_rtc::log::error("[ObstacleDetectionJerkEstimator] The current_residual is not available in the datastore");
    }
    if(ctl.controller().datastore().has("energy_residual"))
    {
        residual_energy_ = ctl.controller().datastore().get<double>("energy_residual");
    }
    else
    {
        mc_rtc::log::error("[ObstacleDetectionJerkEstimator] The energy_residual is not available in the datastore");
    }

    if((residual_energy_ > residual_energy_high_threshold) || (residual_energy_ < residual_energy_low_threshold))
        {
            for(int i = 0; i < jointNumber_; i++)
            {
            if(zurloUse_residual_current_)
            {
                if((residual_current_(i) > residual_current_high_threshold(i)) || (residual_current_(i) < residual_current_low_threshold(i)))
                {
                detection_zurlo_current_ = true;
                if(zurloEstimationControlFlag_)
                {
                    mc_rtc::log::info("Obstacle detected with Zurlo estimation");
                    return true;
                }
                }
            }
            if(zurloUse_residual_)
            {
                if((residual_(i) > residual_high_threshold(i)) || (residual_(i) < residual_low_threshold(i)))
                {
                detection_zurlo_torque_ = true;
                if(zurloEstimationControlFlag_)
                {
                    mc_rtc::log::info("Obstacle detected with Zurlo estimation");
                    return true;
                }
                }
            }
            }
        }
    

    // Update the thresholds
    residual_high_threshold = zurloNiblackThreshold_residual_.adaptiveThreshold(residual_high_threshold, residual_, true);
    residual_current_high_threshold = zurloNiblackThreshold_residual_current_.adaptiveThreshold(residual_current_high_threshold, residual_current_, true);
    residual_energy_high_threshold = zurloNiblackThreshold_residual_energy_.adaptiveThreshold(residual_energy_high_threshold, residual_energy_, true);
    residual_low_threshold = zurloNiblackThreshold_residual_.adaptiveThreshold(residual_low_threshold, residual_, false);
    residual_current_low_threshold = zurloNiblackThreshold_residual_current_.adaptiveThreshold(residual_current_low_threshold, residual_current_, false);
    residual_energy_low_threshold = zurloNiblackThreshold_residual_energy_.adaptiveThreshold(residual_energy_low_threshold, residual_energy_, false);
    return false;
}

void Zurlo::addPlot(std::vector<std::string> & plots, double counter,  mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    counter_ = counter;
    //Residual
    gui.addPlot(
        plots[7],
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "residual(t)", [this]() { return residual_[0]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "residual_high_threshold(t)", [this]() { return residual_high_threshold[0]; }, mc_rtc::gui::Color::Green),
        mc_rtc::gui::plot::Y(
            "residual_low_threshold(t)", [this]() { return residual_low_threshold[0]; }, mc_rtc::gui::Color::Blue)
        );

    //Residual current
    gui.addPlot(
        plots[8],
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "residual_current(t)", [this]() { return residual_current_[0]; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "residual_current_high_threshold(t)", [this]() { return residual_current_high_threshold[0]; }, mc_rtc::gui::Color::Green),
        mc_rtc::gui::plot::Y(
            "residual_current_low_threshold(t)", [this]() { return residual_current_low_threshold[0]; }, mc_rtc::gui::Color::Blue)
        );

    //Residual energy
    gui.addPlot(
        plots[9],
        mc_rtc::gui::plot::X(
            "t", [this]() { return counter_; }),
        mc_rtc::gui::plot::Y(
            "residual_energy(t)", [this]() { return residual_energy_; }, mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "residual_energy_high_threshold(t)", [this]() { return residual_energy_high_threshold; }, mc_rtc::gui::Color::Green),
        mc_rtc::gui::plot::Y(
            "residual_energy_low_threshold(t)", [this]() { return residual_energy_low_threshold; }, mc_rtc::gui::Color::Blue)
        );  
}

void Zurlo::addGui(mc_control::MCGlobalController & ctl)
{
    auto & gui = *ctl.controller().gui();
    gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
        mc_rtc::gui::Checkbox(
            "Zurlo Estimation for collision detection", [this]() { return zurloEstimationFlag_; }, [this](){zurloEstimationFlag_ = !zurloEstimationFlag_;}));
      
      gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
        mc_rtc::gui::Checkbox(
            "Zurlo Estimation stop the system when a collision is detected", [this]() { return zurloEstimationControlFlag_; }, [this](){zurloEstimationControlFlag_ = !zurloEstimationControlFlag_;}));
      
      gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
        mc_rtc::gui::Checkbox(
            "Zurlo Estimation use residual", [this]() { return zurloUse_residual_; }, [this](){zurloUse_residual_ = !zurloUse_residual_;}));
      
      gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
        mc_rtc::gui::Checkbox(
            "Zurlo Estimation use residual current", [this]() { return zurloUse_residual_current_; }, [this](){zurloUse_residual_current_ = !zurloUse_residual_current_;}));
      
      gui.addElement({"Plugins", "ObstacleDetectionJerkEstimator"},
        mc_rtc::gui::NumberInput(
            "Zurlo Obstacle Threshold Sensitivity", [this]() { return this->sensitivityThreshold; },
            [this](double threshold)
            {
              sensitivityThreshold = threshold;
            }));
}

void Zurlo::addLog(mc_control::MCGlobalController & ctl)
{
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_high_threshold", [this]() { return residual_high_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_current_high_threshold", [this]() { return residual_current_high_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_energy_high_threshold", [this]() { return residual_energy_high_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_low_threshold", [this]() { return residual_low_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_current_low_threshold", [this]() { return residual_current_low_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_residual_energy_low_threshold", [this]() { return residual_energy_low_threshold; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_zurlo_current_", [this]() { return detection_zurlo_current_; });
    ctl.controller().logger().addLogEntry("ObstacleDetectionJerkEstimator_detection_zurlo_torque_", [this]() { return detection_zurlo_torque_; });
}

void Zurlo::updateCounter(double counter)
{
    counter_ = counter;
}