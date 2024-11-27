#include "obstacle_detection_jerk_estimator.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

obstacle_detection_jerk_estimator::~obstacle_detection_jerk_estimator() = default;

void obstacle_detection_jerk_estimator::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("obstacle_detection_jerk_estimator::init called with configuration:\n{}", config.dump(true, true));
}

void obstacle_detection_jerk_estimator::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("obstacle_detection_jerk_estimator::reset called");
}

void obstacle_detection_jerk_estimator::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("obstacle_detection_jerk_estimator::before");
}

void obstacle_detection_jerk_estimator::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("obstacle_detection_jerk_estimator::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration obstacle_detection_jerk_estimator::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("obstacle_detection_jerk_estimator", mc_plugin::obstacle_detection_jerk_estimator)
