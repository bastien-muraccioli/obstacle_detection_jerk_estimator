cmake_minimum_required(VERSION 3.1)

project(
  ObstacleDetectionJerkEstimator
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/ObstacleDetectionJerkEstimator.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ObstacleDetectionJerkEstimator.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ObstacleDetectionJerkEstimator.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC src/ObstacleDetectionJerkEstimator.cpp)

set(plugin_HDR src/ObstacleDetectionJerkEstimator.h)

# The add_plugin macro is provided by mc_rtc
add_plugin(ObstacleDetectionJerkEstimator ${plugin_SRC} ${plugin_HDR})

link_libraries(ObstacleDetectionJerkEstimator state-observation::state-observation)
# target_link_libraries(ObstacleDetectionJerkEstimator PUBLIC state-observation::state-observation)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
