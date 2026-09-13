cmake_minimum_required(VERSION 3.16)
project(fly_brain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)

include_directories(include)

# Header-only bio-inspired algorithm library (mixer, complementary filter,
# haltere reflex law, Reichardt EMD + LPTC pooling, baseline cascaded-PID
# math, descending fusion law) - pure math, no rclcpp/controller_interface
# dependency, so it's usable both by this package's own pluginlib controllers
# (HaltereReflexController - M2; OpticFlowController, DescendingFusionController
# - M3/M4) and by fly_controller's BaselinePidController (M1), which depends on
# this package for fly_brain/mixer.hpp + fly_brain/cascaded_pid.hpp.
install(
  DIRECTORY include/
  DESTINATION include
)

# M2: HaltereReflexController - the first real controller plugin in this
# package (controller_interface::ChainableControllerInterface, not the plain
# ControllerInterface that fly_controller's M1 BaselinePidController uses -
# see NOTES.md's "## M2" section for why, and for the exact idiom this follows
# from fly_controller's M1 pluginlib-export setup). M3 added
# OpticFlowController to the same library target/plugin XML below; M4 adds
# DescendingFusionController the same way.
add_library(fly_brain SHARED
  src/haltere_reflex_controller.cpp
  src/optic_flow_controller.cpp
  src/descending_fusion_controller.cpp
  src/phototaxis_controller.cpp
)
# PUBLIC (not PRIVATE): once this target is ament_export_targets()-ed below,
# find_package(fly_brain) exposes it as a modern CMake interface target, and
# ament_target_dependencies() in *consuming* packages (fly_controller) uses a
# found package's target(s) EXCLUSIVELY when present - it stops looking at the
# classic fly_brain_INCLUDE_DIRS variable entirely (confirmed by hitting this:
# a PRIVATE include here built fine standalone but broke fly_controller's
# `#include "fly_brain/cascaded_pid.hpp"` with a plain file-not-found, because
# fly_brain_INCLUDE_DIRS was populated but silently ignored - see
# ament_cmake_target_dependencies.cmake's "if a package provides modern CMake
# interface targets use them exclusively" comment). PUBLIC makes this
# target's own include requirement part of its exported INTERFACE, so
# consumers get it back via the target instead.
target_include_directories(fly_brain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(fly_brain PRIVATE "FLY_BRAIN_BUILDING_DLL")
ament_target_dependencies(fly_brain
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
  std_msgs
  mujoco_ros2_control_msgs
)

pluginlib_export_plugin_description_file(controller_interface fly_brain_plugins.xml)

install(
  TARGETS fly_brain
  EXPORT export_fly_brain
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_fly_brain HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle controller_interface hardware_interface pluginlib
  realtime_tools sensor_msgs std_msgs mujoco_ros2_control_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Plain-assert executable (not gtest) exercising the header-only algorithm
  # library with synthetic inputs only - no rclcpp/controller_interface
  # dependency, no MuJoCo/Docker needed to run it.
  add_executable(test_fly_brain_algorithms test/test_algorithms.cpp)
  target_include_directories(test_fly_brain_algorithms PRIVATE include)
  add_test(NAME fly_brain_algorithms COMMAND test_fly_brain_algorithms)

  # M3a: analytic ground-truth optic-flow bench (see NOTES.md's "## M3"
  # section) - validates ReichardtEmdArray's antisymmetric/symmetric pooling
  # against the real pinhole-camera optic-flow equations for known egomotion,
  # BEFORE trusting it on live camera frames (M3b). Same style as the
  # algorithms test above: plain assert executable, no gtest, no ROS/MuJoCo.
  add_executable(test_m3a_analytic_flow_bench test/test_analytic_flow_bench.cpp)
  target_include_directories(test_m3a_analytic_flow_bench PRIVATE include)
  add_test(NAME fly_brain_m3a_analytic_flow_bench COMMAND test_m3a_analytic_flow_bench)
endif()

ament_export_include_directories(include)
ament_package()
