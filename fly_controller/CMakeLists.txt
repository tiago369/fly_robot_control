cmake_minimum_required(VERSION 3.16)
project(fly_controller)

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
find_package(fly_brain REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)

# M1: BaselinePidController - the conventional PID hover controller used as
# the comparison baseline against fly_brain's own bio-inspired controllers
# (M2 onward, which will get their own package's pluginlib export, per
# fly_brain/CMakeLists.txt's comment). Plain declare_parameter/get_parameter
# (via ControllerInterfaceBase::auto_declare) is used instead of
# generate_parameter_library - the parameter set is small and flat enough
# that the extra codegen step wasn't worth it for M1's scope.
add_library(fly_controller SHARED src/baseline_pid_controller.cpp)
target_include_directories(fly_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(fly_controller PRIVATE "FLY_CONTROLLER_BUILDING_DLL")
ament_target_dependencies(fly_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  fly_brain
  mujoco_ros2_control_msgs
)

pluginlib_export_plugin_description_file(controller_interface fly_controller_plugins.xml)

install(
  TARGETS fly_controller
  EXPORT export_fly_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY urdf config models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_fly_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle controller_interface hardware_interface pluginlib
  realtime_tools fly_brain mujoco_ros2_control_msgs)

ament_package()
