// M1: conventional (non-bio-inspired) baseline PID hover controller. Sanity
// checks the physical model end-to-end through the full ROS2 stack (see
// NOTES.md's "## M1" section for the verification run). This is a plain
// (non-chainable) controller_interface::ControllerInterface: it needs no
// reference/state interfaces exported to other controllers, and mixes real
// hardware_interface state interfaces (IMU) with a plain ROS2 topic
// subscription (/drone/free_joint_states, for altitude) - an established
// ros2_control pattern (see e.g. pid_controller's reference-topic
// subscription), not a hack.
#ifndef FLY_CONTROLLER__BASELINE_PID_CONTROLLER_HPP_
#define FLY_CONTROLLER__BASELINE_PID_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "fly_brain/cascaded_pid.hpp"
#include "fly_brain/complementary_filter.hpp"
#include "fly_brain/mixer.hpp"
#include "fly_controller/visibility_control.h"
#include "mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace fly_controller {

class FLY_CONTROLLER_PUBLIC BaselinePidController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

private:
  using FreeJointStateArray = mujoco_ros2_control_msgs::msg::FreeJointStateArray;

  void free_joint_state_callback(const std::shared_ptr<FreeJointStateArray> msg);

  // Command interfaces claimed, in declaration order: thrust1..4/effort.
  static constexpr std::array<const char*, 4> kThrustJoints = {"thrust1", "thrust2", "thrust3",
                                                                "thrust4"};

  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

  std::unique_ptr<fly_brain::ComplementaryFilter> attitude_filter_;
  std::unique_ptr<fly_brain::CascadedPid> pid_;
  std::unique_ptr<fly_brain::QuadMixer> mixer_;

  rclcpp::Subscription<FreeJointStateArray>::SharedPtr free_joint_state_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<FreeJointStateArray>> free_joint_state_buffer_;

  // Parameters (plain declare_parameter/get_parameter - see fly_controller's
  // CMakeLists.txt comment for why generate_parameter_library wasn't used).
  std::string body_name_;
  double z_setpoint_ = 0.5;
  double mass_ = 1.325;
  double min_thrust_per_rotor_ = 0.0;
  double max_thrust_per_rotor_ = 13.0;
  double complementary_filter_alpha_ = 0.98;

  bool have_z_ = false;
};

}  // namespace fly_controller

#endif  // FLY_CONTROLLER__BASELINE_PID_CONTROLLER_HPP_
