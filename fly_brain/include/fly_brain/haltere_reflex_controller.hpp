// M2: haltere-analog reflex controller. Unlike M1's BaselinePidController
// (plain controller_interface::ControllerInterface), this is a
// controller_interface::ChainableControllerInterface - a real, already-decided
// architectural requirement, not speculative gold-plating: M4's
// DescendingFusionController will chain in front of this one, writing this
// controller's exported roll/pitch/yaw_rate/thrust reference interfaces
// directly instead of this controller sourcing its own fixed setpoint. See
// NOTES.md's "## M2" section for the chainable-controller idiom notes
// (on_export_reference_interfaces_list/update_reference_from_subscribers/
// update_and_write_commands split, confirmed against this jazzy build's
// controller_interface/hardware_interface headers - no source, only compiled
// .so, was available for pid_controller to copy from directly).
//
// Standalone (not chained) mode - how M2 actually runs it, since
// DescendingFusionController doesn't exist yet - sources its setpoint from
// fixed parameters (roll=0, pitch=0, yaw_rate=0, thrust=mass*g feedforward)
// rather than an altitude PID: M2's focus is attitude/rate disturbance
// rejection (the haltere reflex proper), not altitude holding, and
// HaltereReflex::AttitudeSetpoint::thrust is a value, not something this
// controller derives from an altitude error. See NOTES.md for the rationale.
#ifndef FLY_BRAIN__HALTERE_REFLEX_CONTROLLER_HPP_
#define FLY_BRAIN__HALTERE_REFLEX_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>

#include "controller_interface/chainable_controller_interface.hpp"
#include "fly_brain/complementary_filter.hpp"
#include "fly_brain/haltere_reflex.hpp"
#include "fly_brain/mixer.hpp"
#include "fly_brain/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "semantic_components/imu_sensor.hpp"

namespace fly_brain {

class FLY_BRAIN_PUBLIC HaltereReflexController : public controller_interface::ChainableControllerInterface {
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

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
      override;

  controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Command interfaces claimed, in declaration order: thrust1..4/effort
  // (identical to fly_controller::BaselinePidController's - same MJCF
  // actuators), optionally followed by ONE extra chained command interface -
  // see optic_flow_yaw_interface_name_ below (M3). If present, it always
  // lands at index kThrustJoints.size() in command_interfaces_, since
  // ros2_control populates that vector in the same order
  // command_interface_configuration() returned its names.
  static constexpr std::array<const char*, 4> kThrustJoints = {"thrust1", "thrust2", "thrust3",
                                                                "thrust4"};

  // Exported reference interfaces (this controller's chainable input, per the
  // class-level comment above): "roll", "pitch", "yaw_rate", "thrust" - full
  // names are "<controller_name>/roll" etc, following the
  // hardware_interface::Handle(prefix_name, interface_name) convention (see
  // NOTES.md). Populated every cycle by either update_reference_from_
  // subscribers() (standalone mode, fixed setpoint) or an upstream chained
  // controller writing directly through these same handles (chained mode,
  // M4+) - update_and_write_commands() just reads whatever is there.
  hardware_interface::CommandInterface::SharedPtr roll_ref_;
  hardware_interface::CommandInterface::SharedPtr pitch_ref_;
  hardware_interface::CommandInterface::SharedPtr yaw_rate_ref_;
  hardware_interface::CommandInterface::SharedPtr thrust_ref_;

  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;
  std::unique_ptr<fly_brain::ComplementaryFilter> attitude_filter_;
  std::unique_ptr<fly_brain::HaltereReflex> reflex_;
  std::unique_ptr<fly_brain::QuadMixer> mixer_;

  // Parameters (plain declare_parameter/get_parameter, matching
  // BaselinePidController's approach - see fly_controller/CMakeLists.txt's
  // comment on why generate_parameter_library wasn't used).
  double mass_ = 1.325;
  double min_thrust_per_rotor_ = 0.0;
  double max_thrust_per_rotor_ = 13.0;
  double complementary_filter_alpha_ = 0.98;

  // Standalone-mode (is_in_chained_mode() == false) fixed setpoint - see the
  // class-level comment for why this isn't an altitude PID output.
  double standalone_roll_setpoint_ = 0.0;
  double standalone_pitch_setpoint_ = 0.0;
  double standalone_yaw_rate_setpoint_ = 0.0;
  // Defaults to mass_ * 9.80665 in on_init() unless overridden - same
  // hover-feedforward derivation as BaselinePidController's hover_thrust.
  double standalone_thrust_setpoint_ = 0.0;

  // M3: optional chained optic-flow yaw correction (the "optomotor" piece of
  // the future DescendingFusionController's fusion law, exercised directly
  // here per the M3 brief rather than building the full task-command API -
  // see NOTES.md's "## M3" section). When enabled, this controller claims
  // ONE extra command interface - by full name, e.g.
  // "optic_flow_controller/roll_drift" - which is really
  // OpticFlowController's exported reference interface from ITS point of
  // view; from this controller's point of view it's just another command
  // interface, the standard ros2_control chaining pattern (see NOTES.md's
  // "## M2" section). The name is a parameter, not hardcoded, because which
  // field actually tracks camera yaw rotation is NOT "yaw_rate" (see
  // OpticFlowController/reichardt_emd.hpp's own doc comments and the M3a
  // bench: it's "forward_drift" under the bench's idealized panoramic-camera
  // model) and, on the real tilted/perspective nose_cam, empirically
  // "roll_drift" carried the more repeatable disturbance-correlated signal
  // instead - see NOTES.md's "## M3" section in full, including an honest
  // caveat about how noisy the live signal still is. This parameter is where
  // that real-physics-vs-naming (and idealized-bench-vs-real-camera)
  // mismatch gets resolved, once, in config rather than in code.
  bool use_optic_flow_yaw_input_ = false;
  std::string optic_flow_yaw_interface_name_;
  double optic_flow_yaw_gain_ = 0.0;
};

}  // namespace fly_brain

#endif  // FLY_BRAIN__HALTERE_REFLEX_CONTROLLER_HPP_
