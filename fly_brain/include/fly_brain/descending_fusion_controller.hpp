// M4: descending-fusion controller - the top of the fly-brain chain. A
// controller_interface::ChainableControllerInterface (same idiom as M2's
// HaltereReflexController and M3's OpticFlowController - see NOTES.md's
// "## M2"/"## M3" sections for the chainable-controller gotchas this
// followed verbatim rather than re-discovering).
//
// Unlike HaltereReflexController/OpticFlowController, nothing chains INTO
// this controller today. A real, previously-undocumented ros2_control
// constraint found while bringing this controller up for the first time
// (see NOTES.md's "## M4" section): controller_manager's compiled
// ChainableControllerInterface loader REJECTS a chainable controller
// outright at configure time if on_export_reference_interfaces_list() (and
// on_export_state_interfaces_list()) both return empty - "Controller ... is
// chainable, but does not export any state or reference interfaces" - so an
// empty export list (the naive "nothing chains into the top of the chain"
// implementation) doesn't actually work. Fix: export exactly ONE reference
// interface, "task_forward_pitch" - the task command's forward-pitch
// setpoint (see below), which is genuinely useful as a chain point even
// though nothing claims it today (a future mission/waypoint-level
// controller could chain into it to set forward-flight commands
// dynamically, instead of this controller's own fixed parameter). This
// satisfies the constraint with a real, non-fake exported interface rather
// than a placeholder.
//
// What it actually does: claims HaltereReflexController's exported
// "roll"/"pitch"/"yaw_rate"/"thrust" reference interfaces as its OWN command
// interfaces (the standard ros2_control chaining pattern - a downstream
// controller's reference interfaces are command interfaces from an upstream
// controller's point of view - see NOTES.md's "## M2" section), and
// optionally OpticFlowController's chosen reference interface the same way
// (superseding HaltereReflexController's own M3 optic-flow bolt-on - see
// NOTES.md's "## M3" TODO for M4: "HaltereReflexController's M3 extension is
// a narrow, single-axis, additive preview of fusion - M4's real
// DescendingFusionController should supersede it"). Each control cycle:
//   1. Reads /drone/free_joint_states (topic, same pattern
//      fly_controller::BaselinePidController already uses - see NOTES.md's
//      "## M1" section) for z, and runs a small closed-loop altitude PID
//      (duplicated here rather than factored out of CascadedPid - the
//      amount of logic is small, and CascadedPid also bundles an attitude/
//      rate loop this controller doesn't need, since that's
//      HaltereReflex's job downstream).
//   2. Reads a fixed task command from parameters (level roll, a
//      configurable pitch/yaw-rate setpoint - enough to demonstrate
//      hover -> forward flight without a full trajectory system, per the
//      M4 brief's own "don't over-build this" guidance).
//   3. Reads OpticFlowController's chosen reference interface (if enabled).
//   4. (M5) Reads PhototaxisController's target_visible/bearing/
//      area_fraction reference interfaces (if enabled) and, when a target is
//      visible, overrides the task command's yaw_rate/pitch with a real
//      turn-toward/approach law; when no target is visible, commands a slow
//      constant search yaw rate instead - see the M5 member comments below
//      for the exact law and NOTES.md's "## M5" section for why.
//   5. Calls fly_brain::DescendingFusion::fuse() and writes the result into
//      HaltereReflexController's claimed roll/pitch/yaw_rate/thrust command
//      interfaces.
#ifndef FLY_BRAIN__DESCENDING_FUSION_CONTROLLER_HPP_
#define FLY_BRAIN__DESCENDING_FUSION_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "controller_interface/chainable_controller_interface.hpp"
#include "fly_brain/descending_fusion.hpp"
#include "fly_brain/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace fly_brain {

class FLY_BRAIN_PUBLIC DescendingFusionController
    : public controller_interface::ChainableControllerInterface {
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
  using FreeJointStateArray = mujoco_ros2_control_msgs::msg::FreeJointStateArray;

  void free_joint_state_callback(const std::shared_ptr<FreeJointStateArray> msg);

  // Command interfaces claimed, in declaration order: HaltereReflexController's
  // 4 exported reference interfaces (roll, pitch, yaw_rate, thrust - full
  // names built from haltere_controller_name_), optionally followed by ONE
  // extra chained command interface sourced from OpticFlowController - same
  // "always appended last" convention HaltereReflexController's own M3
  // extension used (see haltere_reflex_controller.hpp) - and, M5, optionally
  // followed by exactly 3 more chained command interfaces sourced from
  // PhototaxisController (target_visible, bearing, area_fraction, always in
  // that order and always appended last of all). Their actual index depends
  // on whether the optic-flow interface is also enabled - computed once in
  // command_interface_configuration() and mirrored by
  // update_and_write_commands() via optic_flow_index()/phototaxis_index().
  static constexpr std::size_t kNumHaltereInterfaces = 4;
  static constexpr std::size_t kNumPhototaxisInterfaces = 3;

  // Index of the optic-flow command interface, if claimed (kNumHaltereInterfaces).
  std::size_t optic_flow_index() const { return kNumHaltereInterfaces; }
  // Index of the FIRST phototaxis command interface (target_visible), if
  // claimed - lands right after the optic-flow one when both are enabled, or
  // right after the haltere ones when only phototaxis is enabled.
  std::size_t phototaxis_index() const {
    return kNumHaltereInterfaces + (use_optic_flow_input_ && !optic_flow_source_interface_.empty() ? 1 : 0);
  }

  std::unique_ptr<fly_brain::DescendingFusion> fusion_;

  // The controller's one exported reference interface - see class-level
  // comment for why an empty export list doesn't work. Full name
  // "<node_name>/task_forward_pitch" (same convention as
  // HaltereReflexController/OpticFlowController's own exported interfaces).
  // Standalone mode (always true today, since nothing chains into this)
  // sources it from forward_pitch_setpoint_ every cycle in
  // update_reference_from_subscribers(), same idiom
  // HaltereReflexController's standalone setpoint uses; update_and_write_
  // commands() reads it back rather than reading forward_pitch_setpoint_
  // directly, so a future chained controller could drive it instead with no
  // change needed here.
  hardware_interface::CommandInterface::SharedPtr task_forward_pitch_ref_;

  rclcpp::Subscription<FreeJointStateArray>::SharedPtr free_joint_state_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<FreeJointStateArray>> free_joint_state_buffer_;

  // Parameters (plain declare_parameter/get_parameter - matching the rest of
  // this package, see fly_controller/CMakeLists.txt's comment on why
  // generate_parameter_library wasn't used).
  std::string body_name_ = "x2";
  std::string haltere_controller_name_ = "haltere_reflex_controller";

  double mass_ = 1.325;

  // Real closed-loop altitude hold (the M4 brief's explicit ask - M2/M3's
  // haltere_reflex_controller only ever had an open-loop "climb-compensated"
  // standalone thrust hack, fine for isolated attitude-only tests but not a
  // real hover). Same gain values M1's BaselinePidController validated
  // against this exact plant (see NOTES.md's "## M1" section) - not
  // re-derived from scratch.
  double z_setpoint_ = 0.5;
  double kp_z_ = 12.0;
  double ki_z_ = 2.0;
  double kd_z_ = 8.0;
  double z_integral_ = 0.0;
  double prev_err_z_ = 0.0;
  bool have_z_ = false;

  // Task command (fixed, parameter-sourced - see class comment: "enough to
  // demonstrate hover -> forward flight without a full trajectory system").
  // forward_pitch_setpoint_ != 0 tilts the vehicle forward, converting part
  // of thrust into forward acceleration (standard multirotor forward-flight
  // mechanism) - 0.0 (the default) is a pure hover task.
  double forward_pitch_setpoint_ = 0.0;
  double yaw_rate_setpoint_ = 0.0;

  // Optional chained optic-flow input - supersedes HaltereReflexController's
  // own M3 bolt-on (which should be disabled, via that controller's own
  // use_optic_flow_yaw_input parameter, whenever DescendingFusionController
  // is in the chain - see NOTES.md's "## M4" section). Field choice
  // (roll_drift, not yaw_rate/forward_drift) is resolved centrally in
  // fly_brain::DescendingFusion::fuse() - see descending_fusion.hpp's own
  // doc comment - so this controller only needs to know WHICH interface to
  // claim, not which struct field it semantically corresponds to.
  bool use_optic_flow_input_ = true;
  std::string optic_flow_source_interface_ = "optic_flow_controller/roll_drift";

  // Altitude-hold gate on the optic-flow correction - only trust it once
  // |z_setpoint - z| is under this threshold (m). See update_and_write_
  // commands()'s doc comment: a real, empirically-confirmed climb-transient/
  // optic-flow confound found during M4 verification, not speculative
  // hardening - see NOTES.md's "## M4" section.
  // A stricter version of this gate (additionally requiring |z_dot| under a
  // velocity threshold) was tried and REJECTED - see update_and_write_
  // commands()'s doc comment and NOTES.md's "## M4" section: it caused
  // gate-chattering that destabilized the whole stack, a real regression.
  double optic_flow_altitude_gate_ = 0.1;

  // Post-M4 fix (see NOTES.md's "## Post-M4" yaw-disturbance-regression
  // section): the hard on/off gate above still let a residual climb-transient
  // burst through at full k_optomotor strength the instant it first opened,
  // producing the ~45-56deg startup yaw-spin residual. Rather than adding a
  // SECOND hard threshold condition (the |z_dot| gate above, already tried
  // and rejected for chattering), this ramps the flow correction's effective
  // strength linearly from 0 to 1 over optic_flow_gate_ramp_duration_ seconds
  // AFTER the altitude gate first opens. This is monotonic and one-shot (once
  // latched open it never re-closes and the ramp only counts up), so it
  // cannot chatter the way a second AND-gated threshold on a noisy signal
  // did - it changes how HARD the existing gate opens, not whether a second
  // condition also has to be true.
  double optic_flow_gate_ramp_duration_ = 1.0;
  bool optic_flow_gate_opened_ = false;
  double time_since_gate_open_ = 0.0;

  // M5: optional chained phototaxis input (fly_brain::PhototaxisController) -
  // a real vision-based target-seeking task-command source, layered at the
  // SAME level as the fixed forward_pitch_setpoint_/yaw_rate_setpoint_
  // parameters above (both feed fly_brain::TaskCommand, which
  // DescendingFusion::fuse() then combines with the ever-active haltere/
  // optomotor reflex terms - "descending neurons modulate reflexes, they
  // don't bypass them", same framing as the rest of this controller). When
  // enabled and a target is currently visible, the phototaxis task
  // supersedes the fixed forward_pitch_setpoint_/yaw_rate_setpoint_ values
  // for that cycle; when no target is visible, a slow constant search
  // yaw-rate is commanded instead (a bounded "look around" behavior) rather
  // than reverting to the fixed setpoints, so the vehicle actively tries to
  // reacquire the target instead of just sitting in its last fixed task.
  // Defaults to false (unlike use_optic_flow_input_'s default of true) - M4's
  // controller_manager_m4.yaml doesn't know about this new M5 parameter and
  // never spawns fly_brain::PhototaxisController at all, so a `true` default
  // here would make command_interface_configuration() claim 3
  // "phototaxis_controller/*" interfaces that don't exist in M4's world,
  // breaking M4's activation outright. M5's own controller_manager_m5.yaml
  // explicitly sets this true.
  bool use_phototaxis_input_ = false;
  std::string phototaxis_controller_name_ = "phototaxis_controller";
  // Proportional gain from PhototaxisController's normalized bearing ([-1,1])
  // to a commanded yaw rate (rad/s) - turn toward the target. See
  // update_and_write_commands()'s comment for the sign derivation (bearing>0
  // means the target is toward the drone's physical RIGHT, given nose_cam's
  // mounting - see fly_drone_x2.xml - which needs a NEGATIVE yaw_rate to turn
  // right under this project's REP103 body frame, confirmed empirically
  // during M5 verification, not just derived on paper - see NOTES.md's
  // "## M5" section).
  double k_phototaxis_yaw_ = 1.2;
  // Forward-pitch command (rad) used when the target is visible and roughly
  // centered - tapered toward 0 both by how far off-center the bearing is
  // (don't charge forward while still turning) and by how large (close) the
  // target already appears (don't fly through/into it).
  double phototaxis_forward_pitch_ = 0.08;
  // area_fraction at which the forward-approach command is fully tapered to
  // zero (i.e. "close enough, stop translating") - tuned against the real
  // attractant sphere's real apparent size at the real hover altitude/camera
  // geometry, not guessed - see NOTES.md's "## M5" section.
  double phototaxis_stop_area_fraction_ = 0.08;
  // Constant yaw rate (rad/s) commanded while searching (target not
  // currently visible) - a slow, bounded "look around" sweep, not a fast
  // spin.
  double phototaxis_search_yaw_rate_ = 0.3;

  // M5: horizontal-velocity damping ("station keeping"). A real, empirically
  // -found bug during M5 verification, not speculative hardening: this
  // controller's pitch/roll setpoints command an ATTITUDE angle, not a
  // velocity - once the vehicle picks up horizontal speed under a nonzero
  // phototaxis_forward_pitch_ command and then loses the target (or centers
  // pitch back to 0 for any other reason), a level attitude does NOT stop
  // the vehicle; with MuJoCo's negligible air drag (x2.xml's
  // `viscosity="1.8e-5"`) it simply coasts at whatever velocity it had,
  // indefinitely - confirmed empirically: an early M5 test run drifted
  // several meters past/around the target rather than converging, entirely
  // because of this missing term, not a phototaxis-law sign error. Fix: feed
  // /drone/free_joint_states' twist.linear.{x,y} (world-frame) and the
  // body's current yaw (extracted from pose.pose.orientation) back into a
  // small proportional brake added to task.desired_pitch/roll, opposing
  // whatever body-frame horizontal velocity currently exists - active
  // whenever phototaxis is enabled (gated the same as the rest of the M5
  // logic, so it never touches M1-M4's own behavior).
  double k_vel_damp_pitch_ = 0.15;
  double k_vel_damp_roll_ = 0.15;
  double vx_world_ = 0.0, vy_world_ = 0.0, yaw_ = 0.0;
};

}  // namespace fly_brain

#endif  // FLY_BRAIN__DESCENDING_FUSION_CONTROLLER_HPP_
