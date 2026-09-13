#include "fly_brain/descending_fusion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace fly_brain {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

InterfaceConfiguration DescendingFusionController::command_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  config.names.push_back(haltere_controller_name_ + "/roll");
  config.names.push_back(haltere_controller_name_ + "/pitch");
  config.names.push_back(haltere_controller_name_ + "/yaw_rate");
  config.names.push_back(haltere_controller_name_ + "/thrust");
  // Optionally claim OpticFlowController's chosen reference interface too -
  // always appended last, so it lands at command_interfaces_[
  // kNumHaltereInterfaces] if present (same convention
  // HaltereReflexController's own M3 extension used).
  if (use_optic_flow_input_ && !optic_flow_source_interface_.empty()) {
    config.names.push_back(optic_flow_source_interface_);
  }
  // M5: optionally claim PhototaxisController's 3 exported reference
  // interfaces too, always appended last of all (after optic-flow, if also
  // enabled) - see phototaxis_index()'s own comment for the exact resulting
  // index.
  if (use_phototaxis_input_) {
    config.names.push_back(phototaxis_controller_name_ + "/target_visible");
    config.names.push_back(phototaxis_controller_name_ + "/bearing");
    config.names.push_back(phototaxis_controller_name_ + "/area_fraction");
  }
  return config;
}

InterfaceConfiguration DescendingFusionController::state_interface_configuration() const {
  // No hardware_interface state interfaces - altitude comes from the
  // /drone/free_joint_states topic (same pattern
  // fly_controller::BaselinePidController uses), attitude/rate feedback is
  // HaltereReflexController's own job downstream.
  InterfaceConfiguration config;
  config.type = interface_configuration_type::NONE;
  return config;
}

CallbackReturn DescendingFusionController::on_init() {
  try {
    body_name_ = auto_declare<std::string>("body_name", body_name_);
    haltere_controller_name_ =
        auto_declare<std::string>("haltere_controller_name", haltere_controller_name_);
    mass_ = auto_declare<double>("mass", mass_);

    z_setpoint_ = auto_declare<double>("z_setpoint", z_setpoint_);
    kp_z_ = auto_declare<double>("kp_z", kp_z_);
    ki_z_ = auto_declare<double>("ki_z", ki_z_);
    kd_z_ = auto_declare<double>("kd_z", kd_z_);

    forward_pitch_setpoint_ = auto_declare<double>("forward_pitch_setpoint", forward_pitch_setpoint_);
    yaw_rate_setpoint_ = auto_declare<double>("yaw_rate_setpoint", yaw_rate_setpoint_);

    use_optic_flow_input_ = auto_declare<bool>("use_optic_flow_input", use_optic_flow_input_);
    optic_flow_source_interface_ =
        auto_declare<std::string>("optic_flow_source_interface", optic_flow_source_interface_);
    // See update_and_write_commands()'s comment for why this gate exists - a
    // real bug found during M4 verification, not speculative hardening.
    optic_flow_altitude_gate_ =
        auto_declare<double>("optic_flow_altitude_gate", optic_flow_altitude_gate_);

    // M5: optional chained phototaxis (color-target-seeking) input - see
    // header comment and descending_fusion_controller.hpp's own member
    // comments for the full law/reasoning.
    use_phototaxis_input_ = auto_declare<bool>("use_phototaxis_input", use_phototaxis_input_);
    phototaxis_controller_name_ =
        auto_declare<std::string>("phototaxis_controller_name", phototaxis_controller_name_);
    k_phototaxis_yaw_ = auto_declare<double>("k_phototaxis_yaw", k_phototaxis_yaw_);
    phototaxis_forward_pitch_ =
        auto_declare<double>("phototaxis_forward_pitch", phototaxis_forward_pitch_);
    phototaxis_stop_area_fraction_ =
        auto_declare<double>("phototaxis_stop_area_fraction", phototaxis_stop_area_fraction_);
    phototaxis_search_yaw_rate_ =
        auto_declare<double>("phototaxis_search_yaw_rate", phototaxis_search_yaw_rate_);
    k_vel_damp_pitch_ = auto_declare<double>("k_vel_damp_pitch", k_vel_damp_pitch_);
    k_vel_damp_roll_ = auto_declare<double>("k_vel_damp_roll", k_vel_damp_roll_);

    // fly_brain::DescendingFusion::Gains - see descending_fusion.hpp's own
    // doc comment for the field-naming resolution this implements
    // (k_optomotor multiplies flow.roll_drift, not flow.yaw_rate).
    // k_optomotor default (100.0) carried over directly from
    // haltere_reflex_controller's M3-validated optic_flow_yaw_gain (same
    // field, same real camera, same closed-loop-self-oscillation ceiling
    // documented in NOTES.md's "## M3" section - not re-derived from
    // scratch). k_drift_roll/k_drift_pitch default to 0.0 - deliberately
    // inert, since M3 never validated ANY field for lateral/longitudinal
    // drift correction (only the yaw/optomotor path was ever tested against
    // a real disturbance) - see descending_fusion.hpp's comment.
    fly_brain::DescendingFusion::Gains gains;
    gains.k_task_yaw_rate = auto_declare<double>("k_task_yaw_rate", 1.0);
    gains.k_optomotor = auto_declare<double>("k_optomotor", 100.0);
    gains.k_drift_roll = auto_declare<double>("k_drift_roll", 0.0);
    gains.k_drift_pitch = auto_declare<double>("k_drift_pitch", 0.0);
    fusion_ = std::make_unique<fly_brain::DescendingFusion>(gains);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "DescendingFusionController::on_init failed: %s",
                 e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DescendingFusionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  have_z_ = false;
  z_integral_ = 0.0;
  prev_err_z_ = 0.0;
  free_joint_state_sub_ = get_node()->create_subscription<FreeJointStateArray>(
      "/drone/free_joint_states", rclcpp::SystemDefaultsQoS(),
      std::bind(&DescendingFusionController::free_joint_state_callback, this,
                std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
DescendingFusionController::on_export_reference_interfaces_list() {
  // Exactly ONE exported reference interface - see class-level header
  // comment: controller_manager rejects a chainable controller outright at
  // configure time if this list (and on_export_state_interfaces_list()) are
  // both empty, so "nothing chains into the top of the chain" can't mean
  // "export nothing" here. Nothing claims this today, so is_in_chained_
  // mode() will still always be false in practice - update_reference_from_
  // subscribers() runs every cycle (standalone), same as if this were empty.
  const std::string prefix = get_node()->get_name();
  task_forward_pitch_ref_ =
      std::make_shared<hardware_interface::CommandInterface>(prefix, "task_forward_pitch");
  static_cast<void>(task_forward_pitch_ref_->set_value(forward_pitch_setpoint_));

  std::vector<hardware_interface::CommandInterface::SharedPtr> refs;
  refs.push_back(task_forward_pitch_ref_);
  return refs;
}

CallbackReturn DescendingFusionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  have_z_ = false;
  z_integral_ = 0.0;
  prev_err_z_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn DescendingFusionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void DescendingFusionController::free_joint_state_callback(
    const std::shared_ptr<FreeJointStateArray> msg) {
  free_joint_state_buffer_.writeFromNonRT(msg);
}

controller_interface::return_type DescendingFusionController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Standalone-mode setpoint sourcing (same idiom as
  // HaltereReflexController's own standalone setpoint - see NOTES.md's
  // "## M2" section): write the fixed, parameter-sourced forward-pitch task
  // command into the one exported reference interface every cycle. Since
  // nothing claims this interface today, this always runs (never
  // is_in_chained_mode()) - see on_export_reference_interfaces_list()'s own
  // comment for why the interface exists at all.
  if (!task_forward_pitch_ref_->set_value(forward_pitch_setpoint_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to write task_forward_pitch reference interface value");
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type DescendingFusionController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();

  const auto* msg_ptr = free_joint_state_buffer_.readFromRT();
  double z = 0.0;
  bool found_z = false;
  if (msg_ptr != nullptr && *msg_ptr) {
    for (const auto& fj : (*msg_ptr)->free_joints) {
      if (fj.name == body_name_) {
        z = fj.pose.pose.position.z;
        found_z = true;
        // M5: also capture world-frame horizontal velocity + current yaw,
        // for the velocity-damping term below (see its own member comment
        // in the header for why this is needed) - free, since this topic is
        // already subscribed for z.
        if (use_phototaxis_input_) {
          vx_world_ = fj.twist.twist.linear.x;
          vy_world_ = fj.twist.twist.linear.y;
          const auto& q = fj.pose.pose.orientation;
          yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }
        break;
      }
    }
  }
  if (found_z) {
    have_z_ = true;
  } else if (!have_z_) {
    // No /drone/free_joint_states sample yet (e.g. the first few cycles
    // after activation) - write a safe hover-feedforward fallback rather
    // than running the altitude loop against a bogus z=0.0 or leaving
    // HaltereReflexController's claimed interfaces at whatever they were
    // default-constructed to (this controller is the ONLY writer of those
    // interfaces once chained, so leaving them untouched here would mean
    // NaN/0 commands reach the reflex loop and, downstream, the rotors).
    for (std::size_t i = 0; i < kNumHaltereInterfaces; ++i) {
      static_cast<void>(command_interfaces_[i].set_value(i == 3 ? mass_ * 9.80665 : 0.0));
    }
    return controller_interface::return_type::OK;
  }
  // else: reuse the last known z this cycle (message arrives at 50 Hz,
  // slower than the 100 Hz control loop - same pattern as
  // BaselinePidController).

  // Real closed-loop altitude PID (the M4 brief's explicit ask - see class
  // comment). Small enough to duplicate directly rather than factor
  // CascadedPid's z-loop out into a shared piece - CascadedPid also bundles
  // an attitude/rate loop this controller doesn't need (that's
  // HaltereReflex's job downstream via the claimed roll/pitch/yaw_rate
  // interfaces).
  const double err_z = z_setpoint_ - z;
  z_integral_ += err_z * dt;
  const double z_dot = dt > 0.0 ? (err_z - prev_err_z_) / dt : 0.0;
  prev_err_z_ = err_z;
  const double hover_thrust = mass_ * 9.80665;
  const double thrust = hover_thrust + kp_z_ * err_z + ki_z_ * z_integral_ + kd_z_ * z_dot;

  // Altitude-hold gate on the optic-flow correction - a real bug found
  // during M4 verification (see NOTES.md's "## M4" section), not a
  // theoretical concern: the vehicle always spawns resting on the ground
  // (z~0.1, see NOTES.md's "## M0"/"## M2" sections) and this altitude PID's
  // own (real, closed-loop) initial climb transient up to z_setpoint_
  // produces a large, real receding-ground optic-flow burst - exactly M3's
  // documented "climb confounds optic flow" finding, but now triggered by
  // the UNAVOIDABLE initial climb-to-hover transient rather than M2/M3's
  // open-loop indefinite-climb hack. Left ungated, that burst on
  // flow.roll_drift, multiplied by k_optomotor (100, carried over from M3),
  // produced an uncommanded ~145 degree yaw spin in the first ~1-2s of every
  // run before settling - confirmed empirically, not predicted. Fix: only
  // trust the optic-flow correction once altitude error is within a gate
  // threshold (i.e. once the vehicle is actually near-hover, not still
  // actively climbing) - addresses the root cause (a real sensor confound)
  // rather than just lowering k_optomotor (which would also blunt the
  // yaw-disturbance-recovery benefit during an actual steady-hover
  // disturbance). NOTE - a stricter version of this gate was tried and
  // REJECTED (see NOTES.md's "## M4" section): additionally requiring
  // |z_dot| under a velocity threshold cut the initial kick further in
  // isolation, but in practice caused the gate to chatter open/closed near
  // the threshold, injecting discontinuous step changes in the fused
  // yaw-rate command that saturated individual rotors and, via QuadMixer's
  // allocation coupling under saturation (the mixer's exact-decoupling math
  // assumes unclipped per-rotor thrust), bled into roll/pitch/thrust and
  // produced a genuinely worse, sustained oscillation (z swinging between
  // ~0.33-0.73m, roll to +-19deg) - a real regression, not an improvement.
  // The single-condition altitude-error gate below doesn't chatter (err_z
  // decreases monotonically during the one-time climb, so the gate opens
  // exactly once and stays open) and is what's actually shipped.
  double flow_signal = 0.0;
  if (use_optic_flow_input_ && command_interfaces_.size() > kNumHaltereInterfaces &&
      std::abs(err_z) < optic_flow_altitude_gate_) {
    flow_signal = command_interfaces_[kNumHaltereInterfaces].get_optional<double>().value_or(0.0);
  }

  fly_brain::TaskCommand task;
  task.desired_roll = 0.0;
  // Read back through the exported reference interface (rather than
  // forward_pitch_setpoint_ directly) so a future chained controller could
  // drive this dynamically with no change needed here - see
  // on_export_reference_interfaces_list()'s comment.
  task.desired_pitch = task_forward_pitch_ref_->get_optional<double>().value_or(forward_pitch_setpoint_);
  task.desired_yaw_rate = yaw_rate_setpoint_;
  task.desired_thrust = thrust;

  // M5: phototaxis (color-target-seeking) task-command override - layered at
  // the SAME level as the fixed forward_pitch_setpoint_/yaw_rate_setpoint_
  // task command above (both feed fly_brain::TaskCommand; the haltere
  // attitude/rate reflex and the optomotor correction below stay ever-active
  // regardless - "descending neurons modulate reflexes, they don't bypass
  // them"). When PhototaxisController reports a target in frame, this
  // REPLACES the fixed pitch/yaw_rate task values for this cycle with a real
  // turn-toward/approach law; when no target is visible, a slow constant
  // search yaw rate is commanded instead of falling back to the fixed
  // (usually zero) setpoints, so the vehicle actively looks for the target
  // rather than just sitting still.
  if (use_phototaxis_input_ && command_interfaces_.size() > phototaxis_index() + 2) {
    const auto& visible_if = command_interfaces_[phototaxis_index()];
    const auto& bearing_if = command_interfaces_[phototaxis_index() + 1];
    const auto& area_if = command_interfaces_[phototaxis_index() + 2];
    const bool target_visible = visible_if.get_optional<double>().value_or(0.0) > 0.5;
    const double bearing = bearing_if.get_optional<double>().value_or(0.0);
    const double area_fraction = area_if.get_optional<double>().value_or(0.0);

    if (target_visible) {
      // Turn toward the target. SIGN DERIVATION (confirmed empirically
      // against the real running system during M5 verification, not just
      // derived on paper - see NOTES.md's "## M5" section): PhototaxisController's
      // "bearing" is positive when the target-colored blob's centroid sits
      // toward larger image-X (image right). nose_cam's mounting
      // (fly_drone_x2.xml: local +X camera axis == body (0,-1,0)) makes
      // image-right correspond to the body's physical RIGHT (-Y) under this
      // project's REP103 body frame (x-fwd, y-left, z-up - see mixer.hpp's
      // own header comment). A positive commanded yaw_rate rotates +X toward
      // +Y (LEFT) under the right-hand rule about +Z (up) - so turning
      // toward a target on the RIGHT (bearing>0) needs a NEGATIVE yaw_rate.
      task.desired_yaw_rate = -k_phototaxis_yaw_ * bearing;
      // Approach only when roughly centered (don't charge forward while
      // still turning to face it) and taper off as the target's apparent
      // size approaches phototaxis_stop_area_fraction_ (don't fly into it).
      const double centered_factor = std::max(0.0, 1.0 - std::abs(bearing));
      const double proximity_factor =
          phototaxis_stop_area_fraction_ > 0.0
              ? std::clamp(1.0 - area_fraction / phototaxis_stop_area_fraction_, 0.0, 1.0)
              : 1.0;
      task.desired_pitch = phototaxis_forward_pitch_ * centered_factor * proximity_factor;
    } else {
      // Search: slow constant yaw sweep, hold position otherwise (no blind
      // forward flight while the target isn't in frame).
      task.desired_yaw_rate = phototaxis_search_yaw_rate_;
      task.desired_pitch = 0.0;
    }

    // Horizontal-velocity damping ("station keeping") - see this
    // controller's own k_vel_damp_pitch_/k_vel_damp_roll_ member comment in
    // the header for the real bug this fixes (a level attitude does not
    // stop a coasting vehicle - there is essentially no aerodynamic drag in
    // this MJCF). Rotate the world-frame horizontal velocity into the
    // CURRENT body frame using yaw_ (pitch/roll setpoints are angles about
    // the body's own current x/y axes, which rotate with yaw in the world -
    // see mixer.hpp's REP103 x-fwd/y-left/z-up convention), then brake
    // proportionally to whatever's left. SIGN DERIVATION (see
    // update_and_write_commands()'s phototaxis-bearing comment above for the
    // same style of derivation, confirmed against the real system - see
    // NOTES.md's "## M5" section): positive pitch (nose down) accelerates
    // the vehicle in +x_body (forward), so decelerating a +x_body velocity
    // needs NEGATIVE pitch; positive roll tilts the vehicle's up vector
    // toward -y_body (physical right), producing a -y_body-direction force,
    // so decelerating a +y_body (leftward) velocity needs POSITIVE roll.
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double v_fwd_body = vx_world_ * cos_yaw + vy_world_ * sin_yaw;
    const double v_lat_body = -vx_world_ * sin_yaw + vy_world_ * cos_yaw;
    task.desired_pitch -= k_vel_damp_pitch_ * v_fwd_body;
    task.desired_roll += k_vel_damp_roll_ * v_lat_body;
  }

  // Only .roll_drift is actually read by DescendingFusion::fuse()'s
  // optomotor term (see its own doc comment); the other 3 fields are left at
  // 0.0 since k_drift_roll/k_drift_pitch default to 0.0 (deliberately
  // inert - see on_init()).
  fly_brain::OpticFlowEstimate flow{};
  flow.roll_drift = flow_signal;

  const auto sp = fusion_->fuse(task, flow);

  bool ok = true;
  ok &= command_interfaces_[0].set_value(sp.roll);
  ok &= command_interfaces_[1].set_value(sp.pitch);
  ok &= command_interfaces_[2].set_value(sp.yaw_rate);
  ok &= command_interfaces_[3].set_value(sp.thrust);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to write one or more fused setpoint values to %s/*",
                          haltere_controller_name_.c_str());
  }

  return controller_interface::return_type::OK;
}

}  // namespace fly_brain

PLUGINLIB_EXPORT_CLASS(fly_brain::DescendingFusionController,
                        controller_interface::ChainableControllerInterface)
