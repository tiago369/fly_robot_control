#include "fly_brain/haltere_reflex_controller.hpp"

#include <algorithm>
#include <array>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace fly_brain {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

InterfaceConfiguration HaltereReflexController::command_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  for (const auto* joint : kThrustJoints) {
    config.names.push_back(std::string(joint) + "/effort");
  }
  // M3: optionally claim one extra chained command interface - see header
  // comment. Always appended last, so it lands at command_interfaces_[
  // kThrustJoints.size()] if present.
  if (use_optic_flow_yaw_input_ && !optic_flow_yaw_interface_name_.empty()) {
    config.names.push_back(optic_flow_yaw_interface_name_);
  }
  return config;
}

InterfaceConfiguration HaltereReflexController::state_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  config.names = imu_sensor_->get_state_interface_names();
  return config;
}

CallbackReturn HaltereReflexController::on_init() {
  try {
    mass_ = auto_declare<double>("mass", 1.325);
    min_thrust_per_rotor_ = auto_declare<double>("min_thrust_per_rotor", 0.0);
    max_thrust_per_rotor_ = auto_declare<double>("max_thrust_per_rotor", 13.0);
    complementary_filter_alpha_ = auto_declare<double>("complementary_filter_alpha", 0.98);
    const double drag_to_thrust = auto_declare<double>("drag_to_thrust", 0.0201);

    // Standalone-mode fixed setpoint (see header comment - no
    // DescendingFusionController exists yet to chain in front of this
    // controller, so M2 runs it standalone and needs its own setpoint
    // source). Thrust defaults to hover feedforward (mass * g) plus a small
    // 2% climb margin - NOT exactly mass*g. See NOTES.md's "## M2" section:
    // exactly-hover thrust was tried first and produced a real (not just
    // theoretical) failure mode - with zero climb margin the drone never
    // clears the ground after spawning resting at z~0.1, so its rotors stay
    // close enough to the ground plane that ground-contact forces from any
    // tiny attitude perturbation eventually tip it over for real (a genuine
    // physical flip via MuJoCo contact dynamics, confirmed by watching the
    // free-running standalone controller flip to a stable upside-down rest
    // pose after ~10s with zero disturbance ever applied - not a numerical
    // artifact). The 2% margin gives a gentle, bounded climb (~0.2 m/s^2)
    // that clears the ground well before the disturbance test window without
    // needing an altitude PID (which would defeat the point of testing the
    // attitude/rate reflex in isolation).
    standalone_roll_setpoint_ = auto_declare<double>("standalone_roll_setpoint", 0.0);
    standalone_pitch_setpoint_ = auto_declare<double>("standalone_pitch_setpoint", 0.0);
    standalone_yaw_rate_setpoint_ = auto_declare<double>("standalone_yaw_rate_setpoint", 0.0);
    standalone_thrust_setpoint_ =
        auto_declare<double>("standalone_thrust_setpoint", mass_ * 9.80665 * 1.02);

    // M3: optional chained optic-flow yaw correction - see header comment.
    // Off by default so M1/M2's existing launches (which never set these
    // parameters) are completely unaffected.
    use_optic_flow_yaw_input_ = auto_declare<bool>("use_optic_flow_yaw_input", false);
    optic_flow_yaw_interface_name_ =
        auto_declare<std::string>("optic_flow_yaw_interface_name", "");
    optic_flow_yaw_gain_ = auto_declare<double>("optic_flow_yaw_gain", 0.0);

    fly_brain::HaltereReflex::Gains gains;
    gains.kp_att_roll = auto_declare<double>("kp_att_roll", gains.kp_att_roll);
    gains.kp_att_pitch = auto_declare<double>("kp_att_pitch", gains.kp_att_pitch);
    gains.kp_rate_x = auto_declare<double>("kp_rate_x", gains.kp_rate_x);
    gains.kd_rate_x = auto_declare<double>("kd_rate_x", gains.kd_rate_x);
    gains.kp_rate_y = auto_declare<double>("kp_rate_y", gains.kp_rate_y);
    gains.kd_rate_y = auto_declare<double>("kd_rate_y", gains.kd_rate_y);
    gains.kp_rate_z = auto_declare<double>("kp_rate_z", gains.kp_rate_z);
    gains.kd_rate_z = auto_declare<double>("kd_rate_z", gains.kd_rate_z);

    reflex_ = std::make_unique<fly_brain::HaltereReflex>(gains);
    attitude_filter_ = std::make_unique<fly_brain::ComplementaryFilter>(complementary_filter_alpha_);

    // Real rotor layout, identical to BaselinePidController's (see
    // NOTES.md's "## M1" section - not re-derived here, per its own M2
    // TODO).
    const std::array<fly_brain::QuadMixer::RotorGeometry, 4> rotors = {{
        {-0.14, -0.18, -1.0},  // thrust1
        {-0.14, 0.18, 1.0},    // thrust2
        {0.14, 0.18, -1.0},    // thrust3
        {0.14, -0.18, 1.0},    // thrust4
    }};
    mixer_ = std::make_unique<fly_brain::QuadMixer>(rotors, drag_to_thrust);

    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>("imu");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "HaltereReflexController::on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn HaltereReflexController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
HaltereReflexController::on_export_reference_interfaces_list() {
  const std::string prefix = get_node()->get_name();
  roll_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "roll");
  pitch_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "pitch");
  yaw_rate_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "yaw_rate");
  thrust_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "thrust");

  // Give the exported reference interfaces a sane initial value (rather than
  // NaN, the Handle default with no initial_value) - relevant only for the
  // brief window before the first update() cycle runs, and for a future
  // DescendingFusionController (M4) that may read these back before writing.
  static_cast<void>(roll_ref_->set_value(standalone_roll_setpoint_));
  static_cast<void>(pitch_ref_->set_value(standalone_pitch_setpoint_));
  static_cast<void>(yaw_rate_ref_->set_value(standalone_yaw_rate_setpoint_));
  static_cast<void>(thrust_ref_->set_value(standalone_thrust_setpoint_));

  std::vector<hardware_interface::CommandInterface::SharedPtr> refs;
  refs.push_back(roll_ref_);
  refs.push_back(pitch_ref_);
  refs.push_back(yaw_rate_ref_);
  refs.push_back(thrust_ref_);
  return refs;
}

CallbackReturn HaltereReflexController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!imu_sensor_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to assign IMU state interfaces");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn HaltereReflexController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  imu_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type HaltereReflexController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Standalone mode only (never called while is_in_chained_mode() - see
  // ChainableControllerInterface::update()). No subscriber exists yet: M2's
  // task is a fixed level-attitude-hold setpoint (see header comment) -
  // M4's DescendingFusionController will write these reference interfaces
  // directly instead, bypassing this method entirely (chained mode).
  if (!roll_ref_->set_value(standalone_roll_setpoint_) ||
      !pitch_ref_->set_value(standalone_pitch_setpoint_) ||
      !yaw_rate_ref_->set_value(standalone_yaw_rate_setpoint_) ||
      !thrust_ref_->set_value(standalone_thrust_setpoint_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to write one or more standalone reference interface values");
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type HaltereReflexController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();

  fly_brain::AttitudeSetpoint sp{};
  sp.roll = roll_ref_->get_optional<double>().value_or(0.0);
  sp.pitch = pitch_ref_->get_optional<double>().value_or(0.0);
  sp.yaw_rate = yaw_rate_ref_->get_optional<double>().value_or(0.0);
  sp.thrust = thrust_ref_->get_optional<double>().value_or(standalone_thrust_setpoint_);

  // M3: additive optic-flow yaw correction (optomotor reflex) - modulates the
  // EFFECTIVE yaw-rate setpoint fed to HaltereReflex::step() without
  // disturbing yaw_rate_ref_ itself (which stays whatever standalone mode or
  // a future DescendingFusionController put there - see header comment).
  // command_interfaces_[kThrustJoints.size()] is this controller's own
  // claimed copy of OpticFlowController's chosen reference interface (an
  // ordinary chained command interface from this controller's point of
  // view - see NOTES.md's "## M2"/"## M3" sections).
  if (use_optic_flow_yaw_input_ && command_interfaces_.size() > kThrustJoints.size()) {
    const double flow_signal =
        command_interfaces_[kThrustJoints.size()].get_optional<double>().value_or(0.0);
    sp.yaw_rate += optic_flow_yaw_gain_ * flow_signal;
  }

  fly_brain::ImuSample imu_sample{};
  const auto angular_velocity = imu_sensor_->get_angular_velocity();
  const auto linear_acceleration = imu_sensor_->get_linear_acceleration();
  imu_sample.wx = angular_velocity[0];
  imu_sample.wy = angular_velocity[1];
  imu_sample.wz = angular_velocity[2];
  imu_sample.ax = linear_acceleration[0];
  imu_sample.ay = linear_acceleration[1];
  imu_sample.az = linear_acceleration[2];

  const auto& attitude = attitude_filter_->update(imu_sample, dt);

  const auto cmd = reflex_->step(sp, attitude, imu_sample, dt);

  const auto thrusts = mixer_->allocate(cmd.thrust, cmd.tau_x, cmd.tau_y, cmd.tau_z);

  for (std::size_t i = 0; i < thrusts.size(); ++i) {
    const double clipped = std::clamp(thrusts[i], min_thrust_per_rotor_, max_thrust_per_rotor_);
    if (!command_interfaces_[i].set_value(clipped)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Failed to write command for %s/effort", kThrustJoints[i]);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace fly_brain

PLUGINLIB_EXPORT_CLASS(fly_brain::HaltereReflexController, controller_interface::ChainableControllerInterface)
