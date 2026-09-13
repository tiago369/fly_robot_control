#include "fly_controller/baseline_pid_controller.hpp"

#include <algorithm>
#include <array>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace fly_controller {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

InterfaceConfiguration BaselinePidController::command_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  for (const auto* joint : kThrustJoints) {
    config.names.push_back(std::string(joint) + "/effort");
  }
  return config;
}

InterfaceConfiguration BaselinePidController::state_interface_configuration() const {
  InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  config.names = imu_sensor_->get_state_interface_names();
  return config;
}

CallbackReturn BaselinePidController::on_init() {
  try {
    body_name_ = auto_declare<std::string>("body_name", "x2");
    z_setpoint_ = auto_declare<double>("z_setpoint", 0.5);
    mass_ = auto_declare<double>("mass", 1.325);
    min_thrust_per_rotor_ = auto_declare<double>("min_thrust_per_rotor", 0.0);
    max_thrust_per_rotor_ = auto_declare<double>("max_thrust_per_rotor", 13.0);
    complementary_filter_alpha_ = auto_declare<double>("complementary_filter_alpha", 0.98);
    const double drag_to_thrust = auto_declare<double>("drag_to_thrust", 0.0201);

    fly_brain::CascadedPid::Gains gains;
    gains.kp_z = auto_declare<double>("kp_z", gains.kp_z);
    gains.ki_z = auto_declare<double>("ki_z", gains.ki_z);
    gains.kd_z = auto_declare<double>("kd_z", gains.kd_z);
    gains.kp_att_roll = auto_declare<double>("kp_att_roll", gains.kp_att_roll);
    gains.kp_att_pitch = auto_declare<double>("kp_att_pitch", gains.kp_att_pitch);
    gains.kp_rate_x = auto_declare<double>("kp_rate_x", gains.kp_rate_x);
    gains.kd_rate_x = auto_declare<double>("kd_rate_x", gains.kd_rate_x);
    gains.kp_rate_y = auto_declare<double>("kp_rate_y", gains.kp_rate_y);
    gains.kd_rate_y = auto_declare<double>("kd_rate_y", gains.kd_rate_y);
    gains.kp_rate_z = auto_declare<double>("kp_rate_z", gains.kp_rate_z);
    gains.kd_rate_z = auto_declare<double>("kd_rate_z", gains.kd_rate_z);
    // Feedforward: exact vehicle weight, not a tunable parameter (mass is the
    // parameter; hover_thrust is derived from it so there's one source of
    // truth). See NOTES.md's "## M1" section for how `mass_`'s default
    // (1.325 kg) was derived from x2.xml's geom masses and cross-checked
    // against its <keyframe name="hover"> ctrl values.
    gains.hover_thrust = mass_ * 9.80665;

    pid_ = std::make_unique<fly_brain::CascadedPid>(gains);
    attitude_filter_ = std::make_unique<fly_brain::ComplementaryFilter>(complementary_filter_alpha_);

    // Real rotor layout from the vendored MJCF (models/skydio_x2/x2.xml
    // <site name="thrust1..4">) - NOT a symmetric 45-degree X (x=+-0.14,
    // y=+-0.18, ~52 deg from +x - see mixer.hpp's RotorGeometry constructor
    // comment). Spin signs read off each rotor's <motor gear="0 0 1 0 0 s">
    // yaw-torque component sign.
    const std::array<fly_brain::QuadMixer::RotorGeometry, 4> rotors = {{
        {-0.14, -0.18, -1.0},  // thrust1
        {-0.14, 0.18, 1.0},    // thrust2
        {0.14, 0.18, -1.0},    // thrust3
        {0.14, -0.18, 1.0},    // thrust4
    }};
    mixer_ = std::make_unique<fly_brain::QuadMixer>(rotors, drag_to_thrust);

    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>("imu");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "BaselinePidController::on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaselinePidController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  have_z_ = false;
  free_joint_state_sub_ = get_node()->create_subscription<FreeJointStateArray>(
      "/drone/free_joint_states", rclcpp::SystemDefaultsQoS(),
      std::bind(&BaselinePidController::free_joint_state_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaselinePidController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!imu_sensor_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to assign IMU state interfaces");
    return CallbackReturn::ERROR;
  }
  have_z_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn BaselinePidController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  imu_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void BaselinePidController::free_joint_state_callback(const std::shared_ptr<FreeJointStateArray> msg) {
  free_joint_state_buffer_.writeFromNonRT(msg);
}

controller_interface::return_type BaselinePidController::update(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& period) {
  const double dt = period.seconds();

  const auto* msg_ptr = free_joint_state_buffer_.readFromRT();
  double z = 0.0;
  bool found_z = false;
  if (msg_ptr != nullptr && *msg_ptr) {
    for (const auto& fj : (*msg_ptr)->free_joints) {
      if (fj.name == body_name_) {
        z = fj.pose.pose.position.z;
        found_z = true;
        break;
      }
    }
  }
  if (found_z) {
    have_z_ = true;
  } else if (!have_z_) {
    // No /drone/free_joint_states sample has arrived yet (e.g. first few
    // cycles after activation) - hold thrust at zero rather than run the
    // altitude loop against a bogus z=0.0.
    return controller_interface::return_type::OK;
  }
  // else: reuse the last known z for this cycle (message arrives at 50 Hz,
  // slower than the 100 Hz control loop - see mujoco_ros2_control_plugins.yaml).

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

  const auto cmd = pid_->step(z_setpoint_, z, /*roll_setpoint=*/0.0, /*pitch_setpoint=*/0.0,
                               /*yaw_rate_setpoint=*/0.0, attitude, imu_sample, dt);

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

}  // namespace fly_controller

PLUGINLIB_EXPORT_CLASS(fly_controller::BaselinePidController, controller_interface::ControllerInterface)
