#include "fly_brain/optic_flow_controller.hpp"

#include <algorithm>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace fly_brain {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

InterfaceConfiguration OpticFlowController::command_interface_configuration() const {
  // Pure perception controller: claims no hardware command interfaces at
  // all (never writes to a rotor or anything else) - see header comment.
  InterfaceConfiguration config;
  config.type = interface_configuration_type::NONE;
  return config;
}

InterfaceConfiguration OpticFlowController::state_interface_configuration() const {
  // No hardware_interface state interfaces either - its only input is the
  // /drone/camera/image_raw topic subscription (the camera is not a
  // hardware_interface state interface - see NOTES.md's "## M0" section).
  InterfaceConfiguration config;
  config.type = interface_configuration_type::NONE;
  return config;
}

CallbackReturn OpticFlowController::on_init() {
  try {
    image_topic_ = auto_declare<std::string>("image_topic", image_topic_);
    grid_width_ = auto_declare<int>("grid_width", grid_width_);
    grid_height_ = auto_declare<int>("grid_height", grid_height_);
    tau_ = auto_declare<double>("tau", tau_);

    emd_ = std::make_unique<fly_brain::ReichardtEmdArray>(static_cast<std::size_t>(grid_width_),
                                                           static_cast<std::size_t>(grid_height_),
                                                           tau_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "OpticFlowController::on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpticFlowController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  have_last_stamp_ = false;
  image_sub_ = get_node()->create_subscription<Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OpticFlowController::image_callback, this, std::placeholders::_1));
  activity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/neuron_activity", rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
OpticFlowController::on_export_reference_interfaces_list() {
  const std::string prefix = get_node()->get_name();
  yaw_rate_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "yaw_rate");
  roll_drift_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "roll_drift");
  forward_drift_ref_ =
      std::make_shared<hardware_interface::CommandInterface>(prefix, "forward_drift");
  vertical_drift_ref_ =
      std::make_shared<hardware_interface::CommandInterface>(prefix, "vertical_drift");

  static_cast<void>(yaw_rate_ref_->set_value(0.0));
  static_cast<void>(roll_drift_ref_->set_value(0.0));
  static_cast<void>(forward_drift_ref_->set_value(0.0));
  static_cast<void>(vertical_drift_ref_->set_value(0.0));

  std::vector<hardware_interface::CommandInterface::SharedPtr> refs;
  refs.push_back(yaw_rate_ref_);
  refs.push_back(roll_drift_ref_);
  refs.push_back(forward_drift_ref_);
  refs.push_back(vertical_drift_ref_);
  return refs;
}

CallbackReturn OpticFlowController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  have_last_stamp_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn OpticFlowController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void OpticFlowController::image_callback(const std::shared_ptr<Image> msg) {
  image_buffer_.writeFromNonRT(msg);
}

std::vector<float> OpticFlowController::downsample_grayscale(const Image& img) const {
  std::vector<float> grid(static_cast<std::size_t>(grid_width_) *
                               static_cast<std::size_t>(grid_height_),
                           0.0f);
  if (img.encoding != "rgb8" || img.width == 0 || img.height == 0) {
    return grid;  // sane all-zero grid rather than reading out of bounds
  }
  const std::size_t bytes_per_pixel = 3;
  const auto src_w = static_cast<std::size_t>(img.width);
  const auto src_h = static_cast<std::size_t>(img.height);
  const std::size_t step = img.step > 0 ? img.step : src_w * bytes_per_pixel;

  for (int gy = 0; gy < grid_height_; ++gy) {
    const auto y0 = (static_cast<std::size_t>(gy) * src_h) / static_cast<std::size_t>(grid_height_);
    auto y1 = (static_cast<std::size_t>(gy + 1) * src_h) / static_cast<std::size_t>(grid_height_);
    y1 = std::max(y1, y0 + 1);
    y1 = std::min(y1, src_h);

    for (int gx = 0; gx < grid_width_; ++gx) {
      const auto x0 =
          (static_cast<std::size_t>(gx) * src_w) / static_cast<std::size_t>(grid_width_);
      auto x1 = (static_cast<std::size_t>(gx + 1) * src_w) / static_cast<std::size_t>(grid_width_);
      x1 = std::max(x1, x0 + 1);
      x1 = std::min(x1, src_w);

      double sum = 0.0;
      std::size_t count = 0;
      for (std::size_t y = y0; y < y1; ++y) {
        const std::size_t row_base = y * step;
        for (std::size_t x = x0; x < x1; ++x) {
          const std::size_t idx = row_base + x * bytes_per_pixel;
          if (idx + 2 >= img.data.size()) continue;
          const double r = static_cast<double>(img.data[idx]);
          const double g = static_cast<double>(img.data[idx + 1]);
          const double b = static_cast<double>(img.data[idx + 2]);
          // Standard luminance weighting.
          sum += (0.299 * r + 0.587 * g + 0.114 * b) / 255.0;
          ++count;
        }
      }
      grid[static_cast<std::size_t>(gy) * static_cast<std::size_t>(grid_width_) +
           static_cast<std::size_t>(gx)] =
          count > 0 ? static_cast<float>(sum / static_cast<double>(count)) : 0.0f;
    }
  }
  return grid;
}

controller_interface::return_type OpticFlowController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // No-op by design: unlike HaltereReflexController's exported interfaces
  // (setpoints, meaningfully sourced from a standalone fixed value when not
  // chained), this controller's exported interfaces are sensor *output* -
  // nothing ever writes a "requested" value into them from outside. All real
  // work happens in update_and_write_commands() below, which - critically -
  // runs unconditionally every cycle regardless of chained mode, unlike this
  // method (see header comment and NOTES.md's "## M3" section).
  return controller_interface::return_type::OK;
}

controller_interface::return_type OpticFlowController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  const auto* msg_ptr = image_buffer_.readFromRT();
  if (msg_ptr == nullptr || !(*msg_ptr)) {
    return controller_interface::return_type::OK;  // no frame yet at all
  }
  const auto& msg = **msg_ptr;

  const bool is_new_frame =
      !have_last_stamp_ || msg.header.stamp.sec != last_processed_stamp_.sec ||
      msg.header.stamp.nanosec != last_processed_stamp_.nanosec;
  if (!is_new_frame) {
    // Zero-order hold: camera (~11-15Hz) publishes slower than this
    // controller's 100Hz cycle - just leave the previously-written reference
    // interface values in place (see header comment).
    return controller_interface::return_type::OK;
  }

  double dt = 0.0;
  if (have_last_stamp_) {
    dt = (static_cast<double>(msg.header.stamp.sec) -
          static_cast<double>(last_processed_stamp_.sec)) +
         (static_cast<double>(msg.header.stamp.nanosec) -
          static_cast<double>(last_processed_stamp_.nanosec)) *
             1e-9;
  }
  last_processed_stamp_ = msg.header.stamp;
  have_last_stamp_ = true;

  if (dt <= 0.0) {
    // First frame ever (dt unused - ReichardtEmdArray::update() just seeds
    // its low-pass state on the first call regardless of dt) or a
    // non-monotonic stamp (shouldn't happen with sim time, but don't feed a
    // negative/zero dt into the EMD's low-pass filter).
    dt = 1.0 / 30.0;
  }

  const auto grid = downsample_grayscale(msg);
  const auto est = emd_->update(grid, dt);

  {
    std_msgs::msg::Float32MultiArray activity_msg;
    activity_msg.layout.dim.resize(2);
    activity_msg.layout.dim[0].label = "height";
    activity_msg.layout.dim[0].size = static_cast<uint32_t>(emd_->height());
    activity_msg.layout.dim[0].stride =
        static_cast<uint32_t>(emd_->width() * emd_->height());
    activity_msg.layout.dim[1].label = "width";
    activity_msg.layout.dim[1].size = static_cast<uint32_t>(emd_->width());
    activity_msg.layout.dim[1].stride = static_cast<uint32_t>(emd_->width());
    activity_msg.data = emd_->last_activity();
    activity_pub_->publish(activity_msg);
  }

  // Low-rate telemetry (camera already only publishes at ~7-15Hz, so this is
  // not spammy) - genuinely useful for verifying the live EMD pipeline
  // produces sane, non-NaN, correctly-signed output (see NOTES.md's "## M3"
  // section's M3b verification), not a throwaway debug line.
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                        "optic flow: yaw_rate=%.5f roll_drift=%.5f forward_drift=%.5f "
                        "vertical_drift=%.5f dt=%.4f",
                        est.yaw_rate, est.roll_drift, est.forward_drift, est.vertical_drift, dt);

  if (!yaw_rate_ref_->set_value(est.yaw_rate) || !roll_drift_ref_->set_value(est.roll_drift) ||
      !forward_drift_ref_->set_value(est.forward_drift) ||
      !vertical_drift_ref_->set_value(est.vertical_drift)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to write one or more optic-flow reference interface values");
  }

  return controller_interface::return_type::OK;
}

}  // namespace fly_brain

PLUGINLIB_EXPORT_CLASS(fly_brain::OpticFlowController, controller_interface::ChainableControllerInterface)
