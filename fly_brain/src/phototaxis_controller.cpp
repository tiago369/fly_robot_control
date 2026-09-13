#include "fly_brain/phototaxis_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace fly_brain {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

InterfaceConfiguration PhototaxisController::command_interface_configuration() const {
  // Pure perception controller, same as OpticFlowController: claims no
  // hardware command interfaces at all.
  InterfaceConfiguration config;
  config.type = interface_configuration_type::NONE;
  return config;
}

InterfaceConfiguration PhototaxisController::state_interface_configuration() const {
  // No hardware_interface state interfaces either - only input is the
  // /drone/camera/image_raw topic subscription.
  InterfaceConfiguration config;
  config.type = interface_configuration_type::NONE;
  return config;
}

CallbackReturn PhototaxisController::on_init() {
  try {
    image_topic_ = auto_declare<std::string>("image_topic", image_topic_);
    target_r_ = auto_declare<int>("target_r", target_r_);
    target_g_ = auto_declare<int>("target_g", target_g_);
    target_b_ = auto_declare<int>("target_b", target_b_);
    color_tolerance_ = auto_declare<int>("color_tolerance", color_tolerance_);
    min_channel_spread_ = auto_declare<int>("min_channel_spread", min_channel_spread_);
    min_blob_pixels_ = auto_declare<int>("min_blob_pixels", min_blob_pixels_);

    fly_brain::ColorThreshold thresh;
    thresh.target_r = target_r_;
    thresh.target_g = target_g_;
    thresh.target_b = target_b_;
    thresh.tolerance = color_tolerance_;
    thresh.min_channel_spread = min_channel_spread_;
    detector_ = std::make_unique<fly_brain::ColorBlobDetector>(thresh);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "PhototaxisController::on_init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PhototaxisController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  have_last_stamp_ = false;
  image_sub_ = get_node()->create_subscription<Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PhototaxisController::image_callback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
PhototaxisController::on_export_reference_interfaces_list() {
  const std::string prefix = get_node()->get_name();
  target_visible_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "target_visible");
  bearing_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "bearing");
  area_fraction_ref_ = std::make_shared<hardware_interface::CommandInterface>(prefix, "area_fraction");

  static_cast<void>(target_visible_ref_->set_value(0.0));
  static_cast<void>(bearing_ref_->set_value(0.0));
  static_cast<void>(area_fraction_ref_->set_value(0.0));

  std::vector<hardware_interface::CommandInterface::SharedPtr> refs;
  refs.push_back(target_visible_ref_);
  refs.push_back(bearing_ref_);
  refs.push_back(area_fraction_ref_);
  return refs;
}

CallbackReturn PhototaxisController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  have_last_stamp_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn PhototaxisController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void PhototaxisController::image_callback(const std::shared_ptr<Image> msg) {
  image_buffer_.writeFromNonRT(msg);
}

controller_interface::return_type PhototaxisController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // No-op by design, same reasoning as OpticFlowController's own: these
  // exported interfaces are sensor OUTPUT, never a setpoint written from
  // outside - all real work happens in update_and_write_commands() below,
  // which runs unconditionally every cycle regardless of chained mode.
  return controller_interface::return_type::OK;
}

controller_interface::return_type PhototaxisController::update_and_write_commands(
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
    // Zero-order hold - same rationale as OpticFlowController.
    return controller_interface::return_type::OK;
  }
  last_processed_stamp_ = msg.header.stamp;
  have_last_stamp_ = true;

  fly_brain::ColorBlobResult result;
  if (msg.encoding == "rgb8" && msg.width > 0 && msg.height > 0) {
    const std::size_t width = msg.width;
    const std::size_t height = msg.height;
    const std::size_t step = msg.step > 0 ? msg.step : width * 3;
    if (step == width * 3 && msg.data.size() >= step * height) {
      // Fast path: no row padding, buffer is already exactly
      // width*height*3 contiguous rgb8 bytes - feed it directly.
      result = detector_->detect(msg.data.data(), static_cast<int>(width),
                                  static_cast<int>(height), min_blob_pixels_);
    } else if (msg.data.size() >= step * height) {
      // Row-padded buffer: de-stride into a contiguous scratch buffer first
      // (ColorBlobDetector::detect()'s documented contract is unpadded rgb8,
      // same contract OpticFlowController::downsample_grayscale() places on
      // its own caller).
      std::vector<uint8_t> contiguous(width * height * 3);
      for (std::size_t y = 0; y < height; ++y) {
        std::copy_n(msg.data.begin() + static_cast<long>(y * step), width * 3,
                    contiguous.begin() + static_cast<long>(y * width * 3));
      }
      result = detector_->detect(contiguous.data(), static_cast<int>(width),
                                  static_cast<int>(height), min_blob_pixels_);
    }
    // else: malformed/short buffer - leave `result` at its default
    // (not visible), same "sane all-zero rather than reading OOB" policy
    // OpticFlowController::downsample_grayscale() uses.
  }

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                        "phototaxis: visible=%d bearing=%.4f area_fraction=%.5f pixels=%d",
                        result.visible ? 1 : 0, result.centroid_x, result.area_fraction,
                        result.pixel_count);

  bool ok = true;
  ok &= target_visible_ref_->set_value(result.visible ? 1.0 : 0.0);
  ok &= bearing_ref_->set_value(result.centroid_x);
  ok &= area_fraction_ref_->set_value(result.area_fraction);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Failed to write one or more phototaxis reference interface values");
  }

  return controller_interface::return_type::OK;
}

}  // namespace fly_brain

PLUGINLIB_EXPORT_CLASS(fly_brain::PhototaxisController, controller_interface::ChainableControllerInterface)
