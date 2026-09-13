// M5: phototaxis-analog visual-target-seeking controller. Same
// controller_interface::ChainableControllerInterface idiom as M3's
// OpticFlowController (see NOTES.md's "## M2"/"## M3" sections for the
// chainable-controller gotchas this follows verbatim) - a pure
// "sensor/estimator" leaf, structurally almost identical to
// OpticFlowController: claims NO hardware command interfaces (never writes
// to a rotor) and NO hardware state interfaces (its only input is the
// /drone/camera/image_raw topic subscription), and exports its perception
// output as chainable reference interfaces for DescendingFusionController
// (M4/M5) to claim as command interfaces.
//
// The one real difference from OpticFlowController: this is color-based
// STATIC target detection (fly_brain::ColorBlobDetector, a per-pixel color
// threshold + centroid), not motion-based (fly_brain::ReichardtEmdArray, a
// Reichardt EMD array) - a real fly's phototaxis reflex responds to a
// salient target's appearance, not its motion, and a Reichardt EMD is blind
// to a perfectly static colored object (zero frame-to-frame luminance
// change at any pixel the object doesn't move across). See
// fly_brain/include/fly_brain/color_blob_detector.hpp and
// fly_brain/test/test_algorithms.cpp's test_color_blob_detector() for the
// synthetic-input validation done before this controller was wired to
// anything live (same discipline as M3a's analytic flow bench).
//
// Exported reference interfaces (full names
// "phototaxis_controller/<name>", matching OpticFlowController's own
// "<controller_name>/<field>" convention):
//   - "target_visible": 1.0 if a target-colored blob of at least
//     min_blob_pixels_ pixels is currently in frame, else 0.0.
//   - "bearing": the blob centroid's normalized horizontal image offset,
//     [-1, 1], 0 = image center. Positive = target appears toward image
//     +X (which nose_cam's mounting makes body -Y, i.e. the drone's
//     physical RIGHT - see fly_drone_x2.xml's "nose_cam" xyaxes and REP103's
//     x-fwd/y-left/z-up body frame). Held at 0.0 (with target_visible=0)
//     when no target is in frame - DescendingFusionController is expected to
//     gate on target_visible, not assume a nonzero bearing always means
//     something.
//   - "area_fraction": fraction of the frame's pixels matching the target
//     color, [0, 1] - a simple, correspondence-free "apparent size"/distance
//     proxy (bigger = closer). DescendingFusionController uses this to taper
//     off the forward-approach command as the target grows, rather than
//     flying through/into it.
#ifndef FLY_BRAIN__PHOTOTAXIS_CONTROLLER_HPP_
#define FLY_BRAIN__PHOTOTAXIS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "fly_brain/color_blob_detector.hpp"
#include "fly_brain/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace fly_brain {

class FLY_BRAIN_PUBLIC PhototaxisController : public controller_interface::ChainableControllerInterface {
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
  using Image = sensor_msgs::msg::Image;

  void image_callback(const std::shared_ptr<Image> msg);

  hardware_interface::CommandInterface::SharedPtr target_visible_ref_;
  hardware_interface::CommandInterface::SharedPtr bearing_ref_;
  hardware_interface::CommandInterface::SharedPtr area_fraction_ref_;

  std::unique_ptr<fly_brain::ColorBlobDetector> detector_;

  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Image>> image_buffer_;

  // Parameters (plain declare_parameter/get_parameter, matching the rest of
  // this package - see fly_controller/CMakeLists.txt's comment on why
  // generate_parameter_library wasn't used).
  std::string image_topic_ = "/drone/camera/image_raw";
  int target_r_ = 235, target_g_ = 70, target_b_ = 20;
  int color_tolerance_ = 70;
  int min_channel_spread_ = 50;
  int min_blob_pixels_ = 12;

  // Zero-order-hold state - identical rationale/mechanism to
  // OpticFlowController's own (see that header's comment): the camera
  // (~11-15Hz) publishes slower than the 100Hz control loop, and
  // builtin_interfaces::msg::Time (not rclcpp::Time) sidesteps the
  // sim-time-vs-system-time clock-type mismatch that rclcpp::Time
  // subtraction would throw on.
  bool have_last_stamp_ = false;
  builtin_interfaces::msg::Time last_processed_stamp_;
};

}  // namespace fly_brain

#endif  // FLY_BRAIN__PHOTOTAXIS_CONTROLLER_HPP_
