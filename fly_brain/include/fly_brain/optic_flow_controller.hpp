// M3: optic-flow-analog visual reflex controller. Like M2's
// HaltereReflexController, this is a controller_interface::
// ChainableControllerInterface, but a pure "sensor/estimator" leaf in the
// eventual fly-brain chain: it claims NO hardware command interfaces (it
// never writes to a rotor) and NO hardware state interfaces (its only input
// is a plain ROS2 topic subscription, /drone/camera/image_raw - the camera is
// not a hardware_interface state interface, see NOTES.md's "## M0" camera
// section). It exports fly_brain::OpticFlowEstimate's 4 fields as chainable
// reference interfaces for a downstream controller (M3's
// HaltereReflexController extension, later M4's DescendingFusionController)
// to claim as command interfaces.
//
// Real chainable-controller idiom subtlety found while building this (see
// NOTES.md's "## M3" section): unlike HaltereReflexController, whose exported
// reference interfaces are *setpoints* (meant to be written by an upstream
// controller once chained, or by this controller's own
// update_reference_from_subscribers() when standalone),
// OpticFlowController's exported interfaces are *sensor output* - nothing
// upstream of it ever writes them; only this controller's own camera
// processing does, in every case. Since ChainableControllerInterface::
// update() only calls update_reference_from_subscribers() when NOT
// is_in_chained_mode() (and this controller likely runs chained, once a
// downstream controller claims one of its reference interfaces),  all of the
// real perception work happens in update_and_write_commands() instead, which
// runs unconditionally every cycle regardless of chained mode - see the .cpp
// for the actual split.
#ifndef FLY_BRAIN__OPTIC_FLOW_CONTROLLER_HPP_
#define FLY_BRAIN__OPTIC_FLOW_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "fly_brain/reichardt_emd.hpp"
#include "fly_brain/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace fly_brain {

class FLY_BRAIN_PUBLIC OpticFlowController : public controller_interface::ChainableControllerInterface {
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

  // Downsamples an rgb8 sensor_msgs/Image to a grid_width_ x grid_height_
  // row-major grayscale float grid in [0,1] (simple box-filter average per
  // grid cell - the source camera publishes at ~11-15Hz per NOTES.md's "##
  // M0" section, so there is plenty of headroom to do this per-frame, not
  // per-control-cycle - see update_and_write_commands()).
  std::vector<float> downsample_grayscale(const Image& img) const;

  // Exported reference interfaces: exact fly_brain::OpticFlowEstimate field
  // names ("yaw_rate", "roll_drift", "forward_drift", "vertical_drift") -
  // kept identical to the struct so a future DescendingFusionController (M4)
  // can claim them 1:1 without a translation layer. IMPORTANT (see NOTES.md's
  // "## M3" section): despite the name, "yaw_rate" is NOT the field that
  // tracks a real camera's yaw rotation - "forward_drift" is (confirmed by
  // the M3a analytic bench, test/test_analytic_flow_bench.cpp). This
  // controller exports the struct faithfully as-is (not renamed) to stay
  // compatible with fly_brain::OpticFlowEstimate and
  // fly_brain::DescendingFusion's existing field names; callers (this
  // milestone's own HaltereReflexController extension included) must pick
  // the field that matches the physical quantity they actually want, not the
  // field whose name matches.
  hardware_interface::CommandInterface::SharedPtr yaw_rate_ref_;
  hardware_interface::CommandInterface::SharedPtr roll_drift_ref_;
  hardware_interface::CommandInterface::SharedPtr forward_drift_ref_;
  hardware_interface::CommandInterface::SharedPtr vertical_drift_ref_;

  std::unique_ptr<fly_brain::ReichardtEmdArray> emd_;

  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<Image>> image_buffer_;

  // Per-cell |EMD response| grid, published purely for visualization (a
  // "brain_activity_monitor"-style live view of which vision units are
  // firing) - not consumed by any controller, safe to publish at whatever
  // rate update_and_write_commands() actually runs at (camera-frame-rate,
  // via the zero-order-hold logic below).
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr activity_pub_;

  // Parameters (plain declare_parameter/get_parameter, matching the rest of
  // this package - see fly_controller/CMakeLists.txt's comment on why
  // generate_parameter_library wasn't used).
  std::string image_topic_ = "/drone/camera/image_raw";
  int grid_width_ = 32;
  int grid_height_ = 32;
  double tau_ = 0.05;

  // Zero-order-hold state: the camera (~11-15Hz) publishes slower than the
  // controller_manager's control loop (100Hz, see controller_manager.yaml) -
  // exactly the "visual vs haltere reflex loop" timing mismatch the plan
  // anticipated. Only recompute the EMD when a genuinely NEW frame (by
  // header.stamp) has arrived; otherwise update_and_write_commands() leaves
  // the previously-written reference interface values untouched, which IS
  // the hold - CommandInterface storage persists whatever was last
  // set_value()'d.
  // Stored as raw builtin_interfaces::msg::Time (not rclcpp::Time) and
  // differenced by hand in the .cpp - rclcpp::Time subtraction throws at
  // runtime if the two Time objects were constructed with mismatched clock
  // types (RCL_ROS_TIME vs RCL_SYSTEM_TIME), and a default-constructed
  // rclcpp::Time here would default to RCL_SYSTEM_TIME while the image
  // message's header.stamp is sim time (use_sim_time:=true) - avoiding the
  // rclcpp::Time type entirely sidesteps that trap.
  bool have_last_stamp_ = false;
  builtin_interfaces::msg::Time last_processed_stamp_;
};

}  // namespace fly_brain

#endif  // FLY_BRAIN__OPTIC_FLOW_CONTROLLER_HPP_
