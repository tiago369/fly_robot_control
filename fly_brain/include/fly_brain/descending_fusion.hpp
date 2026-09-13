#pragma once
#include "fly_brain/haltere_reflex.hpp"
#include "fly_brain/reichardt_emd.hpp"

namespace fly_brain {

struct TaskCommand {
  double desired_roll = 0.0, desired_pitch = 0.0, desired_yaw_rate = 0.0, desired_thrust = 0.0;
};

// Descending-neuron analog: modulates the haltere reflex's setpoints with
// optic-flow self-motion estimates rather than bypassing the reflex loop,
// mirroring how real descending neurons steer reflex circuits instead of
// replacing them. Vision runs slower than the haltere loop, so the caller is
// expected to hold the last OpticFlowEstimate between vision updates
// (zero-order hold) rather than this class re-deriving cadence itself.
struct FusionGains {
  double k_task_yaw_rate = 1.0;
  double k_optomotor = 0.5;   // oppose perceived self-rotation
  double k_drift_roll = 0.3;  // course/centering correction
  double k_drift_pitch = 0.3;
};

class DescendingFusion {
public:
  using Gains = FusionGains;

  explicit DescendingFusion(Gains gains = Gains{}) : g_(gains) {}

  // M4 field-naming resolution (see NOTES.md's "## M3" and "## M4" sections
  // for the full derivation - summarized here since this is the one place
  // the decision actually gets made, per M3's own TODO: "decide ... whether
  // to rename the struct fields or just swap which field each gain
  // multiplies, rather than leaving the mismatch implicit").
  //
  // DECISION: keep fly_brain::OpticFlowEstimate's field NAMES unchanged
  // (renaming would ripple into OpticFlowController's already-shipped,
  // verified exported reference-interface names - e.g.
  // "optic_flow_controller/roll_drift", which controller_manager_m3.yaml's
  // haltere_reflex_controller already claims by that literal string - with no
  // benefit beyond cosmetics). Instead, swap which field the optomotor gain
  // multiplies, here, in the one place a caller actually turns flow into a
  // setpoint:
  //   - M3a's IDEALIZED bench (a horizon-level panoramic camera) found
  //     "forward_drift" is the field that tracks yaw rotation, not the
  //     literal "yaw_rate" field (see NOTES.md's "## M3" section,
  //     test/test_analytic_flow_bench.cpp).
  //   - M3b's REAL camera (nose_cam: tilted ~18deg down at a nearby ground
  //     plane, full perspective - geometrically different from the bench)
  //     found EMPIRICALLY that "roll_drift" carries an even more repeatable
  //     disturbance-correlated yaw signal than "forward_drift" on that real
  //     geometry (gain=100, ~6-7% faster yaw-disturbance recovery - a real
  //     but modest effect, not a dramatic one - see NOTES.md's "## M3"
  //     verification table).
  //   - DescendingFusionController runs against the SAME real nose_cam
  //     OpticFlowController M3b validated against (not the idealized bench),
  //     so this fuse() uses flow.roll_drift for the optomotor term -
  //     matching what was actually measured on the real running system, not
  //     the bench's idealized geometry.
  //
  // The k_drift_roll/k_drift_pitch course-centering terms below are NOT
  // touched by this decision - M3 never validated ANY field for lateral/
  // longitudinal drift-correction (only the yaw/optomotor path was ever
  // tested against a real disturbance), so DescendingFusionController ships
  // these gains defaulted to 0.0 (see its own on_init()) rather than wiring
  // an unvalidated field to a nonzero gain. They stay wired to their
  // pre-M3 fields (roll_drift/forward_drift) so the API/struct shape is
  // ready for a future tuning pass (e.g. the corridor-centering stretch
  // goal), but are inert unless a caller explicitly opts in with a nonzero
  // gain AND validates the field choice the same empirical way M3 did.
  AttitudeSetpoint fuse(const TaskCommand& task, const OpticFlowEstimate& flow) const {
    AttitudeSetpoint sp;
    sp.yaw_rate = g_.k_task_yaw_rate * task.desired_yaw_rate - g_.k_optomotor * flow.roll_drift;
    sp.roll = task.desired_roll + g_.k_drift_roll * flow.roll_drift;
    sp.pitch = task.desired_pitch + g_.k_drift_pitch * flow.forward_drift;
    sp.thrust = task.desired_thrust;
    return sp;
  }

private:
  Gains g_;
};

}  // namespace fly_brain
