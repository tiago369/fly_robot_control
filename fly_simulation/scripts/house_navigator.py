#!/usr/bin/env python3
"""M7: House-navigation beacon-sequencing node.

Environment/simulation-orchestration logic, not part of the fly's brain
(nothing here perceives anything - same framing as M6's moving_target.py,
which this is directly adapted from), so it lives in fly_simulation as a
plain rclpy script, installed the same way (see CMakeLists.txt).

What it does: drives a single free-jointed beacon body ("attractant_target",
same mechanism as M6 - see models/skydio_x2/drone_scene_m7.xml's own header
comment) through a FIXED, ORDERED sequence of waypoints - one per
room/opening - rather than M6's repeating random relocation. The beacon
SPAWNS at WAYPOINTS[0] (baked into drone_scene_m7.xml, matching M5/M6's own
"initial position lives in the MJCF, the script only relocates it" pattern),
and each time the drone (body "x2") gets within `APPROACH_THRESHOLD_M` of the
CURRENT waypoint, this node teleports the beacon to the NEXT waypoint via
mujoco_ros2_control_node's real `set_free_joint_state` service (identical
call shape to moving_target.py - see that file's own comments for the
service/message details, not repeated here). After the FINAL waypoint is
reached, it logs "ARRIVED" once and stops - no further relocation, no
departure/re-arm hysteresis needed (unlike M6's repeating chase, each
advance's new waypoint is a fresh, far-away point, so there is no realistic
risk of an immediate re-trigger against a *different* target position).

Single moving body vs. multiple beacon bodies: this reuses M6's exact,
already-proven set_free_joint_state/gravcomp=1 parking mechanism verbatim
(see NOTES.md's "## M6" section) rather than adding N separate bodies plus
logic to swap which one is "live" (e.g. by teleporting the inactive ones far
away/underground, or juggling contype/conaffinity/visibility per body) - the
brief's own suggested default, and simpler here because there is still
exactly one "active" perceptual target at any moment either way.

Distance metric: HORIZONTAL only (hypot(dx, dy), ignoring z), same rationale
as moving_target.py - the drone holds altitude tightly and waypoint 1 is
deliberately at a different z (0.4, elevated to clear the window's sill, see
drone_scene_m7.xml) than waypoints 0/2 (0.15, floor-level), so factoring z
into the trigger distance would make the threshold mean different things per
leg for no benefit; horizontal distance is what "has the drone reached this
point in the room" actually means here.

Requires the M7 scene (models/skydio_x2/drone_scene_m7.xml, loaded via
m7_house_flight.launch.py) where "attractant_target" is a real MuJoCo
free-joint body - see NOTES.md's "## M7" section for the room/opening
geometry this waypoint sequence was designed against.

Usage (inside the container, after `ros2 launch fly_simulation
m7_house_flight.launch.py` is already up in another shell):
    ros2 run fly_simulation house_navigator.py

Verify in another shell:
    ros2 topic echo /drone/free_joint_states
    (watch this node's own log output for "ADVANCING" / "ARRIVED" lines with
    the real trigger distance and target/leg timing)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.node import Node

DRONE_BODY_NAME = "x2"
TARGET_BODY_NAME = "attractant_target"

# The fixed, ordered waypoint sequence - one per room/opening, ending at the
# final "arrival" room. Coordinates match models/skydio_x2/drone_scene_m7.xml
# exactly (see that file's own header comment for the full geometric
# reasoning behind each one - door/window sizing, camera-FOV/sill-clearance
# math for waypoint 1's elevated z, and the occlusion argument for waypoint
# 2's y offset). Each tuple is (x, y, z, label).
WAYPOINTS = [
    (3.0, 0.0, 0.15, "through door (Room 1 -> Room 2)"),
    (7.0, 1.0, 0.4, "through window (Room 2 -> Room 3)"),
    (9.0, 0.3, 0.15, "arrival (Room 3)"),
]

# Trigger the advance a bit BEFORE the drone would fully converge on the
# current waypoint - same reasoning as M6's APPROACH_THRESHOLD_M (1.3m, inside
# the brief's own suggested 1.2-1.5m band, comfortably above M5's own
# converged hover distance of ~0.88-0.92m so the drone is still visibly
# closing in, not sitting through the last few tens of cm of the
# distance-taper mechanism's slow final crawl - see NOTES.md's "## M5"
# section). Reused verbatim (same geometry class - a 0.15m-radius sphere,
# same phototaxis_stop_area_fraction_ - governs every leg here too).
APPROACH_THRESHOLD_M = 1.3

# Minimum wall-clock time between advances - belt-and-suspenders against a
# pathological rapid-refire (e.g. a stuck service response), same role as
# M6's MIN_RELOCATE_INTERVAL_S, never expected to bind in normal operation.
MIN_ADVANCE_INTERVAL_S = 2.0

TARGET_Z_UNUSED = None  # z comes from WAYPOINTS directly, per-leg (see module docstring).


class HouseNavigatorNode(Node):
    def __init__(self):
        super().__init__("house_navigator")

        self.drone_xyz = None
        self.target_index = 0  # which WAYPOINTS[] entry the beacon currently sits at / drone is chasing.
        self.arrived = False

        self.last_advance_time = None  # rclpy.time.Time, None until the first advance.
        self.leg_start_time = None  # rclpy.time.Time when the CURRENT leg started (for per-leg timing).

        self._set_free_joint_state_client = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state"
        )

        self.create_subscription(
            FreeJointStateArray, "/drone/free_joint_states", self._on_free_joint_states, 10
        )

        self.get_logger().info(
            f"house_navigator up: {len(WAYPOINTS)} waypoints, "
            f"approach_threshold={APPROACH_THRESHOLD_M}m. "
            f"Waypoint 0 ({WAYPOINTS[0][3]}) is baked into the MJCF spawn pose - "
            f"waiting for the drone to reach it."
        )

    def _on_free_joint_states(self, msg: FreeJointStateArray):
        if self.arrived:
            return

        for fj in msg.free_joints:
            if fj.name == DRONE_BODY_NAME:
                p = fj.pose.pose.position
                self.drone_xyz = (p.x, p.y, p.z)

        if self.drone_xyz is None:
            return

        if self.leg_start_time is None:
            self.leg_start_time = self.get_clock().now()

        wp_x, wp_y, _wp_z, _label = WAYPOINTS[self.target_index]
        dx = self.drone_xyz[0] - wp_x
        dy = self.drone_xyz[1] - wp_y
        distance = math.hypot(dx, dy)

        stamp = msg.header.stamp
        sim_t = stamp.sec + stamp.nanosec * 1e-9

        if distance < APPROACH_THRESHOLD_M and self._cooldown_elapsed():
            self._advance(distance, sim_t)

    def _cooldown_elapsed(self) -> bool:
        if self.last_advance_time is None:
            return True
        return (self.get_clock().now() - self.last_advance_time).nanoseconds >= (
            MIN_ADVANCE_INTERVAL_S * 1e9
        )

    def _advance(self, distance_at_trigger: float, sim_t: float):
        reached_label = WAYPOINTS[self.target_index][3]
        leg_elapsed_s = None
        if self.leg_start_time is not None:
            leg_elapsed_s = (self.get_clock().now() - self.leg_start_time).nanoseconds / 1e9

        self.last_advance_time = self.get_clock().now()

        if self.target_index >= len(WAYPOINTS) - 1:
            # Reached the final waypoint - log arrival, stop, no more teleports.
            self.arrived = True
            self.get_logger().info(
                f"[t={sim_t:.2f}] ARRIVED at waypoint {self.target_index} "
                f"({reached_label}): distance={distance_at_trigger:.3f}m < "
                f"{APPROACH_THRESHOLD_M}m"
                + (f", leg took {leg_elapsed_s:.2f}s" if leg_elapsed_s is not None else "")
                + ". Navigation complete - no further relocation."
            )
            return

        next_index = self.target_index + 1
        next_x, next_y, next_z, next_label = WAYPOINTS[next_index]

        self.get_logger().info(
            f"[t={sim_t:.2f}] ADVANCING from waypoint {self.target_index} "
            f"({reached_label}): distance={distance_at_trigger:.3f}m < "
            f"{APPROACH_THRESHOLD_M}m"
            + (f", leg took {leg_elapsed_s:.2f}s" if leg_elapsed_s is not None else "")
            + f". Teleporting beacon to waypoint {next_index} ({next_label}) at "
            f"({next_x:.3f}, {next_y:.3f}, {next_z:.3f})."
        )

        self.target_index = next_index
        self.leg_start_time = self.get_clock().now()

        request = SetFreeJointState.Request()
        entry = FreeJointState()
        entry.name = TARGET_BODY_NAME
        entry.pose = PoseStamped()
        entry.pose.header.frame_id = ""  # world frame
        entry.pose.pose.position.x = next_x
        entry.pose.pose.position.y = next_y
        entry.pose.pose.position.z = next_z
        # entry.pose.pose.orientation left at its message default (w=1, i.e.
        # identity) - a sphere's orientation is unobservable anyway.
        entry.twist = TwistStamped()
        entry.twist.header.frame_id = ""  # world frame
        # entry.twist.twist left at its message default (all-zero) - explicit
        # velocity zeroing so the relocated beacon never carries over any
        # residual qvel, same belt-and-suspenders reasoning as M6.
        request.free_joints = [entry]

        if not self._set_free_joint_state_client.service_is_ready():
            self.get_logger().warn(
                "set_free_joint_state service not ready yet - advance request will wait."
            )
        future = self._set_free_joint_state_client.call_async(request)
        future.add_done_callback(self._on_advance_response)

    def _on_advance_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - report, never crash the node over a bad response.
            self.get_logger().error(f"set_free_joint_state call failed: {exc!r}")
            return
        if response.success:
            self.get_logger().info(f"set_free_joint_state: {response.message}")
        else:
            self.get_logger().error(f"set_free_joint_state REJECTED: {response.message}")


def main():
    rclpy.init()
    node = HouseNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
