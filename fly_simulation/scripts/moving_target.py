#!/usr/bin/env python3
"""M6: Moving-target relocation node.

Environment/simulation-orchestration logic, not part of the fly's brain
(nothing here perceives anything - it reads ground-truth free-joint state and
calls a MuJoCo teleport service) - so it lives in fly_simulation as a plain
rclpy script, same idiom as brain_activity_monitor.py (installed the same
way, see CMakeLists.txt).

What it does: watches /drone/free_joint_states for the drone's ("x2") and the
attractant target's ("attractant_target") ground-truth horizontal position.
Once the drone gets within `approach_threshold` of the target (a bit BEFORE
M5's own converged hover distance of ~0.88-0.92m, so relocation triggers
during the final approach rather than only after the drone has already
settled - see the module-level constants below for the exact numbers and
why), it calls mujoco_ros2_control_node's real `set_free_joint_state` service
(mujoco_ros2_control_msgs/srv/SetFreeJointState) to teleport the target to a
new point ~3m from the drone's CURRENT position (matching M5's own validated
spawn geometry - see NOTES.md's "## M5" section), at a bearing within the
camera's horizontal FOV of the drone's current heading so the newly-placed
target is actually re-detectable rather than landing out of frame or behind
the drone.

A simple 2-state debounce (WAITING_FOR_APPROACH / WAITING_FOR_DEPARTURE, with
hysteresis between the trigger and re-arm distances, see NOTES.md's "## M6"
section) stops it from re-triggering every control cycle while the drone
lingers near a target it has already caught.

Requires the M6 scene (models/skydio_x2/drone_scene_m6.xml, loaded via
m6_moving_target.launch.py) where "attractant_target" is a real MuJoCo
free-joint body - M1-M5's drone_scene.xml has it welded to the world with no
joint, which set_free_joint_state rejects outright ("Body is not driven by a
free joint").

Usage (inside the container, after `ros2 launch fly_simulation
m6_moving_target.launch.py` is already up in another shell):
    ros2 run fly_simulation moving_target.py

Verify in another shell:
    ros2 topic echo /drone/free_joint_states
    (watch this node's own log output for "RELOCATING" lines with the real
    before/after target position and drone distance at that instant)
"""
import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.node import Node

DRONE_BODY_NAME = "x2"
TARGET_BODY_NAME = "attractant_target"

# Trigger the relocation a bit BEFORE the drone fully converges (M5's own
# verified converged hover distance was ~0.876-0.920m from the target
# center - see NOTES.md's "## M5" section), per the brief's own suggested
# 1.2-1.5m range: 1.3m is comfortably inside the phase where the drone is
# still visibly closing in (so the "catch" reads as a real approach, not a
# no-op at spawn) but early enough that the drone doesn't have to sit through
# the last ~0.4m of slow taper-limited crawl (the distance-taper mechanism
# documented in "## M5" makes that final stretch the slowest part of the
# approach) before every single relocation.
APPROACH_THRESHOLD_M = 1.3

# Hysteresis: don't re-arm (allow the NEXT relocation to trigger) until the
# drone is this far from the target again. A relocation itself jumps the
# distance back out to ~RELOCATE_SPAWN_DISTANCE_M (~3m, see below), so this
# threshold is crossed almost immediately after a successful relocation - its
# real job is guarding against re-triggering on the SAME approach (e.g. if
# the drone's own bearing/area_fraction wobble, documented in "## M5",
# transiently pushes distance back below 1.3m right at the moment of
# relocation, before /drone/free_joint_states has published the target's new,
# far-away position).
DEPART_REARM_THRESHOLD_M = 2.0

# Minimum wall-clock time between relocations, regardless of distance -
# belt-and-suspenders against any pathological rapid-refire (e.g. a stuck
# service response or a message-ordering fluke), never expected to bind in
# normal operation.
MIN_RELOCATE_INTERVAL_S = 2.0

# New target distance from the drone's CURRENT position at the moment of
# relocation - identical to M5's own original drone-to-target spawn distance
# (3.0m, drone_scene.xml's attractant_target placement), reusing that
# already-validated "far enough to be a real navigation test, close enough to
# converge in well under a minute" geometry rather than picking a new number.
RELOCATE_SPAWN_DISTANCE_M = 3.0

# New target bearing = the drone's CURRENT yaw +/- a random offset within
# this half-angle. nose_cam is fovy=45deg (vertical half-angle 22.5deg,
# unset in fly_drone_x2.xml so MuJoCo's default applies - see NOTES.md's
# "## M5" section) at 640x480 (4:3 aspect); horizontal FOV widens from the
# vertical one by the aspect ratio: 2*atan(tan(22.5deg)*4/3) ~= 57.8deg
# total, i.e. a +/-28.9deg horizontal half-angle. +/-20deg keeps a real
# margin inside that (the sphere isn't a point target, and the drone's own
# heading wobbles by design - see "## M5"'s bearing oscillation note - so a
# target placed right at the edge of the frustum could drift out of frame
# before the drone finishes turning toward it).
RELOCATE_ANGLE_HALF_RANGE_RAD = math.radians(20.0)

TARGET_Z_M = 0.15  # sphere radius - rests exactly on the ground plane, matches drone_scene.xml/drone_scene_m6.xml.

WAITING_FOR_APPROACH = "WAITING_FOR_APPROACH"
WAITING_FOR_DEPARTURE = "WAITING_FOR_DEPARTURE"


def yaw_from_quaternion(q):
    """Standard atan2 yaw extraction, same formula
    descending_fusion_controller.cpp uses (see NOTES.md's "## M5" section) -
    kept numerically identical rather than re-derived independently."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MovingTargetNode(Node):
    def __init__(self):
        super().__init__("moving_target")

        self.drone_xyz = None
        self.drone_yaw = None
        self.target_xyz = None

        self.state = WAITING_FOR_APPROACH
        self.last_relocate_time = None  # rclpy.time.Time, None until the first relocation.
        self.relocate_count = 0

        self._rng = random.Random()

        self._set_free_joint_state_client = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state"
        )

        self.create_subscription(
            FreeJointStateArray, "/drone/free_joint_states", self._on_free_joint_states, 10
        )

        self.get_logger().info(
            f"moving_target up: approach_threshold={APPROACH_THRESHOLD_M}m, "
            f"depart_rearm_threshold={DEPART_REARM_THRESHOLD_M}m, "
            f"relocate_spawn_distance={RELOCATE_SPAWN_DISTANCE_M}m, "
            f"relocate_angle_half_range={math.degrees(RELOCATE_ANGLE_HALF_RANGE_RAD):.1f}deg."
        )

    def _on_free_joint_states(self, msg: FreeJointStateArray):
        for fj in msg.free_joints:
            if fj.name == DRONE_BODY_NAME:
                p = fj.pose.pose.position
                self.drone_xyz = (p.x, p.y, p.z)
                self.drone_yaw = yaw_from_quaternion(fj.pose.pose.orientation)
            elif fj.name == TARGET_BODY_NAME:
                p = fj.pose.pose.position
                self.target_xyz = (p.x, p.y, p.z)

        if self.drone_xyz is None or self.target_xyz is None:
            return

        dx = self.drone_xyz[0] - self.target_xyz[0]
        dy = self.drone_xyz[1] - self.target_xyz[1]
        distance = math.hypot(dx, dy)

        stamp = msg.header.stamp
        sim_t = stamp.sec + stamp.nanosec * 1e-9

        if self.state == WAITING_FOR_APPROACH and distance < APPROACH_THRESHOLD_M:
            if self._cooldown_elapsed():
                self._relocate(distance, sim_t)
        elif self.state == WAITING_FOR_DEPARTURE and distance > DEPART_REARM_THRESHOLD_M:
            self.state = WAITING_FOR_APPROACH
            self.get_logger().info(
                f"[t={sim_t:.2f}] re-armed (distance={distance:.3f}m > "
                f"{DEPART_REARM_THRESHOLD_M}m) - waiting for next approach."
            )

    def _cooldown_elapsed(self) -> bool:
        if self.last_relocate_time is None:
            return True
        return (self.get_clock().now() - self.last_relocate_time).nanoseconds >= (
            MIN_RELOCATE_INTERVAL_S * 1e9
        )

    def _relocate(self, distance_at_trigger: float, sim_t: float):
        old_target = self.target_xyz
        drone_x, drone_y, _ = self.drone_xyz
        heading = self.drone_yaw if self.drone_yaw is not None else 0.0
        angle = heading + self._rng.uniform(-RELOCATE_ANGLE_HALF_RANGE_RAD, RELOCATE_ANGLE_HALF_RANGE_RAD)
        new_x = drone_x + RELOCATE_SPAWN_DISTANCE_M * math.cos(angle)
        new_y = drone_y + RELOCATE_SPAWN_DISTANCE_M * math.sin(angle)

        self.state = WAITING_FOR_DEPARTURE
        self.last_relocate_time = self.get_clock().now()
        self.relocate_count += 1

        self.get_logger().info(
            f"[t={sim_t:.2f}] RELOCATING #{self.relocate_count}: distance={distance_at_trigger:.3f}m "
            f"< {APPROACH_THRESHOLD_M}m. target {old_target[0]:.3f},{old_target[1]:.3f} -> "
            f"{new_x:.3f},{new_y:.3f} (drone heading={math.degrees(heading):.1f}deg, "
            f"drone at {drone_x:.3f},{drone_y:.3f})."
        )

        request = SetFreeJointState.Request()
        entry = FreeJointState()
        entry.name = TARGET_BODY_NAME
        entry.pose = PoseStamped()
        entry.pose.header.frame_id = ""  # world frame
        entry.pose.pose.position.x = new_x
        entry.pose.pose.position.y = new_y
        entry.pose.pose.position.z = TARGET_Z_M
        # entry.pose.pose.orientation left at its message default (w=1, i.e.
        # identity) - a sphere's orientation is unobservable anyway.
        entry.twist = TwistStamped()
        entry.twist.header.frame_id = ""  # world frame
        # entry.twist.twist left at its message default (all-zero) - explicit
        # velocity zeroing so a relocated target never carries over any
        # residual qvel (belt-and-suspenders: the body is parked via
        # gravcomp=1 with no applied forces, so qvel should already be ~0 -
        # see drone_scene_m6.xml - but a teleport is the natural place to
        # also guarantee it, per the brief's own suggestion).
        request.free_joints = [entry]

        if not self._set_free_joint_state_client.service_is_ready():
            self.get_logger().warn(
                "set_free_joint_state service not ready yet - relocation request will wait."
            )
        future = self._set_free_joint_state_client.call_async(request)
        future.add_done_callback(self._on_relocate_response)

    def _on_relocate_response(self, future):
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
    node = MovingTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
