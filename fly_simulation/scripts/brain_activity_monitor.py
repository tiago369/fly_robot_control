#!/usr/bin/env python3
"""Fly Brain Activity Monitor.

A live "neuron cloud" view of the fly brain, in the style of connectome
point-cloud viewers: individual points (real Reichardt-EMD detector cells
from optic_flow_controller's exported /optic_flow_controller/neuron_activity
grid, plus a smaller "reflex core" cluster standing in for the haltere +
descending-fusion signal) laid out in two roughly eye-shaped lobes, each
brightening toward white as that unit's real activation increases. Nothing
here is fabricated: every point's brightness comes from a real published
value; only its fixed on-screen position is an artistic (not anatomical)
layout choice, since this project's brain has ~1100 units total, not the
~140k of a real connectome.

Run inside the container (after `ros2 launch fly_simulation <...>.launch.py`
is already up in another shell):
    ros2 run fly_simulation brain_activity_monitor.py
"""
import math
import random
import threading
import tkinter as tk

import rclpy
from controller_manager_msgs.srv import ListControllers
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

CANVAS_W, CANVAS_H = 900, 620
BG = "#050507"
DIM_COLOR = (26, 26, 53)  # dim blue-gray at zero activation
BRIGHT_COLOR = (255, 255, 255)  # white at full activation

VISION_GRID_W, VISION_GRID_H = 32, 32  # must match optic_flow_controller's grid_width/grid_height
CORE_POINT_COUNT = 90


def lerp_color(t):
    t = max(0.0, min(1.0, t))
    r = int(DIM_COLOR[0] + t * (BRIGHT_COLOR[0] - DIM_COLOR[0]))
    g = int(DIM_COLOR[1] + t * (BRIGHT_COLOR[1] - DIM_COLOR[1]))
    b = int(DIM_COLOR[2] + t * (BRIGHT_COLOR[2] - DIM_COLOR[2]))
    return f"#{r:02x}{g:02x}{b:02x}"


def lobe_layout(rng):
    """Fixed (seeded) point positions: two almond/eye-shaped lobes (real
    optic-lobe-inspired layout, split left/right by grid column - our single
    camera doesn't have two eyes, this is purely a familiar visual grouping)
    for the VISION_GRID_W*VISION_GRID_H EMD cells, laid out in row-major
    order matching neuron_activity's data layout."""
    positions = []
    lobe_w, lobe_h = 260, 300
    cy = CANVAS_H * 0.34
    half_w = VISION_GRID_W // 2
    for row in range(VISION_GRID_H):
        v = (row / (VISION_GRID_H - 1)) - 0.5  # -0.5..0.5
        angle = v * math.pi * 0.85
        for col in range(VISION_GRID_W):
            left = col < half_w
            cx = CANVAS_W * (0.32 if left else 0.68)
            u = ((col % half_w) / max(1, half_w - 1)) - 0.5  # -0.5..0.5
            radius_x = (lobe_w / 2) * math.cos(angle)
            x = cx + u * 2.0 * radius_x
            y = cy + v * lobe_h
            x += rng.uniform(-6, 6)
            y += rng.uniform(-6, 6)
            positions.append((x, y))
    return positions


def core_layout(rng):
    """Elongated jittered blob below the vision lobes, standing in for the
    haltere-reflex + descending-fusion "core" (a handful of scalar signals,
    rendered as a small point cluster rather than a single dot so it reads
    visually as part of the same brain-cloud style)."""
    positions = []
    cx, cy = CANVAS_W * 0.5, CANVAS_H * 0.72
    for _ in range(CORE_POINT_COUNT):
        u = rng.gauss(0, 1)
        v = rng.gauss(0, 1)
        x = cx + u * 26
        y = cy + v * 90
        positions.append((x, y))
    return positions


class BrainMonitorNode(Node):
    def __init__(self):
        super().__init__("brain_activity_monitor")
        self.lock = threading.Lock()
        self.controller_states = {}
        self.vision_activity = [0.0] * (VISION_GRID_W * VISION_GRID_H)
        self.gyro_mag = 0.0
        self.z = None
        self.vz = None
        # Slowly-decaying max, so the visualization stays reactive regardless
        # of the EMD's actual (small, unit-dependent) magnitude scale.
        self._vision_scale = 1e-4
        self._gyro_scale = 1e-3

        self._list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.create_timer(0.5, self._poll_controllers)

        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 10)
        self.create_subscription(
            Float32MultiArray,
            "/optic_flow_controller/neuron_activity",
            self._on_vision_activity,
            5,
        )
        self.create_subscription(
            FreeJointStateArray, "/drone/free_joint_states", self._on_free_joint, 10
        )

    def _poll_controllers(self):
        if not self._list_client.service_is_ready():
            return
        future = self._list_client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_list_controllers)

    def _on_list_controllers(self, future):
        try:
            resp = future.result()
        except Exception:  # noqa: BLE001 - best-effort UI polling, never fatal
            return
        with self.lock:
            self.controller_states = {c.name: c.state for c in resp.controller}

    def _on_imu(self, msg):
        mag = (
            msg.angular_velocity.x ** 2
            + msg.angular_velocity.y ** 2
            + msg.angular_velocity.z ** 2
        ) ** 0.5
        with self.lock:
            self.gyro_mag = mag
            self._gyro_scale = max(self._gyro_scale * 0.999, mag, 1e-3)

    def _on_vision_activity(self, msg):
        with self.lock:
            self.vision_activity = list(msg.data)
            peak = max(self.vision_activity) if self.vision_activity else 0.0
            self._vision_scale = max(self._vision_scale * 0.999, peak, 1e-4)

    def _on_free_joint(self, msg):
        if not msg.free_joints:
            return
        fj = msg.free_joints[0]
        with self.lock:
            self.z = fj.pose.pose.position.z
            self.vz = fj.twist.twist.linear.z

    def snapshot(self):
        with self.lock:
            return {
                "controller_states": dict(self.controller_states),
                "vision_activity": list(self.vision_activity),
                "vision_scale": self._vision_scale,
                "gyro_mag": self.gyro_mag,
                "gyro_scale": self._gyro_scale,
                "z": self.z,
                "vz": self.vz,
            }


class BrainCloudApp:
    def __init__(self, root, node: BrainMonitorNode):
        self.root = root
        self.node = node
        rng = random.Random(1234)  # fixed seed: stable layout across runs

        root.title("Fly Brain Activity Monitor")
        root.configure(bg=BG)

        self.canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg=BG, highlightthickness=0)
        self.canvas.pack()

        self.title_text = self.canvas.create_text(
            16, 16, anchor="nw", fill="#888888", font=("Sans", 11, "bold"), text=""
        )
        self.status_text = self.canvas.create_text(
            CANVAS_W / 2, CANVAS_H - 14, fill="#666666", font=("Sans", 9), text=""
        )

        self.vision_positions = lobe_layout(rng)
        self.vision_dots = [
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=lerp_color(0), outline="")
            for x, y in self.vision_positions
        ]

        self.core_positions = core_layout(rng)
        self.core_dots = [
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=lerp_color(0), outline="")
            for x, y in self.core_positions
        ]

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh()

    def refresh(self):
        snap = self.node.snapshot()
        states = snap["controller_states"]

        activity = snap["vision_activity"]
        scale = snap["vision_scale"] or 1e-4
        if len(activity) == len(self.vision_dots):
            for dot, value in zip(self.vision_dots, activity):
                self.canvas.itemconfig(dot, fill=lerp_color(value / scale))

        gyro_t = (snap["gyro_mag"] or 0.0) / (snap["gyro_scale"] or 1e-3)
        core_color = lerp_color(gyro_t)
        for dot in self.core_dots:
            self.canvas.itemconfig(dot, fill=core_color)

        n_units = len(self.vision_dots) + len(self.core_dots)
        active = [n for n, s in states.items() if s == "active"]
        self.canvas.itemconfig(
            self.title_text,
            text=f"fly_brain — {n_units} units — active: {', '.join(active) if active else 'none'}",
        )

        z, vz = snap["z"], snap["vz"]
        status = f"z={z:.3f} m  vz={vz:+.3f} m/s" if z is not None else "waiting for sim..."
        self.canvas.itemconfig(self.status_text, text=status)

        self.root.after(120, self.refresh)

    def on_close(self):
        rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    node = BrainMonitorNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    BrainCloudApp(root, node)
    root.mainloop()


if __name__ == "__main__":
    main()
