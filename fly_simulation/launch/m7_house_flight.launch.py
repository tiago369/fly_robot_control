"""
M7: same full fly-brain chain as M5/M6 (optic_flow_controller +
phototaxis_controller -> haltere_reflex_controller ->
descending_fusion_controller, all spawned together via one
--activate-as-group call - see m5_phototaxis.launch.py's own docstring and
NOTES.md's "## M3" section for why that's non-negotiable), navigating a small
3-room "house" (models/skydio_x2/drone_scene_m7.xml) from room to room
through a door and a window using vision-only phototaxis - no scripted
waypoint-following of the drone itself. scripts/house_navigator.py drives a
single free-jointed beacon body through a FIXED, ORDERED sequence of 3
waypoints (one per room/opening, see that script's own module docstring) via
mujoco_ros2_control_node's real set_free_joint_state service, instead of
M6's repeating random relocation.

Reuses controller_manager_m5.yaml and mujoco_ros2_control_plugins_m6.yaml
UNCHANGED - M7 adds no new controller type/parameter (same deviation
rationale as M6 itself: a byte-for-byte duplicate config file with zero real
differences isn't worth a new milestone number) and the free-joint body names
("x2", "attractant_target") are identical to M6's, so
mujoco_ros2_control_plugins_m6.yaml's body_names list already covers M7's
scene with no changes needed. Only the scene file itself
(drone_scene_m7.xml, selected via drone.urdf.xacro's existing `mujoco_model`
arg - no xacro changes needed, that arg was already made generic by M6) and
the orchestration script are new. M1-M6's own launch/config/scene files are
completely untouched by this milestone.

See NOTES.md's "## M7" section for the room/opening geometry, the
beacon-sequencing mechanism, and real per-leg trajectory/timing verification
numbers.

Usage (inside the container):
    ros2 launch fly_simulation m7_house_flight.launch.py
    ros2 launch fly_simulation m7_house_flight.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_controllers
    ros2 topic echo /drone/free_joint_states
    # house_navigator.py's own log lines (via its launch stdout) print each
    # ADVANCING/ARRIVED event with the real trigger distance and per-leg
    # timing.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    controller_pkg_share = FindPackageShare("fly_controller")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([controller_pkg_share, "urdf", "drone.urdf.xacro"]),
            " headless:=",
            LaunchConfiguration("headless"),
            " sim_speed_factor:=",
            LaunchConfiguration("sim_speed_factor"),
            # M7: the only difference from m6_moving_target.launch.py's xacro
            # invocation - loads the 3-room house scene (see module docstring
            # above and drone_scene_m7.xml's own header comment).
            " mujoco_model:=drone_scene_m7.xml",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Reused unchanged from M5 - see module docstring for why M7 doesn't need
    # its own controller_manager config variant (same reasoning as M6).
    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager_m5.yaml"]
    )
    # Reused unchanged from M6 - body_names ("x2", "attractant_target") are
    # identical under drone_scene_m7.xml, see module docstring.
    mujoco_plugins_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "mujoco_ros2_control_plugins_m6.yaml"]
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            output="both",
            emulate_tty=True,
            parameters=[
                robot_description,
                {"use_sim_time": True},
                ParameterFile(controller_manager_config),
                ParameterFile(mujoco_plugins_config),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")]
                if os.environ.get("ROS_DISTRO") == "humble"
                else []
            ),
            on_exit=Shutdown(),
        ),
    ]

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_sensor_broadcaster", "--param-file", controller_manager_config],
            output="both",
        )
    )

    # Same activation-race precaution as M5/M6 - all four fly-brain
    # controllers spawned together in ONE spawner invocation with
    # --activate-as-group.
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "optic_flow_controller",
                "phototaxis_controller",
                "haltere_reflex_controller",
                "descending_fusion_controller",
                "--activate-as-group",
                "--param-file",
                controller_manager_config,
            ],
            output="both",
        )
    )

    # M7: the fixed-sequence beacon-navigation node - see
    # scripts/house_navigator.py's own module docstring for what it does and
    # why it's a standalone fly_simulation script rather than a fly_brain
    # controller.
    nodes.append(
        Node(
            package="fly_simulation",
            executable="house_navigator.py",
            output="both",
            parameters=[{"use_sim_time": True}],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run MuJoCo's Simulate app without its own GLFW window "
                "(EGL/OSMesa headless rendering is used for the camera regardless).",
            ),
            DeclareLaunchArgument(
                "sim_speed_factor",
                default_value="1.0",
                description="MuJoCo Simulate App speed scaling; <0 uses the App window's own setting.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
