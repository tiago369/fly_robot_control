"""
M6: same full fly-brain chain as M5 (optic_flow_controller +
phototaxis_controller -> haltere_reflex_controller ->
descending_fusion_controller, all spawned together via one
--activate-as-group call - see m5_phototaxis.launch.py's own docstring and
NOTES.md's "## M3" section for why that's non-negotiable), plus a repeating
"chase" behavior: once the drone gets close enough to the attractant target,
scripts/moving_target.py teleports it to a new nearby location so the fly
keeps re-acquiring and pursuing a target that keeps relocating, instead of a
one-shot approach-and-stop.

This requires the target to be a real MuJoCo free-joint body (M5's
drone_scene.xml has it welded to the world with no joint at all -
set_free_joint_state rejects that outright). Rather than editing
drone_scene.xml / mujoco_ros2_control_plugins.yaml in place (which would
change M1-M5's shared model/config), M6 uses its own variants:
  - models/skydio_x2/drone_scene_m6.xml (attractant_target gets a
    <freejoint/> + gravcomp="1", see that file's own header comment) -
    selected via drone.urdf.xacro's new `mujoco_model` arg (default
    "drone_scene.xml", so every other launch file is unaffected).
  - config/mujoco_ros2_control_plugins_m6.yaml (free_joint_state_publisher's
    body_names also lists "attractant_target" - the shared
    mujoco_ros2_control_plugins.yaml can't list it, since that name isn't a
    free-joint body under M1-M5's own scene file and the plugin hard-fails
    init() if a listed body_names entry doesn't resolve).
controller_manager_m5.yaml is reused UNCHANGED - M6 adds no new controller
types or parameters, only a new node watching existing topics/services, so a
full-copy variant would just be a duplicate file (see NOTES.md's "## M6"
section for why this deviates from M3/M4/M5's per-milestone full-copy
config-file precedent).

See NOTES.md's "## M6" section for the relocation thresholds/geometry and
real multi-cycle verification numbers.

Usage (inside the container):
    ros2 launch fly_simulation m6_moving_target.launch.py
    ros2 launch fly_simulation m6_moving_target.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_controllers
    ros2 topic echo /drone/free_joint_states
    # moving_target.py's own RCLCPP-style log lines (via its launch stdout)
    # print each RELOCATING event with the real trigger distance and the
    # target's before/after position.
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
            # M6: the only difference from m5_phototaxis.launch.py's xacro
            # invocation - loads the scene variant where attractant_target is
            # a real free-joint body (see module docstring above).
            " mujoco_model:=drone_scene_m6.xml",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Reused unchanged from M5 - see module docstring for why M6 doesn't need
    # its own controller_manager config variant.
    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager_m5.yaml"]
    )
    # M6-specific: body_names also covers "attractant_target" - see module
    # docstring.
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

    # Same activation-race precaution as M5 - all four fly-brain controllers
    # spawned together in ONE spawner invocation with --activate-as-group.
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

    # M6: the target-relocation node itself - see scripts/moving_target.py's
    # own module docstring for what it does and why it's a standalone
    # fly_simulation script rather than a fly_brain controller.
    nodes.append(
        Node(
            package="fly_simulation",
            executable="moving_target.py",
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
