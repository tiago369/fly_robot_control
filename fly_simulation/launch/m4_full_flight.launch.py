"""
M4: bring up mujoco_ros2_control with the drone model loaded, plus
robot_state_publisher, the IMU broadcaster, and the full fly-brain chain:
fly_brain's OpticFlowController (M3, unchanged) -> fly_brain's
HaltereReflexController (M2's reflex loop, unchanged, own M3 optic-flow
bolt-on disabled - see controller_manager_m4.yaml's own header comment) ->
fly_brain's new DescendingFusionController (M4), which claims
haltere_reflex_controller's roll/pitch/yaw_rate/thrust reference interfaces
directly and optic_flow_controller/roll_drift, runs a real closed-loop
altitude PID against /drone/free_joint_states, and writes the fused
setpoint every cycle - see NOTES.md's "## M4" section and
fly_brain/include/fly_brain/descending_fusion_controller.hpp.

Uses controller_manager_m4.yaml (a full, separate copy of
controller_manager_m3.yaml - see that file's own header comment for why not
a layered --param-file override) so M1/M2/M3's launches are completely
unaffected.

Usage (inside the container):
    ros2 launch fly_simulation m4_full_flight.launch.py
    ros2 launch fly_simulation m4_full_flight.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /drone/free_joint_states
    ros2 topic echo /imu_sensor_broadcaster/imu

Yaw disturbance-recovery test (identical to M3's - see NOTES.md's "## M3" and
"## M4" sections for the comparison methodology and numbers), fired once
descending_fusion_controller reports active:
    ros2 service call /external_wrench/apply_wrench \\
      mujoco_ros2_control_msgs/srv/ApplyExternalWrench \\
      "{wrenches: {external_wrenches: [{wrench: {header: {frame_id: 'x2'}, \\
        wrench: {torque: {x: 0.0, y: 0.0, z: 0.15}}}, \\
        duration: {sec: 1, nanosec: 500000000}}]}}"

Altitude-hold check: subscribe to /drone/free_joint_states and confirm z
stays within the same +-5cm band around z_setpoint (0.5m, see
controller_manager_m4.yaml's descending_fusion_controller.z_setpoint) that
M1's BaselinePidController held in its own verification (NOTES.md's "## M1"
section) - this is the concrete proof the new closed-loop altitude-hold
addition (replacing M2/M3's open-loop climb-compensated hack) actually works.
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
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # M4-specific config (see module docstring for why this isn't the shared
    # controller_manager.yaml or M3's controller_manager_m3.yaml).
    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager_m4.yaml"]
    )
    mujoco_plugins_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "mujoco_ros2_control_plugins.yaml"]
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

    # All three fly-brain controllers MUST be spawned together, in ONE
    # spawner invocation with --activate-as-group - the exact same activation
    # race M3 hit spawning just two chained controllers separately (see
    # NOTES.md's "## M3" section) applies with equal or greater force to a
    # three-controller chain: each spawner process independently calls
    # activate as soon as its own controller loads/configures, with no
    # guarantee upstream controllers' exported reference interfaces are
    # registered as "available" yet. --activate-as-group resolves and
    # activates the whole dependency chain via one atomic switch_controller
    # call. Order within the list does not matter for this (the flag
    # resolves the real dependency order itself) - listed here in
    # chain order (leaves first) purely for readability.
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "optic_flow_controller",
                "haltere_reflex_controller",
                "descending_fusion_controller",
                "--activate-as-group",
                "--param-file",
                controller_manager_config,
            ],
            output="both",
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
