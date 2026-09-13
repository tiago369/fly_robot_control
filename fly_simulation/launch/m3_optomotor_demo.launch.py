"""
M3: bring up mujoco_ros2_control with the drone model loaded, plus
robot_state_publisher, the IMU broadcaster, fly_brain's
HaltereReflexController (M2's reflex loop, unchanged) AND fly_brain's new
OpticFlowController, chained together: OpticFlowController subscribes to
/drone/camera/image_raw (the new body-fixed "nose_cam" - see NOTES.md's
"## M3" section for why "track" doesn't work for this), runs the Reichardt
EMD array on a coarse grayscale grid, and exports yaw_rate/roll_drift/
forward_drift/vertical_drift reference interfaces; HaltereReflexController
claims "optic_flow_controller/roll_drift" (NOT "yaw_rate", and NOT
"forward_drift" either despite the name - see NOTES.md's "## M3" section for
why) as an extra command interface and additively folds it into its effective
yaw-rate setpoint, so a genuine visually-sensed yaw rotation is fed back into
the reflex loop - a preview of M4's full descending-fusion chain, exercising
only the yaw/optomotor path end-to-end with real vision data.

Uses controller_manager_m3.yaml (a full, separate copy of
controller_manager.yaml with haltere_reflex_controller's 3 new
use_optic_flow_yaw_input/optic_flow_yaw_interface_name/optic_flow_yaw_gain
parameters set, plus the optic_flow_controller entry) rather than the shared
controller_manager.yaml, so M1/M2's launches are completely unaffected.

Usage (inside the container):
    ros2 launch fly_simulation m3_optomotor_demo.launch.py
    ros2 launch fly_simulation m3_optomotor_demo.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /drone/camera/image_raw --no-arr
    ros2 topic echo /imu_sensor_broadcaster/imu

Disturbance-recovery test (yaw axis, unlike M2's roll-axis one - see NOTES.md's
"## M3" section for the chosen magnitude/duration): once both controllers are
active, inject a torque disturbance via mujoco_ros2_control_plugins'
ExternalWrenchPlugin (same service used by M1/M2, shared config):
    ros2 service call /external_wrench/apply_wrench \\
      mujoco_ros2_control_msgs/srv/ApplyExternalWrench \\
      "{wrenches: {external_wrenches: [{wrench: {header: {frame_id: 'x2'}, \\
        wrench: {torque: {x: 0.0, y: 0.0, z: 0.15}}}, \\
        duration: {sec: 1, nanosec: 500000000}}]}}"

Magnitude chosen empirically (see NOTES.md's "## M3" section): 0.15 N*m / 1.5s
gives a peak yaw rate of ~1.9 rad/s decaying over several seconds - large
enough to be a clear, sustained disturbance (not a brief kick), small enough
to stay within the regime the ~11-15Hz camera / 32x32 EMD grid can actually
track without severe inter-frame aliasing (a much larger pulse - e.g. 0.6 N*m,
M2's roll magnitude - produces an 8 rad/s peak on this axis, i.e. tens of
degrees of rotation per camera frame, well outside what any EMD-style
correlator can resolve).
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

    # M3-specific config (see module docstring for why this isn't the shared
    # controller_manager.yaml).
    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager_m3.yaml"]
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

    # optic_flow_controller + haltere_reflex_controller MUST be spawned
    # together, in ONE spawner invocation with --activate-as-group - a real,
    # hard-won finding (see NOTES.md's "## M3" section): two SEPARATE spawner
    # Node actions (even listing optic_flow_controller first) race, because
    # each spawner process independently calls activate as soon as its own
    # controller loads/configures - there is no guarantee optic_flow_
    # controller's exported reference interfaces are registered as
    # "available" in the resource manager by the time haltere_reflex_
    # controller's OWN spawner process asks to activate it, and
    # controller_manager refuses activation outright when a claimed command
    # interface isn't available yet (actual error hit during development,
    # from an earlier iteration before the source interface was switched to
    # roll_drift: "Unable to activate controller 'haltere_reflex_controller'
    # since the command interface 'optic_flow_controller/forward_drift' is
    # not available") rather than waiting/retrying. --activate-as-group makes spawner resolve and
    # activate the whole dependency chain via one atomic switch_controller
    # call, which is what actually works.
    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "optic_flow_controller",
                "haltere_reflex_controller",
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
