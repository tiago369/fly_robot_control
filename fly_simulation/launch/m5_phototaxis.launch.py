"""
M5: bring up mujoco_ros2_control with the drone model loaded (now including
the bright orange/red "attractant_target" sphere 3m ahead of spawn, see
fly_controller/models/skydio_x2/drone_scene.xml), plus robot_state_publisher,
the IMU broadcaster, and the full fly-brain chain: fly_brain's
OpticFlowController (M3, unchanged) + fly_brain's new PhototaxisController
(M5, color-target perception) -> fly_brain's HaltereReflexController (M2's
reflex loop, unchanged) -> fly_brain's DescendingFusionController (M4's
closed-loop altitude hold, now also chaining PhototaxisController's
target_visible/bearing/area_fraction reference interfaces into a real
turn-toward/approach task command - see NOTES.md's "## M5" section and
fly_brain/include/fly_brain/descending_fusion_controller.hpp).

Uses controller_manager_m5.yaml (a full, separate copy of
controller_manager_m4.yaml - see that file's own header comment for why not a
layered --param-file override) so M1-M4's launches are completely unaffected.

Usage (inside the container):
    ros2 launch fly_simulation m5_phototaxis.launch.py
    ros2 launch fly_simulation m5_phototaxis.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_controllers
    ros2 control list_hardware_interfaces
    ros2 topic echo /drone/free_joint_states
    ros2 topic echo /phototaxis_controller/... (via list_hardware_interfaces' claimed values,
        or subscribe to the RCLCPP_INFO_THROTTLE telemetry in the node's own log output)

Target-seeking verification: subscribe to /drone/free_joint_states (body
"x2"), compute horizontal distance to the known attractant position (3.0, 0.0)
(see drone_scene.xml's "attractant_target" body pos) over time, and confirm it
measurably and substantially decreases from spawn to a converged/hovering-near
-target end state - see NOTES.md's "## M5" section for the real numbers from
the last verification run.
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

    # M5-specific config (see module docstring for why this isn't the shared
    # controller_manager.yaml or M3/M4's own config files).
    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager_m5.yaml"]
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

    # All four fly-brain controllers MUST be spawned together, in ONE spawner
    # invocation with --activate-as-group - the same chained-controller
    # activation race M3/M4 documented (see NOTES.md's "## M3" section: each
    # spawner process independently calls activate as soon as ITS OWN
    # controller loads/configures, with no guarantee an upstream controller's
    # exported reference interfaces are registered as "available" yet).
    # Order within the list does not matter (the flag resolves the real
    # dependency order itself) - listed here in chain order (leaves first)
    # purely for readability.
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
