"""
M0: bring up mujoco_ros2_control with the drone model loaded, plus
robot_state_publisher and the IMU broadcaster. Proves the Docker + ROS2 +
mujoco_ros2_control + drone-model plumbing works end to end. No fly_brain or
baseline PID controller is spawned yet (that's M1+).

Usage (inside the container):
    ros2 launch fly_simulation m0_bringup.launch.py
    ros2 launch fly_simulation m0_bringup.launch.py headless:=false   # needs GUI passthrough

Verify in another shell (inside the container):
    ros2 control list_hardware_interfaces
    ros2 control list_controllers
    ros2 topic echo /drone/free_joint_states
    ros2 topic echo /imu_sensor_broadcaster/imu
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

    controller_manager_config = PathJoinSubstitution(
        [controller_pkg_share, "config", "controller_manager.yaml"]
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
        # mujoco_ros2_control ships its own ros2_control_node (same executable
        # name/params as upstream controller_manager's, with compatibility
        # patches - see mujoco_ros2_control/README.md, "Hardware Interface
        # Setup"). Using the upstream controller_manager's node here would not
        # load the MuJoCo simulation.
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

    for controller in ["imu_sensor_broadcaster"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controller_manager_config],
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
