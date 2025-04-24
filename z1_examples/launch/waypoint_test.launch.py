from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()
    sim_ignition = LaunchConfiguration("sim_ignition")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("z1_bringup"), "/launch/z1.launch.py"
        ], ),
        launch_arguments={
            "sim_ingition": sim_ignition,
            "starting_controller": "joint_trajectory_controller",
        }.items(),
    )

    # Delay the call of the waypoint publication by 6 seconds
    waypoint_executable = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="z1_examples",
                executable="waypoint_test.py",
                output="screen",
            ),
        ],
    )

    nodes_to_start += [
        bringup_launch,
        waypoint_executable,
    ]
    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name", default_value="z1", description="Name of the robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Launch simulation in Ignition Gazebo?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
