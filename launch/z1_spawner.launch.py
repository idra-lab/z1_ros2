import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    )
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
    get_package_share_path,
    )



def launch_setup(context, *args, **kwargs):

    xacro_file = context.launch_configurations["xacro_file"]
    rviz = context.launch_configurations["rviz"]
    controllers = context.launch_configurations["controllers"]
    sim_ignition = context.launch_configurations["sim_ignition"]

    robot_description_content = xacro.process(xacro_file, mappings={
        "prefix": "",
        "simulation_controllers": controllers,
        "sim_ignition": sim_ignition
        })
    robot_description = {"robot_description": robot_description_content}
    with open("file.urdf", "w") as f:
        f.write(robot_description_content)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {
                "use_sim_time": True,
            }
            ],
        )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_path("unitree_z1_hw_interface"), "rviz", "z1.rviz"
                )
            ],
        condition=IfCondition(rviz),
        )

    gui_joint_space_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        )

    notes_to_start = [
        robot_state_publisher_node,
        rviz_node,
        gui_joint_space_pub_node,
        ]
    return notes_to_start



def generate_launch_description():
    package_name = "unitree_z1_hw_interface"
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value=os.path.join(
                get_package_share_path(package_name), "urdf", "z1.urdf.xacro"
                ),
            description="Path to xacro file of the Z1 manipulator"
            )
        )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers",
            default_value=os.path.join(
                get_package_share_path(package_name), "config", "z1_controllers.yaml"
                ),
            description="path to the controllers.yaml file"
            )
        )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Launch simulation in Ignition Gazebo?"
            )
        )
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
        )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
        )
