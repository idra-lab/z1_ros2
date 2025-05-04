# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch



def launch_setup(context, *args, **kwargs):

    sim_ignition = LaunchConfiguration("sim_ignition").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    starting_controller = LaunchConfiguration("starting_controller").perform(context)

    bringup_file = os.path.join(
        get_package_share_path("z1_bringup"), "launch", "z1.launch.py"
    )
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_file),
        launch_arguments={
            "sim_ignition": sim_ignition,
            "rviz": "false",
            "starting_controller": starting_controller,
        }.items(),
    )

    # move_group.launch.file but with correct setting of "use_sim_time"
    moveit_config = MoveItConfigsBuilder("z1_description", package_name="z1_moveit")
    moveit_config = moveit_config.to_moveit_configs()
    moveit_config.trajectory_execution["use_sim_time"] = (sim_ignition == "true")

    rviz_launch_file = os.path.join(
        get_package_share_path("z1_moveit"), "launch", "moveit_rviz.launch.py"
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        condition=IfCondition(rviz),
    )

    return [
        bringup_launch,
        generate_move_group_launch(moveit_config),
        rviz_launch,
    ]



def generate_launch_description():
    declared_arguments = []

    rviz_config_default = os.path.join(
        get_package_share_path("z1_moveit"), "config", "moveit.rviz"
    )
    xacro_file_default = os.path.join(
        get_package_share_path("z1_description"), "urdf", "z1.urdf.xacro"
    )
    controller_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "config", "z1_controllers.yaml"
    )

    # --- Launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value=controller_config_default,
            description=
            "Path to the controllers.yaml file that can be loaded by the robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper", default_value="true", description="Use the gripper?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "starting_controller",
            default_value="joint_trajectory_controller",
            description="Name of the controller to be started"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Launch simulation in Ignition Gazebo?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_default,
            description="Path to RViz configuration file"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
