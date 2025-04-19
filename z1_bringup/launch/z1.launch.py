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



def launch_setup(context, *args, **kwargs):

    nodes_to_start = list()

    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")
    with_gripper = LaunchConfiguration("with_gripper")
    rviz = LaunchConfiguration("rviz")
    controller_config = LaunchConfiguration("controller_config")
    starting_controller = LaunchConfiguration("starting_controller")
    sim_ignition = LaunchConfiguration("sim_ignition")

    use_sim_time = (sim_ignition.perform(context) == "true")

    # Conditions that tells wether the robot is simulated or not
    # For now it is easy, since only ignition is supported
    # In any case, the following reference could be useful:
    # https://answers.ros.org/question/394181/multiple-conditions-for-ifcondition-in-ros2-launch-script/
    is_simulation = IfCondition(sim_ignition)
    is_real = UnlessCondition(sim_ignition)

    #   ____
    #  / ___|___  _ __ ___  _ __ ___   ___  _ __  ___
    # | |   / _ \| '_ ` _ \| '_ ` _ \ / _ \| '_ \/ __|
    # | |__| (_) | | | | | | | | | | | (_) | | | \__ \
    #  \____\___/|_| |_| |_|_| |_| |_|\___/|_| |_|___/
    #
    robot_description_content = xacro.process(
        xacro_file.perform(context),
        mappings={
            "name": robot_name.perform(context),
            "prefix": "",
            "with_gripper": with_gripper.perform(context),
            "controllers": controller_config.perform(context),
            "sim_ignition": sim_ignition.perform(context),
        }
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": use_sim_time,
        }],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
    )

    starting_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[starting_controller.perform(context), "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(get_package_share_path("z1_description"), "rviz", "z1.rviz")
        ],
        condition=IfCondition(rviz),
    )

    rviz_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start += [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        starting_controller_spawner,
        rviz_delayed,
    ]

    #  ____            _   ____       _           _
    # |  _ \ ___  __ _| | |  _ \ ___ | |__   ___ | |_
    # | |_) / _ \/ _` | | | |_) / _ \| '_ \ / _ \| __|
    # |  _ <  __/ (_| | | |  _ < (_) | |_) | (_) | |_
    # |_| \_\___|\__,_|_| |_| \_\___/|_.__/ \___/ \__|
    #
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, controller_config, {
                "use_sim_time": use_sim_time
            }
        ],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ],
        condition=is_real,
    )

    z1_controller_path = os.path.join(
        get_package_share_path("z1_hardware_interface"), "controller"
    )
    z1_controller_executable = os.path.join(z1_controller_path, "z1_ctrl")
    z1_controller_process = ExecuteProcess(
        cmd=[z1_controller_executable],
        cwd=z1_controller_path,
        condition=is_real,
        output="screen",
    )

    nodes_to_start += [
        controller_manager_node,
        z1_controller_process,
    ]

    #  ___            _ _   _
    # |_ _|__ _ _ __ (_) |_(_) ___  _ __
    #  | |/ _` | '_ \| | __| |/ _ \| '_ \
    #  | | (_| | | | | | |_| | (_) | | | |
    # |___\__, |_| |_|_|\__|_|\___/|_| |_|
    #     |___/
    ignition_simulator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ], ),
        launch_arguments={
            "gz_args": " -r -v 1 empty.sdf",
        }.items(),
        condition=IfCondition(sim_ignition),
    )

    ignition_spawn_z1_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "/robot_description",
        ],
        condition=IfCondition(sim_ignition),
    )

    nodes_to_start += [
        ignition_simulator_node,
        ignition_spawn_z1_node,
    ]
    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    rviz_config_default = os.path.join(
        get_package_prefix("z1_bringup"), "rviz", "z1.rviz"
    )
    xacro_file_default = os.path.join(
        get_package_share_path("z1_description"), "urdf", "z1.urdf.xacro"
    )
    controller_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "config", "z1_controllers.yaml"
    )

    # --- Setup environment variables
    MDL_ENV_VAR = "IGN_GAZEBO_RESOURCE_PATH"
    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + os.path.join(
            get_package_prefix("z1_description"), "share"
        )
    else:
        os.environ[MDL_ENV_VAR] = os.path.join(
            get_package_prefix("z1_description"), "share"
        )

    LIB_ENV_VAR = "IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
    if LIB_ENV_VAR in os.environ:
        os.environ[LIB_ENV_VAR] += ":/opt/ros/humble/lib"
    else:
        os.environ[LIB_ENV_VAR] = "/opt/ros/humble/lib"

    # --- Launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value=xacro_file_default,
            description="Path to xacro file of the Z1 manipulator"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name", default_value="z1", description="Name of the robot"
        )
    )

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
            default_value="torque_controller",
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
