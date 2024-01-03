from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    )
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
        PythonLaunchDescriptionSource
        )
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
    get_package_share_path,
    )



def launch_setup(context, *args, **kwargs):

    xacro_file = context.launch_configurations["xacro_file"]
    gripper = context.launch_configurations["gripper"]
    rviz = context.launch_configurations["rviz"]
    controllers = context.launch_configurations["controllers"]
    sim_ignition = context.launch_configurations["sim_ignition"]
    # use_sim_time = True


    print("Retrieved launch configurations")
    use_ignition = sim_ignition == "true"
    use_sim_time = use_ignition
    control_name = "IgnitionSystem" if use_ignition else "z1"


    print("PRocessing xacro")
    robot_description_content = xacro.process(
        xacro_file,
        mappings={
            "name": control_name,
            "prefix": "",
            "with_gripper": gripper,
            "simulation_controllers": controllers,
            "sim_ignition": sim_ignition,
            }
        )
    robot_description = {"robot_description": robot_description_content}
    with open("file.urdf", "w") as f:
        f.write(robot_description_content)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": use_sim_time,
            }],
        )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers, {
            "use_sim_time": use_sim_time
            }],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ],
        condition=UnlessCondition(sim_ignition),
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
            }]
        )

    joint_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time
            }]
        )

    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["motion_control_handle", "-c"," --stopped " "/controller_manager"],
        arguments=["motion_control_handle", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    controller_list = [
        joint_state_broadcaster_spawner,
        # joint_controller_node,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        ]

    print("Setup RViz")
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

    print("Setup ignition spawn entity")
    # Ignition nodes
    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "z1",
            "-topic",
            "robot_description",
        ],
        condition=IfCondition(sim_ignition),
    )



    # ign_gazebo_path = get_package_share_directory("ros_ign_gazebo")
    # ign_gazebo_launchfile = ign_gazebo_path + "/launch/ign_gazebo.launch.py"
    print("Setup ignition node launch")
    ignition_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"],
            ),
            launch_arguments={"ign_args": " -r -v 1 empty.sdf"}.items(),
            condition=IfCondition(sim_ignition),
            )

    print("Setup state broadcaster delay")
    delay_rviz = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
                ),
            )


    print("Setup controller delay")
    controller_delay = TimerAction(
            period=5.0,
            actions=[*controller_list],
            )


    print("Returning notes to start")
    notes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        controller_delay,
        delay_rviz,
        ignition_spawn_entity,
        ignition_node,
        ]
    return notes_to_start



def generate_launch_description():
    package_name = "unitree_z1_hw_interface"
    declared_arguments = []

    pkg_prefix_dir = get_package_prefix(package_name)

    # --- Setup environment variables
    MDL_ENV_VAR = "IGN_GAZEBO_RESOURCE_PATH"
    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + os.path.join(pkg_prefix_dir, "share")
    else:
        os.environ[MDL_ENV_VAR] = os.path.join(pkg_prefix_dir, "share")

    LIB_ENV_VAR = "IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
    if LIB_ENV_VAR in os.environ:
        os.environ[LIB_ENV_VAR] += ":/opt/ros/humble/lib"
    else:
        os.environ[LIB_ENV_VAR] = "/opt/ros/humble/lib"

    # LD_LIB_PATH = "LD_LIBRARY_PATH"
    # if LD_LIB_PATH in os.environ:
    #     os.environ[LD_LIB_PATH] += ":/opt/ros/humble/lib"
    # else:
    #     os.environ[LD_LIB_PATH] = "/opt/ros/humble/lib"

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
            "gripper", default_value="false", description="Using the default gripper"
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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
        )
