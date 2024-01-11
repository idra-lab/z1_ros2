from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os,time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
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

    
    # os.system("ros2 topic pub /stopSimulation std_msgs/msg/Bool '{data: true}' --once")
    # time.sleep(0.5)
    # os.system("ros2 topic pub /startSimulation std_msgs/msg/Bool '{data: true}' --once")
    # time.sleep(0.5)

    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")
    with_gripper = LaunchConfiguration("with_gripper")
    rviz = LaunchConfiguration("rviz")
    available_controllers = LaunchConfiguration("available_controllers")

    use_sim_time = True

    # Conditions that tells wether the robot is simulated or not
    # For now it is easy, since only ignition is supported
    # In any case, the following reference could be useful:
    # https://answers.ros.org/question/394181/multiple-conditions-for-ifcondition-in-ros2-launch-script/

    robot_description_content = xacro.process(
        xacro_file.perform(context),
        mappings={
            "name": robot_name.perform(context),
            "prefix": "",
            "with_gripper": with_gripper.perform(context),
            "simulation_controllers": available_controllers.perform(context),
            "sim_ignition": "false",
            "coppelia": "true",
        }
    )
    robot_description = {"robot_description": robot_description_content}

    with open("z1.urdf", "w") as f:
        f.write(robot_description_content)

    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {
            "use_sim_time": use_sim_time,
        }],
    )
    
    
    print("available_controllers", available_controllers.perform(context))
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, available_controllers, {
                "use_sim_time": use_sim_time
            }
        ],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ],
        #condition=is_real,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{
            "use_sim_time": use_sim_time,
            "set_state": "active",
        }],
        condition=IfCondition(rviz),
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

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        # joint_state_broadcaster_spawner,
        # delay_rviz,
    ]
    return nodes_to_start



def generate_launch_description():
    package_name = "z1_description"
    declared_arguments = []

    pkg_prefix_dir = get_package_share_path(package_name)
    


    # --- Launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            default_value=os.path.join(
                pkg_prefix_dir, "urdf", "z1.urdf.xacro"
            ),
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
            "available_controllers",
            default_value=os.path.join(
                get_package_share_path("z1_coppelia_hw"), "config", "z1_coppelia.yaml"
            ),
            description=
            "Path to the controllers.yaml file that can be loaded by the robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper",
            default_value="true",
            description="Use the default gripper?"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                pkg_prefix_dir, "rviz", "z1.rviz"
            ),
            description="Path to RViz configuration file"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )





