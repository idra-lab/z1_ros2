# Unitree Z1 ROS2 package

This is a community-driven package that enable the [Z1 Manipulator](https://shop.unitree.com/products/unitree-z1) from [Unitree](https://www.unitree.com/) to work in ROS2.

[![Humble CI](https://github.com/idra-lab/z1_ros2/actions/workflows/humble.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/humble.yml)
[![Jazzy CI](https://github.com/idra-lab/z1_ros2/actions/workflows/jazzy.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/jazzy.yml) 
[![Rolling CI](https://github.com/idra-lab/z1_ros2/actions/workflows/rolling.yml/badge.svg)](https://github.com/idra-lab/z1_ros2/actions/workflows/rolling.yml)

## Quick Start

To use this package in ROS2, first clone this repository in a ROS2 workspace, e.g.:
``` bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/idra-lab/z1_ros2.git
```

All external dependencies can be installed with [`rosdep`](https://wiki.ros.org/rosdep):
``` bash
rosdep update
rosdep install --from-paths ~/ros2_ws/src --ignore-src
```
Make sure that you sourced the ROS2 global workspace (`source /opt/ros/humble/setup.bash`) and then  simply build the workspace as:
``` bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Finally, make sure to source also the built workspace (`source ~/ros2_ws/install/setup.bash`).


## ROS2 packages

This repository contains different sub-packages:

- [`z1_description`](z1_description/README.md): contains the URDFs for the Z1 robot, as well as its meshes;
- [`z1_bringup`](z1_bringup/README.md): contains configuration and launch files for the Z1 manipulator;
- [`z1_hardware_interface`](z1_hardware_interface/README.md): provides the [ROS2 control](https://control.ros.org/rolling/index.html) hardware interface for the Z1 manipulator;
- [`z1_moveit`](z1_moveit/README.md): [MoveIt!](https://moveit.ai/) integration for the Z1 manipulator;
- [`z1_examples`](z1_examples/README.md): contains some simple scripts to test and validate the functionalities of the robot;


For more information for each package, please refer to the corresponding `README`.


## Robot in action

To get started with the Z1 manipulator in the simulation environment, you may call
```
ros2 launch z1_bringup z1.launch.py starting_controller:=joint_trajectory_controller
```
This make sure to launch the robot with the [`joint_trajectory_controller`](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html), which provide a simple motion planning facility. 

To test the proper functionality, we can use the [`waypoint_test.py`](./z1_examples/z1_examples/waypoint_test.py) script to send a default plan as follows:
```
ros2 run z1_examples waypoint_test.py
```
The outcome of the simulation shall be the following:

![](/docs/resources/gazebo-waypoint-example.gif)

By using the same bringup launch file, it becomes really easy to transition from the simulation environment to the connection with the real robot. It is necessary to launch
```
ros2 launch z1_bringup z1.launch.py starting_controller:=joint_trajectory_controller sim_ignition:=false
```
alongside with the `waypoint_test.py` executable, and the hardware interface which connects to the real robot is loaded over the simulator interface, and the robot performs the same motion.

![](/docs/resources/robot-waypoint-example.gif)

**Note:** this example uses the `position` command interface for the robot (as specified in the [configuration file](z1_bringup/config/z1_controllers.yaml) for the `joint_trajectory_controller`), and you may have some trouble replicating the simulation on your machine.
As mentioned in [this issue](https://github.com/idra-lab/z1_ros2/issues/8), a temporary fix is to delete all appearences of the `effort` command interface from the [`z1.ros2_control.xacro`](./z1_description/urdf/z1.ros2_control.xacro). 
Note that this is a problem of the ROS2 control plugin for Ignition, and not some misconfiguration of this package; the connection with the hardware does not suffer this problem.
A better fix to deleting parts from the URDF is planned but not implemented yet.


## Contributing

Everyone is welcome to contribute to this repository. 

If you want to improve something, or have some particular request, please first open an issue to disclose your idea with everyone.

As general rule, please develop your feature/bug-fix on a new branch, and create a pull request targeting the **development branch** (`devel`).
There we will make sure that the change is working as expected, and will update the reference of the `main` branch accordingly, to guarantee the stability of such branch.
