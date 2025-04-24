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


## ROS2 packages

This repository contains different sub-packages:

- [`z1_description`](z1_description/README.md): contains the URDFs for the Z1 robot, as well as its meshes;
- [`z1_bringup`](z1_bringup/README.md): contains configuration and launch files for the Z1 manipulator;
- [`z1_hardware_interface`](z1_hardware_interface/README.md): provides the [ROS2 control](https://control.ros.org/rolling/index.html) hardware interface for the Z1 manipulator;

## Testing the robot

To test the robot in the simulation environment, you can simply call the launch file provided by the `z1_bringup` package as follows:
```
ros2 launch z1_bringup z1.launch.py
```
The default option of `sim_ignition:=true` starts, by default, the ignition simulator. 
But switching to the communication with the real robot is easy, as it is simply necessary to set such variable to `false` as follows:
```
ros2 launch z1_bringup z1.launch.py sim_ignition:=false
```

More details on all launch file parameters can be found in the `z1_bringup` package [README](z1_bringup/README.md).
If you plan connecting you application to the physical hardware, make sure also to check out the `z1_hardware_interface` package [README](z1_hardware_interface/README.md).


## Contributing

Everyone is welcome to contribute to this repository. 

If you want to improve something, or have some particular request, please first open an issue to disclose your idea with everyone.

As general rule, please develop your feature/bug-fix on a new branch, and create a pull request targeting the **development branch** (`devel`).
There we will make sure that the change is working as expected, and will update the reference of the `main` branch accordingly, to guarantee the stability of such branch.
