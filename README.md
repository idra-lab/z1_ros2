# Unitree Z1 ROS2 package

This is a community-driven package that enable the [Z1 Manipulator](https://shop.unitree.com/products/unitree-z1) from [Unitree](https://www.unitree.com/) to work in ROS2.


## Installation

To use this package in ROS2, first clone this repository in a ROS2 workspace, e.g.:
``` bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/idra-lab/z1_ros2.git
```

At the current stage, we rely on [gitmodules](https://git-scm.com/docs/gitmodules) to fetch external dependencies for the project, particularly to build the hardware interface; we plan to remove such dependency, but in the meantime make sure to fetch them by calling the proper command:
``` bash
cd ~/ros2_ws/src/z1_ros2
git submodule init
```

Finally, you shall be able to build the workspace:
``` bash
cd ~/ros2_ws
colcon build
```


## ROS2 packages

This repository contains different sub-packages:

- `z1_description`: contains the URDFs for the Z1 robot, as well as its meshes;
- `z1_bringup`: contains configuration and launch files for the Z1 manipulator;
- `z1_hw_interface`: provides the [ROS2 control](https://control.ros.org/rolling/index.html) hardware interface for the Z1 manipulator.


## Testing the robot

To test the robot in the simulation environment, you can directly call the command
```
ros2 launch z1_bringup z1.launch.py sim_ignition:=true
```
More details on how to launch the robots can be found in the `z1_bringup` package [README](z1_bringup/README.md).
