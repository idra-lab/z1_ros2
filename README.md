# Unitree Z1 Hardware Interface

This package is an attempt to create the hardware interface for the [Z1 manipulator](https://shop.unitree.com/products/unitree-z1)
from Unitree.
The idea is to use their [sdk](https://github.com/unitreerobotics/z1_sdk) to create a
proper hardware interface for the [ROS2 control](https://control.ros.org/master/index.html)
library.

The repository contains also the urdf file of the Z1 manipulator that has been adapted
to properly work in the ROS2 framework. Source of the original urdf files, as well as
of the meshes, can be found [here](https://github.com/unitreerobotics/unitree_ros/tree/master).

**Note:** as of last update, also the original launch and configuration files available
for ROS have been uploaded here, but their porting is still work in progress.
The urdf files are also on development.
