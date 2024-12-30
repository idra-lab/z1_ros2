# Z1 Hardware Interface

This package is an attempt to create the hardware interface for the
[Z1 manipulator](https://shop.unitree.com/products/unitree-z1) from Unitree.
The idea is to use their [SDK](https://github.com/unitreerobotics/z1_sdk) to create a
proper hardware interface for the [ROS2 control](https://control.ros.org/master/index.html)
library.

## Robot Description

The repository contains also the URDF file of the Z1 manipulator that has been adapted
to properly work in the ROS2 framework. Source of the original URDF files, as well as
of the meshes, can be found [here](https://github.com/unitreerobotics/unitree_ros/tree/master).

Such files have been however modified to work with the
[ROS2 control](https://control.ros.org/master/index.html) hardware interface.
URDFs have been modified using as reference the official
[ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
package from Universal Robots, which has been extensively tested in our lab. and its
effectiveness is proven.

Furthermore, [launch](launch/) files are provided to ease the integration in other
projects.


## SDK Integration

This repository contains 2 external dependencies provided by Unitree: the
[Z1 controller](https://github.com/unitreerobotics/z1_controller)
and the [Z1 SDK](https://github.com/unitreerobotics/z1_sdk). The former is the one that's
actually used to communicate with the robot and performs the actual control, while the
latter provides the public interface to externally control the robot.

In this case, the [CMakeLists.txt](./CMakeLists.txt) builds the `z1_ctrl` application
(from the Z1 controller package) while building the whole ROS2 workspace; the SDK is
instead imported as dependency for the hardware interface (for both header file inclusion
and dynamic library linking).
