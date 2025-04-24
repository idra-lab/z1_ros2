# Z1 Hardware Interface

This package contains the hardware interface for the Unitree Z1 robot by leveraging the Unitree provided packages:

- [`z1_controller`](https://github.com/unitreerobotics/z1_controller):
  for the direct communication with the robot at a lower level;
- [`z1_sdk`](https://github.com/unitreerobotics/z1_sdk): 
  to access `z1_controller` from the ROS2 hardware interface and send commands to the manipulator.

In particular, the hardware interface is capable of exposing all command interfaces, namely: `position`, `velocity`, and `effort` (torque).

## First startup and network configuration

Prior the starting of the communication with the robot though ROS2, make sure you followed the [official instructions](https://support.unitree.com/home/en/Z1_developer/poweron) before powering the robot.

There, they also show how to correctly setup the local network to ensure the communication with the robot that, by default, has IP address `192.168.123.110`. 
In short, make sure the network port where the robot is connected into is configured with manual IPv4 address and the following parameters:

- address: `192.168.123.235`;
- netmask: `255.255.255.0`;
- gateway: `192.168.123.1`.

## `z1_controller`

This package builds the official `z1_controller` package, but also install accordingly the executables and the configuration files.
For simplicity and ease of modification, the `z1_controller`s [configuration folder](https://github.com/unitreerobotics/z1_controller/tree/master/config) has been copied at [`config/`](./config).
You may refer to the [official website documentation](https://support.unitree.com/home/en/Z1_developer/sdk_intro) to correctly configure the [`config.xml`](./config/config.xml) file, even though the standard values are good defaults.

## Hardware interface

This package implementes a [ROS2 control hardware interface](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) by using the `z1_sdk` library to send low-level command to the robot and read its state.
