# Z1 Bringup package

This package contains default launch and configuration files to work with the Unitree Z1 manipulator.

The main entry-point for the robot is provided by the `z1.launch.py` launch file; to start the robot call

```
ros2 launch z1_bringup z1.launch.py
```


## Launch arguments
Launch arguments provided by `z1.launch.py` are:

- `xacro_file` (default: [`z1_description/urdf/z1.urdf.xacro`](../z1_description/urdf/z1.urdf.xacro)): path to the xacro file of the Z1 manipulator;
- `with_gripper` (default: `true`): loads the Z1 gripper;
- `controller_config` (default: [`config/z1_controllers.yaml`](./config/z1_controller.yaml)): full path to the _yaml_ file with the controller configuration;
- `starting_controller` (default: `torque_controller`): controller that get started by default;
- `sim_ignition` (default: `true`): loads the ignition simulation environment;
- `rviz` (default: `true`): open RViz2;
- `rviz_config` (defaults: [`rviz/z1.rviz`](./rviz/z1.rviz)): full path to the _yaml_ file with the rviz configuration;

