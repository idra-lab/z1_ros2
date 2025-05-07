# Z1 MoveIt! package

This package contains a basic MoveIt configuration that can be directly used for moving the Z1 manipulator.

To lunch the simulation environment with Moveit configured it is simply necessary to call:
```
ros2 launch z1_moveit z1_moveit.launch.py
```

As for the _standard_ bringup file, the `sim_ignition` flag can be disabled to enable the connection with the real robot:
```
ros2 launch z1_moveit z1_moveit.launch.py sim_ignition:=false
```
Other launch arguments include the `rviz` activation flag and its `rviz_config`, the `starting_controller` that can be used by MoveIt! (make sure that it is a valid one!) taken from a `controller_config`.
