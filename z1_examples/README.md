# Z1 Examples

This package contains miscellaneous codes and scripts to test the Z1 manipulator.

### `waypoint_test.py`

[This executable node](z1_examples/waypoint_test.py) assumes that the robot is configured with a [`joint_trajectory_controller`](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) that uses the position interface.
By calling the executable
``` 
ros2 run z1_examples waypoint_test.py
```
perform some motion to pass through some joint configuration which are specified in the script as numpy arrays.

To properly run the demo, and not having to worry about the _bringup_ of the robot (wether real or in simulation), you may rely on the corresponding launch file which does everything:
```
ros2 launch z1_examples waypoint_test.launch.py sim_ignition:=true
```
By setting `sim_ignition` to false, the same motion will be executed on the real robot and not on the simulator!
