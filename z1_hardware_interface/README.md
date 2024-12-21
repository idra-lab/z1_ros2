# Z1 Hardware Interface

This package contains the hardware interface for the Unitree Z1 robot.

### 🚧⚠️ Warning ️⚠️🚧

The hardware interface is not yet finalised, thus it is **not functional**. Due to problem with the robot on our laboratory, we cannot guarantee the delivery of the final hardware interface.

## Working principle

This package internally fetches 2 unitree libraries:

- [`z1_controller`](https://github.com/unitreerobotics/z1_controller):
  this is used to directly communicate with the robot at a lower level;
- [`z1_sdk`](https://github.com/unitreerobotics/z1_sdk): 
  provides a simplified interface to communicate with the robot and enables to send commands.

Currently, the hardare interface spawns two entities: a `z1_controller` to access the robot, and a `z1_sdk` component that directly interfaces with the robot.

As future work, we might envision to drop the dependency on `z1_sdk` if we can find a more effective way to set the control inputs to the robot via the APIs provided by `z1_controller`.