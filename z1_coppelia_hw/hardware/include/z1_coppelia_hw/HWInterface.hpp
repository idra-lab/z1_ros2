// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "z1_coppelia_hw/visibility_control.h"

namespace z1_coppelia_hw
{

class Robot_Controller : public rclcpp::Node
{
  public:
    Robot_Controller();
    void SendCmd(std::vector<double> cmd);
  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
};


class Z1EffortFakeHardware : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(Z1EffortFakeHardware);

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::shared_ptr<Robot_Controller> comms;
  
private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_pos_;
  std::vector<double> hw_vel_;
  std::vector<double> hw_eff_;
};

}  // namespace ros2_control_demo_example_1

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
