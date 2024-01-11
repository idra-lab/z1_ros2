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

#include "z1_coppelia_hw/HWInterface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std;
namespace z1_coppelia_hw
{

  sensor_msgs::msg::JointState current_joint_state;
  // HardwareComms Methods
  Robot_Controller::Robot_Controller() : Node("robot_controller")
  {
    // Lamda callback for stateSub_
    auto stateCB =
        [this](const sensor_msgs::msg::JointState::SharedPtr state) -> void
    {
      for (int i = 0; i < 6; i++)
      {
        current_joint_state.position[i] = state->position[i];
        current_joint_state.velocity[i] = state->velocity[i];
      }
    };
    command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/coppelia_set_joints", 0);
    subscription = this->create_subscription<sensor_msgs::msg::JointState>("/coppelia_joint_states", 0, stateCB);
  }
  void Robot_Controller::SendCmd(std::vector<double> cmd)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = cmd;
    command_pub_->publish(msg);
  }

  // ##############################################################################
  // HW INTERFACE IMPLEMENTATION
  hardware_interface::CallbackReturn Z1EffortFakeHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    // initializing variable to save the current joint state
    current_joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    current_joint_state.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_joint_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_joint_state.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // initializing publisher for set joint in Coppelia

    // launching listener to Coppelia topic for joint states
    comms = std::make_shared<Robot_Controller>();
    executor_.add_node(comms);
    std::thread([this]()
                { executor_.spin(); })
        .detach();

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    // END: This part here is for exemplary purposes - Please do not copy to your production code
    hw_pos_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_vel_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_eff_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // RRBotSystemPositionOnly has exactly two states and command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Z1EffortFakeHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Z1EffortFakeHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // if (joint.state_interfaces.size() != 1)
      // {
      //   RCLCPP_FATAL(
      //       rclcpp::get_logger("Z1EffortFakeHardware"),
      //       "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
      //       joint.state_interfaces.size());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Z1EffortFakeHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("Z1EffortFakeHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Z1EffortFakeHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("Z1EffortFakeHardware"), "Configuring ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("Z1EffortFakeHardware"), "%.1f seconds left...",
          hw_start_sec_ - i);
    }
    float initial_conf[]={0.0,2.0,0.0,-90.0*3.14159/180.0,0.0,90.0*3.14159/180.0};
    for (uint i = 0; i < hw_pos_.size(); i++)
    {
      hw_pos_[i] = initial_conf[i];
      hw_vel_[i] = 0.0;
      hw_commands_[i] = 0.0;
    }

    RCLCPP_INFO(rclcpp::get_logger("Z1EffortFakeHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // EXPOSING STATE
  std::vector<hardware_interface::StateInterface>
  Z1EffortFakeHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_pos_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_eff_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Z1EffortFakeHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn Z1EffortFakeHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("Z1EffortFakeHardware"), "Activating ...please wait...");

    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("Z1EffortFakeHardware"), "%.1f seconds left...",
          hw_start_sec_ - i);
    }
    // for (uint i = 0; i < hw_pos_.size(); i++)
    // {
    //   hw_commands_[i] = hw_pos_[i];
    // }

    RCLCPP_INFO(rclcpp::get_logger("Z1EffortFakeHardware"), "HW Interface successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn Z1EffortFakeHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(
        rclcpp::get_logger("Z1EffortFakeHardware"), "Deactivating ...please wait...");

    for (int i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("Z1EffortFakeHardware"), "%.1f seconds left...",
          hw_stop_sec_ - i);
    }

    RCLCPP_INFO(rclcpp::get_logger("Z1EffortFakeHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Z1EffortFakeHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    for (uint i = 0; i < hw_pos_.size(); i++)
    {
      hw_pos_[i] = current_joint_state.position[i];
      hw_vel_[i] = current_joint_state.velocity[i];
      // RCLCPP_INFO(rclcpp::get_logger("Z1EffortFakeHardware"), "Reading");
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("Z1EffortFakeHardware"), "pos " << current_joint_state.position[i]);
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("Z1EffortFakeHardware"), "vel " << current_joint_state.velocity[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Z1EffortFakeHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Robot_Controller::SendCmd(hw_commands_);
    comms->SendCmd(hw_commands_);
    return hardware_interface::return_type::OK;
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    z1_coppelia_hw::Z1EffortFakeHardware, hardware_interface::SystemInterface)
