#ifndef UNITREE_Z1_HW_INTERFACE_HPP__
#define UNITREE_Z1_HW_INTERFACE_HPP__

#include <cstddef>
#include <unitree_arm_sdk/control/unitreeArm.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

/*
 * Functions that needs to be implemented:
 *  - Lifecycle:
 *      on_configure
 *      on_shutdown
 *      on_activate
 *      on_deactivate
 *      on_error
 *  - HW interface:
 *      on_init
 *      export_state_interfaces
 *      export_command_interfaces
 *      prepare_command_mode_switch (opt)
 *      perform_command_mode_switch (opt)
 *      read
 *      write
 */

namespace unitree {
    namespace z1 {
        namespace hw_interface {

            class UnitreeZ1HWInterface : public hardware_interface::SystemInterface {
            public:
                RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeZ1HWInterface)

                UnitreeZ1HWInterface();
                ~UnitreeZ1HWInterface();

                hardware_interface::CallbackReturn on_configure(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_cleanup(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_shutdown(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_activate(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_deactivate(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_error(
                        const rclcpp_lifecycle::State& prev_state) override;

                hardware_interface::CallbackReturn on_init(
                        const hardware_interface::HardwareInfo& hw_info) override;

                std::vector<hardware_interface::StateInterface>
                export_state_interfaces() override;

                std::vector<hardware_interface::CommandInterface>

                export_command_interfaces() override;

                hardware_interface::return_type read(
                        const rclcpp::Time&     time,
                        const rclcpp::Duration& period) override;

                hardware_interface::return_type write(
                        const rclcpp::Time&     time,
                        const rclcpp::Duration& period) override;


            private:
                bool with_gripper;
                std::size_t n_joints;
                void shutdown();

                std::vector<double> rob_q;
                std::vector<double> rob_dq;
                std::vector<double> rob_ddq;
                std::vector<double> rob_tau;
                std::vector<double> cmd_q;
                std::vector<double> cmd_dq;
                std::vector<double> cmd_tau;


            };


        }  // namespace hw_interface
    }      // namespace z1
}  // namespace unitree

#endif  // UNITREE_Z1_HW_INTERFACE_HPP__
