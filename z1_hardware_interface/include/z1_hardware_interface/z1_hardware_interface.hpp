#ifndef UNITREE_Z1_HW_INTERFACE_HPP__
#define UNITREE_Z1_HW_INTERFACE_HPP__

#include <cstddef>
#include <unitree_arm_sdk/control/unitreeArm.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace unitree {
    namespace z1 {

        using Vector6d            = Eigen::Matrix<double, 6, 1>;
        using unitreeArmUniquePtr = std::unique_ptr<UNITREE_ARM::unitreeArm>;

        class Z1HwInterface : public hardware_interface::SystemInterface {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(Z1HwInterface)

            Z1HwInterface();
            ~Z1HwInterface();

            hardware_interface::CallbackReturn on_configure(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_cleanup(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_shutdown(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_activate(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_deactivate(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_error(
                    const rclcpp_lifecycle::State& prev_state
            ) override;

            hardware_interface::CallbackReturn on_init(
                    const hardware_interface::HardwareInfo& hw_info
            ) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces(
            ) override;

            std::vector<hardware_interface::CommandInterface>

            export_command_interfaces() override;

            hardware_interface::return_type read(
                    const rclcpp::Time& time, const rclcpp::Duration& period
            ) override;

            hardware_interface::return_type write(
                    const rclcpp::Time& time, const rclcpp::Duration& period
            ) override;

            hardware_interface::return_type perform_command_mode_switch(
                    const std::vector<std::string>& start_interfaces,
                    const std::vector<std::string>& stop_interfaces
            ) override;


        private:
            bool                with_gripper;
            unitreeArmUniquePtr arm;

            Vector6d cmd_q     = Vector6d::Zero();
            Vector6d cmd_dq    = Vector6d::Zero();
            Vector6d cmd_tau   = Vector6d::Zero();
            Vector6d state_q   = Vector6d::Zero();
            Vector6d state_dq  = Vector6d::Zero();
            Vector6d state_ddq = Vector6d::Zero();
            Vector6d state_tau = Vector6d::Zero();

            double gripper_cmd_q     = 0;
            double gripper_cmd_dq    = 0;
            double gripper_cmd_tau   = 0;
            double gripper_state_q   = 0;
            double gripper_state_dq  = 0;
            double gripper_state_ddq = 0;
            double gripper_state_tau = 0;

            void        shutdown();
            std::size_t n_joints() const;
        };


    }  // namespace z1
}  // namespace unitree

#endif  // UNITREE_Z1_HW_INTERFACE_HPP__
