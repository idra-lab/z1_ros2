#ifndef UNITREE_Z1_HW_INTERFACE_HPP__
#define UNITREE_Z1_HW_INTERFACE_HPP__

#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace unitree::z1 {

class HardwareInterface : public hardware_interface::SystemInterface {
public:
    using Vec6 = Eigen::Vector<double, 6>;  // NOLINT: magic number

    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

    HardwareInterface()           = default;
    ~HardwareInterface() override = default;

    HardwareInterface(const HardwareInterface&)             = delete;
    HardwareInterface(const HardwareInterface&&)            = delete;
    HardwareInterface& operator=(const HardwareInterface&)  = delete;
    HardwareInterface& operator=(const HardwareInterface&&) = delete;


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

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

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
    rclcpp::Logger _logger = rclcpp::get_logger("z1_hardware_interface");

    rclcpp::Logger&
    get_logger() {
        return _logger;
    }
};


}  // namespace unitree::z1

#endif  // UNITREE_Z1_HW_INTERFACE_HPP__
