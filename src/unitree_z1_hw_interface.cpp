#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <unitree_z1_hw_interface/unitree_z1_hw_interface.hpp>

using unitree::z1::hw_interface::UnitreeZ1HWInterface;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

UnitreeZ1HWInterface::UnitreeZ1HWInterface() {}

UnitreeZ1HWInterface::~UnitreeZ1HWInterface() {}

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_configure(
    const rclcpp_lifecycle::State& prev_state) {}

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_cleanup(
    const rclcpp_lifecycle::State& prev_state) {}

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_shutdown(
    const rclcpp_lifecycle::State& prev_state) {}

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_activate(
    const rclcpp_lifecycle::State& prev_state) {}

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_deactivate(
    const rclcpp_lifecycle::State& prev_state) {}

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_error(
    const rclcpp_lifecycle::State& prev_state) {}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_init(
    const hardware_interface::HardwareInfo& hw_info) {}

std::vector<hardware_interface::StateInterface>
UnitreeZ1HWInterface::export_state_interfaces(){};

std::vector<hardware_interface::CommandInterface>
UnitreeZ1HWInterface::export_command_interfaces() {}

hardware_interface::return_type UnitreeZ1HWInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {}

hardware_interface::return_type UnitreeZ1HWInterface::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//
