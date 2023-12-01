#include <stdexcept>
#include <string>
#include <cctype>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logging.hpp>
#include <unitree_z1_hw_interface/unitree_z1_hw_interface.hpp>
#include "hardware_interface/system_interface.hpp"

using unitree::z1::hw_interface::UnitreeZ1HWInterface;


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

static void to_lower_string(const std::string& str);

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

UnitreeZ1HWInterface::UnitreeZ1HWInterface() {
    // TODO
}

UnitreeZ1HWInterface::~UnitreeZ1HWInterface() {
    // TODO
}

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/
/**
 * According to ROS2 lifecycle documentation, the state is regarded as a finite state
 * machine with the following states:
 *   - unconfigured
 *   - inactive
 *   - active
 *   - finalized
 *
 * The following are the transition functions that are called when the state changes.
 *
 */

/**
 * This function should setup the communication with the hardware and activate it.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_configure(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

/**
 * This function should clean up the communication with the hardware and deactivate it.
 * Opposite of on_configure.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_cleanup(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

/**
 * This function should shutdown the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_shutdown(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

/**
 * This function should activate the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_activate(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_deactivate(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_error(
        const rclcpp_lifecycle::State& prev_state) {
    // TODO
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

/**
 * This function is responsible to initialize parameters coming from the URDF file
 * through "hw_info".
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_init(
        const hardware_interface::HardwareInfo& hw_info) {
    // TODO
}

std::vector<hardware_interface::StateInterface>
UnitreeZ1HWInterface::export_state_interfaces(){
        // TODO
};

std::vector<hardware_interface::CommandInterface>
UnitreeZ1HWInterface::export_command_interfaces() {
    // TODO
}

hardware_interface::return_type UnitreeZ1HWInterface::read(
        const rclcpp::Time& time, const rclcpp::Duration& period) {
    // TODO
}

hardware_interface::return_type UnitreeZ1HWInterface::write(
        const rclcpp::Time& time, const rclcpp::Duration& period) {
    // TODO
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//



//  ____  _        _   _
// / ___|| |_ __ _| |_(_) ___ ___
// \___ \| __/ _` | __| |/ __/ __|
//  ___) | || (_| | |_| | (__\__ \
// |____/ \__\__,_|\__|_|\___|___/
//

/**
 * @bried Convert a string to lower case.
 *
 * @param str       The string to be converted.
 */
static void to_lower_string(const std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(),
                          [](unsigned char c) { return std::tolower(c); }
    );
}
