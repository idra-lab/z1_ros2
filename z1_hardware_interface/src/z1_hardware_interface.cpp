#include "z1_hardware_interface/z1_hardware_interface.hpp"

#include "rclcpp/logging.hpp"

//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using unitree::z1::HardwareInterface;

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/
hardware_interface::CallbackReturn
HardwareInterface::on_configure(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_configure()");
    if (hardware_interface::SystemInterface::on_configure(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_configure() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_cleanup()");
    if (hardware_interface::SystemInterface::on_cleanup(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_cleanup() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_shutdown()");
    if (hardware_interface::SystemInterface::on_shutdown(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_shutdown() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_activate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_activate()");
    if (hardware_interface::SystemInterface::on_activate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_activate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_deactivate()");
    if (hardware_interface::SystemInterface::on_deactivate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_deactivate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_error(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "called on_error()");
    if (hardware_interface::SystemInterface::on_error(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_error() processed correctly");
    return hardware_interface::CallbackReturn::SUCCESS;
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

hardware_interface::CallbackReturn
HardwareInterface::on_init(const hardware_interface::HardwareInfo& hw_info) {
#ifdef SHOW_DEBUG_MESSAGES
    rclcpp::Logger logger = get_logger();
    logger.set_level(rclcpp::Logger::Level::Debug);
#endif

    RCLCPP_DEBUG(get_logger(), "calling on_init()");
    if (hardware_interface::SystemInterface::on_init(hw_info)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_init() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(get_logger(), "on_init() completed successfully");
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // TODO
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
HardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // TODO
    return command_interfaces;
}

hardware_interface::return_type
HardwareInterface::
        read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::
        write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& /* stop_interfaces */
) {
    return hardware_interface::return_type::OK;
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
 * @bried Convert in-place a string to lower case.
 *
 * @param[in,out] str       The string to be converted.
 */
static void
to_lower_string(std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
        unitree::z1::HardwareInterface, hardware_interface::SystemInterface
);
