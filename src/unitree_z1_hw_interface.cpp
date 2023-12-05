#include <cctype>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>
#include <unitree_arm_sdk/model/ArmModel.h>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <unitree_z1_hw_interface/unitree_z1_hw_interface.hpp>


using unitree::z1::hw_interface::UnitreeZ1HWInterface;


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

#define SHOW_DEBUG_MESSAGES
#define Z1_HWI_LOGGER rclcpp::get_logger("UnitreeZ1HWInterface")

static void to_lower_string(std::string& str);

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

UnitreeZ1HWInterface::UnitreeZ1HWInterface() {
    rclcpp::on_shutdown(std::bind(&UnitreeZ1HWInterface::shutdown, this));
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
 * This function should setup the communication with the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_configure(
        const rclcpp_lifecycle::State& prev_state){
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_configure()");
    if(hardware_interface::SystemInterface::on_configure(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_configure() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should clean up the communication with the hardware and deactivate it.
 * Opposite of on_configure.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_cleanup(
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_cleanup()");
    if(hardware_interface::SystemInterface::on_cleanup(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_cleanup() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should shutdown the hardware.
 *
 * @note This function call is bugged and isn't actually triggered.
 *       The constructor so bounds the private function "shutdown()" to properly
 *       shutdown the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_shutdown(
        const rclcpp_lifecycle::State& prev_state){
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_shutdown()");
    if(hardware_interface::SystemInterface::on_shutdown(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_shutdown() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_shutdown() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should activate the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_activate(
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_activate()");
    if(hardware_interface::SystemInterface::on_shutdown(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_shutdown() failed");

    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_activate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_deactivate(
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_deactivate()");
    if(hardware_interface::SystemInterface::on_deactivate(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_deactivate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn UnitreeZ1HWInterface::on_error(
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "called on_error()");
    if(hardware_interface::SystemInterface::on_error(prev_state) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_error() processed correctly");
    return hardware_interface::CallbackReturn::SUCCESS;
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
#ifdef SHOW_DEBUG_MESSAGES
    rclcpp::Logger logger = Z1_HWI_LOGGER;
    logger.set_level(rclcpp::Logger::Level::Debug);
#endif
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_init()");
    if(hardware_interface::SystemInterface::on_init(hw_info) != hardware_interface::CallbackReturn::SUCCESS){
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_init() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string gripper_string = hw_info.hardware_parameters.at("gripper");
    to_lower_string(gripper_string);
    with_gripper = gripper_string == "true";
    if(with_gripper){
        RCLCPP_INFO(Z1_HWI_LOGGER, "Gripper is enabled");
    }else{
        RCLCPP_INFO(Z1_HWI_LOGGER, "Gripper is disabled");
    }

    n_joints = hw_info.joints.size();
    rob_q.resize(n_joints);
    rob_dq.resize(n_joints);
    rob_ddq.resize(n_joints);
    rob_tau.resize(n_joints);
    cmd_q.resize(n_joints);
    cmd_dq.resize(n_joints);
    cmd_tau.resize(n_joints);

    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_init() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UnitreeZ1HWInterface::export_state_interfaces(){
    std::vector<hardware_interface::StateInterface> state_interfaces;
    RCLCPP_INFO(Z1_HWI_LOGGER, "Exporting %lu state interfaces", n_joints);
    for (std::size_t i = 0; i < n_joints; i++) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Exporting state interface for joint %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &rob_q[i]);
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &rob_dq[i]);
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &rob_ddq[i]);
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &rob_tau[i]);
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
UnitreeZ1HWInterface::export_command_interfaces() {
    RCLCPP_INFO(Z1_HWI_LOGGER, "Exporting %lu command interfaces", n_joints);
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < n_joints; i++) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Exporting command interface for joint %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_q[i]);
        command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_dq[i]);
        command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &cmd_tau[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type UnitreeZ1HWInterface::read(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // TODO
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeZ1HWInterface::write(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // TODO
    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//

void UnitreeZ1HWInterface::shutdown() {
    RCLCPP_INFO(Z1_HWI_LOGGER, "Shutting down the hardware interface");
    // TODO
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Hardware interface shut down successfully");

}

//  ____  _        _   _
// / ___|| |_ __ _| |_(_) ___ ___
// \___ \| __/ _` | __| |/ __/ __|
//  ___) | || (_| | |_| | (__\__ \
// |____/ \__\__,_|\__|_|\___|___/
//

/**
 * @bried Convert a string to lower case.
 *
 * @param[in,out] str       The string to be converted.
 */
static void to_lower_string(std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(),
                   [](unsigned char c) { return std::tolower(c); });
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(unitree::z1::hw_interface::UnitreeZ1HWInterface,
                       hardware_interface::SystemInterface);
