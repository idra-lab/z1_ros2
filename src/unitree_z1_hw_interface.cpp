#include <cctype>
#include <cstddef>
#include <Eigen/Core>
#include <stdexcept>
#include <string>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>
#include <unitree_arm_sdk/model/ArmModel.h>
#include <vector>
#include <stdio.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_z1_hw_interface/unitree_z1_hw_interface.hpp>

#include "unitree_arm_sdk/utilities/loop.h"


using unitree::z1::hw_interface::UnitreeZ1HWInterface;


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

#define SHOW_DEBUG_MESSAGES
#define Z1_HWI_LOGGER rclcpp::get_logger("UnitreeZ1HWInterface")
#define LOG_PERIOD 50
#define SHOW_SDK_DATA

typedef Eigen::Matrix<double, 6, 1> Vector6d;

using pos_vel_pair = std::pair<Vector6d, Vector6d>;


static void               to_lower_string(std::string& str);
std::vector<pos_vel_pair> interpolate(const Vector6d& qi,
                                      const Vector6d& qf,
                                      double          dt,
                                      double          tf);
std::string pretty_vector(const std::vector<double>& vec);

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
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_configure()");
    if (hardware_interface::SystemInterface::on_configure(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
    if (hardware_interface::SystemInterface::on_cleanup(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
        const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "calling on_shutdown()");
    if (hardware_interface::SystemInterface::on_shutdown(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_shutdown() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    arm->lowcmd->setControlGain();
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
    if (hardware_interface::SystemInterface::on_shutdown(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
    if (hardware_interface::SystemInterface::on_deactivate(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
    if (hardware_interface::SystemInterface::on_error(prev_state) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
    if (hardware_interface::SystemInterface::on_init(hw_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(Z1_HWI_LOGGER, "parent on_init() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string gripper_string = hw_info.hardware_parameters.at("gripper");
    to_lower_string(gripper_string);
    with_gripper = gripper_string == "true";
    if (with_gripper) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Gripper is enabled");
    } else {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Gripper is disabled");
    }

    arm = new UNITREE_ARM::unitreeArm(with_gripper);
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "arm object created");
    arm->sendRecvThread->start();
    arm->setArmCmd(arm->lowstate->getQ(), arm->lowstate->getQd());
    arm->backToStart();
    arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    RCLCPP_INFO(Z1_HWI_LOGGER, "Waiting arm for 1 second");
    arm->sendRecvThread->shutdown();
    rclcpp::sleep_for(std::chrono::seconds(1));
    arm->sendRecvThread->start();
    arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);

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
UnitreeZ1HWInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    RCLCPP_INFO(Z1_HWI_LOGGER, "Exporting %lu state interfaces", n_joints);
    for (std::size_t i = 0; i < n_joints; i++) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Exporting state interface for joint %s",
                     info_.joints[i].name.c_str());
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_POSITION, &rob_q[i]);
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_VELOCITY, &rob_dq[i]);
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_ACCELERATION,
                                      &rob_ddq[i]);
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_EFFORT, &rob_tau[i]);
    }
    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
UnitreeZ1HWInterface::export_command_interfaces() {
    RCLCPP_INFO(Z1_HWI_LOGGER, "Exporting %lu command interfaces", n_joints);
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < n_joints; i++) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Exporting command interface for joint %s",
                     info_.joints[i].name.c_str());
        command_interfaces.emplace_back(info_.joints[i].name,
                                        hardware_interface::HW_IF_POSITION, &cmd_q[i]);
        command_interfaces.emplace_back(info_.joints[i].name,
                                        hardware_interface::HW_IF_VELOCITY, &cmd_dq[i]);
        command_interfaces.emplace_back(info_.joints[i].name,
                                        hardware_interface::HW_IF_EFFORT, &cmd_tau[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type UnitreeZ1HWInterface::read(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    for (std::size_t i = 0; i < n_joints; i++) {
        rob_q[i]   = arm->lowstate->q[i];
        rob_dq[i]  = arm->lowstate->dq[i];
        rob_ddq[i] = arm->lowstate->ddq[i];
        rob_tau[i] = arm->lowstate->tau[i];
    }

#ifdef SHOW_SDK_DATA
    static std::size_t k = 0;
    if(k++ % LOG_PERIOD == 0) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "received q:   %s", pretty_vector(rob_q).c_str());
        RCLCPP_INFO(Z1_HWI_LOGGER, "received dq:  %s", pretty_vector(rob_dq).c_str());
        RCLCPP_INFO(Z1_HWI_LOGGER, "received tau: %s", pretty_vector(rob_tau).c_str());
    }
#endif
    arm->sendRecv();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeZ1HWInterface::write(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    for (std::size_t i = 0; i < n_joints; i++) {
        arm->lowcmd->q[i]   = cmd_q[i];
        arm->lowcmd->dq[i]  = cmd_dq[i];
        arm->lowcmd->tau[i] = cmd_tau[i];
    }
    arm->sendRecv();

#ifdef SHOW_SDK_DATA
    static std::size_t k = 0;
    if(k++ % LOG_PERIOD == 0) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "commanded q:   %s", pretty_vector(cmd_q).c_str());
        RCLCPP_INFO(Z1_HWI_LOGGER, "commanded dq:  %s", pretty_vector(cmd_dq).c_str());
        RCLCPP_INFO(Z1_HWI_LOGGER, "commanded tau: %s", pretty_vector(cmd_tau).c_str());
    }
#endif

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeZ1HWInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) {
    std::string control_mode;
    std::string mystring;
    if (start_interfaces.empty()) {
        control_mode = hardware_interface::HW_IF_EFFORT;
        RCLCPP_INFO(Z1_HWI_LOGGER, "No control_mode is specified, leaving unchanged!");
        return hardware_interface::return_type::OK;
    }
    mystring = start_interfaces[0];
    auto slash_id = mystring.find('/');
    control_mode = mystring.substr(slash_id + 1);

    if (control_mode == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Switching to position control");
        arm->lowcmd->setControlGain();
    } else if (control_mode == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Switching to velocity control");
        arm->lowcmd->setControlGain();
        arm->lowcmd->setZeroKp();
    } else if (control_mode == hardware_interface::HW_IF_EFFORT) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Switching to torque control");
        arm->lowcmd->setZeroKp();
        arm->lowcmd->setZeroKd();
    } else {
        RCLCPP_ERROR(Z1_HWI_LOGGER, "Unknown control mode %s", control_mode.data());
        return hardware_interface::return_type::ERROR;
    }

    arm->sendRecv();
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
    arm->lowcmd->setControlGain();
    Vector6d qi = Vector6d::Zero();
    for (std::size_t i = 0; i < 6; i++) {
        qi[i]             = arm->lowstate->q[i];
        arm->lowcmd->q[i] = qi[i];
    }
    double& dt    = arm->_ctrlComp->dt;
    auto    timer = UNITREE_ARM::Timer(dt);
    auto    traj  = interpolate(qi, Vector6d::Zero(), dt, 4);

    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Starting shutdown trajectory");
    for (const auto& pair : traj) {
        std::cout << pair.first.transpose() << std::endl;
        arm->setArmCmd(pair.first, pair.second);
        arm->sendRecv();
        timer.sleep();
    }
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Trajectory completed");
    arm->sendRecvThread->start();
    arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm->sendRecvThread->shutdown();
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

/**
 * @brief Interpolate between two points in joint space.
 *
 * @param[in] qi        Initial joint position.
 * @param[in] qf        Final joint position.
 * @param[in] dt        Time step.
 * @param[in] tf        Total time.
 *
 * @return A vector of pairs of joint position and velocity.
 */
std::vector<pos_vel_pair> interpolate(const Vector6d& qi,
                                      const Vector6d& qf,
                                      double          dt,
                                      double          tf) {
    std::vector<pos_vel_pair> result;
    const Vector6d            a_coeff = -2 * (qf - qi) / (tf * tf * tf);
    const Vector6d            b_coeff = 3 * (qf - qi) / (tf * tf);
    const Vector6d            c_coeff = Vector6d::Zero();
    const Vector6d            d_coeff = qi;

    Vector6d q_vec, dq_vec;
    for (double t = 0; t < tf; t += dt) {
        q_vec  = a_coeff * t * t * t + b_coeff * t * t + c_coeff * t + d_coeff;
        dq_vec = 3 * a_coeff * t * t + 2 * b_coeff * t + c_coeff;
        result.push_back(std::make_pair(q_vec, dq_vec));
    }
    return result;
}

/**
 * @brief Convert a vector to a string.
 *
 * @param[in] vec       The vector to be converted.
 *
 * @return A string representation of the vector.
 */
std::string pretty_vector(const std::vector<double>& vec) {
    std::string result = "[";
    char temp[20];
    for (std::size_t i = 0; i < vec.size(); i++) {
        if (i != 0) {
            result += ", ";
        }
        sprintf(temp, "%6.4f", vec[i]);
        result += std::string(temp);
    }
    result += "]";
    return result;
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
