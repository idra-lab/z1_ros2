#include "unitree_z1_hw_interface/unitree_z1_hw_interface.hpp"

#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "unitree_arm_sdk/control/unitreeArm.h"
#include "unitree_arm_sdk/message/arm_common.h"
#include "unitree_arm_sdk/model/ArmModel.h"
#include "unitree_arm_sdk/utilities/loop.h"


//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

#define SHOW_DEBUG_MESSAGES
#define Z1_HWI_LOGGER rclcpp::get_logger("Z1HwInterface")
#define LOG_PERIOD    50
#define USE_5TH_ORDER_POLYNOMIAL
#define SHOW_SDK_DATA


using unitree::z1::Vector6d;
using unitree::z1::Z1HwInterface;

using pos_vel_pair = std::pair<Vector6d, Vector6d>;


static void               to_lower_string(std::string& str);
std::vector<pos_vel_pair> interpolate(
        const Vector6d& qi, const Vector6d& qf, double dt, double tf
);

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Z1HwInterface::Z1HwInterface() {
    rclcpp::on_shutdown(std::bind(&Z1HwInterface::shutdown, this));
    // TODO
}

Z1HwInterface::~Z1HwInterface() {
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
hardware_interface::CallbackReturn Z1HwInterface::on_configure(
        const rclcpp_lifecycle::State& prev_state
) {
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
hardware_interface::CallbackReturn Z1HwInterface::on_cleanup(
        const rclcpp_lifecycle::State& prev_state
) {
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
hardware_interface::CallbackReturn
Z1HwInterface::on_shutdown(const rclcpp_lifecycle::State& /* prev_state */) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should activate the hardware.
 */
hardware_interface::CallbackReturn Z1HwInterface::on_activate(
        const rclcpp_lifecycle::State& prev_state
) {
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
hardware_interface::CallbackReturn Z1HwInterface::on_deactivate(
        const rclcpp_lifecycle::State& prev_state
) {
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
hardware_interface::CallbackReturn Z1HwInterface::on_error(
        const rclcpp_lifecycle::State& prev_state
) {
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
hardware_interface::CallbackReturn Z1HwInterface::on_init(
        const hardware_interface::HardwareInfo& hw_info
) {
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


    /*
     * Create a pipe to count the number of z1_ctrl processes running.
     * If no process is running, the resulting number of z1_ctrl processes returned by
     * the shell pipe is 2, thus the controller is started. If one process is running,
     * the resulting number of z1_ctrl processes returned by the shell pipe is higher
     * avoiding the controller to be started.
     */
    FILE* nprocs_cmd = popen("ps aux | grep z1_ctrl | wc -l", "r");
    int   n_procs;
    fscanf(nprocs_cmd, "%d", &n_procs);
    pclose(nprocs_cmd);

    if (n_procs <= 2) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Starting the controller");
        const std::string controller_path =
                ament_index_cpp::get_package_prefix("unitree_z1_hw_interface") +
                "/lib/controller";
        const std::string launch_subcmd = "cd " + controller_path +
                                          " && chmod +x ./z1_ctrl" +
                                          " && ./z1_ctrl && disown";
        const std::string launch_cmd =
                "x-terminal-emulator -e '" + launch_subcmd + "' &";
        system(launch_cmd.c_str());
    }

    std::string gripper_config = hw_info.hardware_parameters.at("gripper");
    to_lower_string(gripper_config);
    with_gripper = gripper_config == "true";
    if (with_gripper) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Gripper is enabled");
    } else {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Gripper is disabled");
    }

    arm = std::make_unique<UNITREE_ARM::unitreeArm>(with_gripper);
    arm->sendRecvThread->start();
    arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm->setArmCmd(arm->lowstate->getQ(), arm->lowstate->getQd());
    arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    arm->sendRecvThread->shutdown();

    RCLCPP_DEBUG(Z1_HWI_LOGGER, "on_init() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Z1HwInterface::export_state_interfaces(
) {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    RCLCPP_INFO(Z1_HWI_LOGGER, "Exporting state interfaces for %lu joints", n_joints());
    for (std::size_t i = 0; i < 6; i++) {
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_q[i]
        );
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_dq[i]
        );
        state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_ACCELERATION,
                &state_ddq[i]
        );
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_tau[i]
        );
    }

    if (with_gripper) {
        state_interfaces.emplace_back(
                info_.joints[6].name,
                hardware_interface::HW_IF_POSITION,
                &gripper_state_q
        );
        state_interfaces.emplace_back(
                info_.joints[6].name,
                hardware_interface::HW_IF_VELOCITY,
                &gripper_state_dq
        );
        state_interfaces.emplace_back(
                info_.joints[6].name,
                hardware_interface::HW_IF_ACCELERATION,
                &gripper_state_ddq
        );
        state_interfaces.emplace_back(
                info_.joints[6].name,
                hardware_interface::HW_IF_EFFORT,
                &gripper_state_tau
        );
    }

    return state_interfaces;
};

std::vector<hardware_interface::CommandInterface>
Z1HwInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    RCLCPP_INFO(
            Z1_HWI_LOGGER, "Exporting command interfaces for %lu joints", n_joints()
    );
    for (std::size_t i = 0; i < 6; i++) {
        command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_q[i]
        );
        command_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &cmd_tau[i]
        );
    }

    if (with_gripper) {
        command_interfaces.emplace_back(
                info_.joints[6].name, hardware_interface::HW_IF_POSITION, &gripper_cmd_q
        );
        command_interfaces.emplace_back(
                info_.joints[6].name, hardware_interface::HW_IF_EFFORT, &gripper_cmd_tau
        );
    }
    return command_interfaces;
}

hardware_interface::return_type Z1HwInterface::
        read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    for (std::size_t i = 0; i < 6; i++) {
        state_q[i]   = arm->lowstate->q[i];
        state_dq[i]  = arm->lowstate->dq[i];
        state_ddq[i] = arm->lowstate->ddq[i];
        state_tau[i] = arm->lowstate->tau[i];
    }
    if (with_gripper) {
        gripper_state_q   = arm->lowstate->q[6];
        gripper_state_dq  = arm->lowstate->dq[6];
        gripper_state_ddq = arm->lowstate->ddq[6];
        gripper_state_tau = arm->lowstate->tau[6];
    }
    arm->sendRecv();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Z1HwInterface::
        write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    for (std::size_t i = 0; i < 6; i++) {
        arm->lowcmd->q[i]   = cmd_q[i];
        arm->lowcmd->dq[i]  = cmd_dq[i];
        arm->lowcmd->tau[i] = cmd_tau[i];
    }
    if (with_gripper) {
        arm->lowcmd->q[6]   = gripper_cmd_q;
        arm->lowcmd->dq[6]  = gripper_cmd_dq;
        arm->lowcmd->tau[6] = gripper_cmd_tau;
    }
    arm->sendRecv();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Z1HwInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& /* stop_interfaces */
) {
    std::string control_mode;
    std::string req_cmd_interface;
    if (start_interfaces.empty()) {
        control_mode = hardware_interface::HW_IF_EFFORT;
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "No control_mode is specified, leaving unchanged!");
        return hardware_interface::return_type::OK;
    }
    req_cmd_interface = start_interfaces[0];
    auto slash_id     = req_cmd_interface.find('/');
    control_mode      = req_cmd_interface.substr(slash_id + 1);

    if (control_mode == hardware_interface::HW_IF_POSITION) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Switching to position control");
        arm->lowcmd->setControlGain();
    } else if (control_mode == hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Switching to velocity control");
        RCLCPP_WARN(Z1_HWI_LOGGER, "Velocity control is not fully tested yet!");
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

void Z1HwInterface::shutdown() {
    RCLCPP_INFO(Z1_HWI_LOGGER, "Shutting down the hardware interface");
    arm->lowcmd->setControlGain();
    Vector6d qi = Vector6d::Zero();
    for (std::size_t i = 0; i < 6; i++) {
        qi[i]             = arm->lowstate->q[i];
        arm->lowcmd->q[i] = qi[i];
    }
    const double& dt    = arm->_ctrlComp->dt;
    auto          timer = UNITREE_ARM::Timer(dt);
    auto          traj  = interpolate(qi, Vector6d::Zero(), dt, 4);

    for (const auto& pair : traj) {
        std::cout << pair.first.transpose() << std::endl;
        arm->setArmCmd(pair.first, pair.second);
        arm->sendRecv();
        timer.sleep();
    }
    arm->sendRecvThread->start();
    arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm->sendRecvThread->shutdown();
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Hardware interface shut down successfully");
}

std::size_t Z1HwInterface::n_joints() const { return with_gripper ? 7 : 6; }

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
static void to_lower_string(std::string& str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
        return std::tolower(c);
    });
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
std::vector<pos_vel_pair> interpolate(
        const Vector6d& qi, const Vector6d& qf, double dt, double tf
) {
    std::vector<pos_vel_pair> result;
    const Vector6d            delta   = qf - qi;
    double                    tf_pow2 = pow(tf, 2);
    double                    tf_pow3 = pow(tf, 3);
    double                    tf_pow4 = pow(tf, 4);
    double                    tf_pow5 = pow(tf, 5);
#ifndef USE_5TH_ORDER_POLYNOMIAL
    const Vector6d a_coeff = -2 * delta / tf_pow3;
    const Vector6d b_coeff = 3 * delta / tf_pow2;
    const Vector6d c_coeff = Vector6d::Zero();
    const Vector6d d_coeff = qi;
#else
    const Vector6d a_coeff = 6 * delta / tf_pow5;
    const Vector6d b_coeff = -15 * delta / tf_pow4;
    const Vector6d c_coeff = 10 * delta / tf_pow3;
    const Vector6d d_coeff = Vector6d::Zero();
    const Vector6d e_coeff = Vector6d::Zero();
    const Vector6d f_coeff = qi;
#endif

    Vector6d q_vec, dq_vec;
    for (double t = 0; t < tf; t += dt) {
        const double t_pow2 = pow(t, 2);
        const double t_pow3 = pow(t, 3);
        const double t_pow4 = pow(t, 4);
        const double t_pow5 = pow(t, 5);
#ifndef USE_5TH_ORDER_POLYNOMIAL
        q_vec  = a_coeff * t_pow3 + b_coeff * t_pow2 + c_coeff * t + d_coeff;
        dq_vec = 3 * a_coeff * t_pow2 + 2 * b_coeff * t + c_coeff;
#else
        q_vec = a_coeff * t_pow5 + b_coeff * t_pow4 + c_coeff * t_pow3 +
                d_coeff * t_pow2 + e_coeff * t + f_coeff;
        dq_vec = 5 * a_coeff * t_pow4 + 4 * b_coeff * t_pow3 + 3 * c_coeff * t_pow2 +
                 2 * d_coeff * t + e_coeff;
#endif

        result.push_back(std::make_pair(q_vec, dq_vec));
    }
    return result;
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(unitree::z1::Z1HwInterface, hardware_interface::SystemInterface);
