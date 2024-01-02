#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <math.h>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>
#include <unitree_arm_sdk/model/ArmModel.h>
#include <unitree_arm_sdk/utilities/loop.h>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
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
#define LOG_PERIOD    50
#define USE_5TH_ORDER_POLYNOMIAL
#define SHOW_SDK_DATA

typedef Eigen::Matrix<double, 6, 1> Vector6d;

using pos_vel_pair = std::pair<Vector6d, Vector6d>;


static void               to_lower_string(std::string& str);
std::vector<pos_vel_pair> interpolate(const Vector6d& qi,
                                      const Vector6d& qf,
                                      double          dt,
                                      double          tf);
std::string               pretty_vector(const std::vector<double>& vec);

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


    /*
     * Create a pipe to count the number of z1_ctrl processes running.
     * If no process is running, the resulting number of z1_ctrl processes returned by
     * the shell pipe is 2, thus the controller is started. If one process is running,
     * the resulting number of z1_ctrl processes returned by the shell pipe is higher
     * avoiding the controller to be started.
     */
    const std::string cmd_num_procs = "ps aux | grep z1_ctrl | wc -l";
    FILE*             cmd_pipe      = popen(cmd_num_procs.c_str(), "r");
    int               n_procs;
    fscanf(cmd_pipe, "%d", &n_procs);
    pclose(cmd_pipe);
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Number of z1_ctrl processes: %d", n_procs);

    if (n_procs <= 2) {
        RCLCPP_INFO(Z1_HWI_LOGGER, "Starting the controller");
        const std::string prefix =
                ament_index_cpp::get_package_prefix("unitree_z1_hw_interface");
        const std::string ctrl_path = prefix + "/lib/controller";
        const std::string subcmd    = "cd " + ctrl_path + " && chmod +x ./z1_ctrl" +
                                   " && ./z1_ctrl && disown";
        const std::string cmd = "x-terminal-emulator -e '" + subcmd + "' &";
        int               ok  = system(cmd.c_str());
        // rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Command: %s", cmd.c_str());
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Result : %d", ok);
    }

    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Setting up KDL chain");
    if (!kdl_parser::treeFromString(hw_info.original_xml, arm_tree)) {
        RCLCPP_ERROR(Z1_HWI_LOGGER, "Failed to construct KDL tree from original XML");
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::string root_link         = "world";
    std::string end_effector_link = "gripperMover";
    if (!arm_tree.getChain(root_link, end_effector_link, arm_chain)) {
        RCLCPP_ERROR(Z1_HWI_LOGGER, "Failed to construct KDL chain from KDL tree");
        return hardware_interface::CallbackReturn::ERROR;
    }
    arm_q.resize(arm_chain.getNrOfJoints());
    arm_dq.resize(arm_chain.getNrOfJoints());
    arm_ddq.resize(arm_chain.getNrOfJoints());
    for (std::size_t i = 0; i < arm_chain.getNrOfJoints(); i++) {
        arm_q(i)   = 0;
        arm_dq(i)  = 0;
        arm_ddq(i) = 0;
    }
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "KDL chain setup completed successfully");

    std::string gripper_string = hw_info.hardware_parameters.at("gripper");
    to_lower_string(gripper_string);
    with_gripper = gripper_string == "true";
    if (with_gripper) {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Gripper is enabled");
    } else {
        RCLCPP_DEBUG(Z1_HWI_LOGGER, "Gripper is disabled");
    }

    arm = new UNITREE_ARM::unitreeArm(with_gripper);
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Starting communication");
    arm->sendRecvThread->start();
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Communication started");
    arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm->setArmCmd(arm->lowstate->getQ(), arm->lowstate->getQd());
    arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    arm->sendRecvThread->shutdown();
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Communication shutdown");

    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Resizing:");
    std::size_t kdl_chain_size = arm_chain.getNrOfJoints();
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "KDL chain size: %lu", kdl_chain_size);
    RCLCPP_DEBUG(Z1_HWI_LOGGER, "Joint names size: %lu", hw_info.joints.size());

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
        RCLCPP_DEBUG(Z1_HWI_LOGGER,
                     "Exporting state interface for joint %s",
                     info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &rob_q[i]);
        state_interfaces.emplace_back(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &rob_dq[i]);
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_ACCELERATION,
                                      &rob_ddq[i]);
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
        RCLCPP_DEBUG(Z1_HWI_LOGGER,
                     "Exporting command interface for joint %s",
                     info_.joints[i].name.c_str());
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
    for (std::size_t i = 0; i < n_joints; i++) {
        rob_q[i]   = arm->lowstate->q[i];
        rob_dq[i]  = arm->lowstate->dq[i];
        rob_ddq[i] = arm->lowstate->ddq[i];
        rob_tau[i] = arm->lowstate->tau[i];
    }

    std::size_t chain_size = arm_chain.getNrOfJoints();
    std::size_t n_joints   = std::min(this->n_joints, chain_size);
    for (std::size_t i = 0; i < std::min(n_joints, chain_size); i++) {
        arm_q(i)   = arm->lowstate->q[i];
        arm_dq(i)  = arm->lowstate->dq[i];
        arm_ddq(i) = arm->lowstate->ddq[i];
    }

#ifdef SHOW_SDK_DATA
    static std::size_t k = 0;
    if (k++ % LOG_PERIOD == 0) {
        KDL::ChainDynParam dyn_param(arm_chain, KDL::Vector(0, 0, -9.81));
        KDL::JntArray      arm_tau(chain_size);
        dyn_param.JntToGravity(arm_q, arm_tau);
        // RCLCPP_INFO(Z1_HWI_LOGGER, "received q:   %s", pretty_vector(rob_q).c_str());
        // RCLCPP_INFO(Z1_HWI_LOGGER, "received dq:  %s",
        // pretty_vector(rob_dq).c_str());
        RCLCPP_INFO(Z1_HWI_LOGGER, "received tau: %s", pretty_vector(rob_tau).c_str());
        std::string tmp = "";
        for (std::size_t i = 0; i < chain_size; i++) {
            tmp += std::to_string(arm_tau(i)) + " ";
        }
        RCLCPP_INFO(Z1_HWI_LOGGER, "computed tau: %s", tmp.c_str());
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
    if (k++ % LOG_PERIOD == 0) {
        // RCLCPP_INFO(Z1_HWI_LOGGER, "commanded q:   %s",
        // pretty_vector(cmd_q).c_str()); RCLCPP_INFO(Z1_HWI_LOGGER, "commanded dq: %s",
        // pretty_vector(cmd_dq).c_str()); RCLCPP_INFO(Z1_HWI_LOGGER, "commanded tau:
        // %s", pretty_vector(cmd_tau).c_str());
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
    mystring      = start_interfaces[0];
    auto slash_id = mystring.find('/');
    control_mode  = mystring.substr(slash_id + 1);

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
std::vector<pos_vel_pair> interpolate(const Vector6d& qi,
                                      const Vector6d& qf,
                                      double          dt,
                                      double          tf) {
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

/**
 * @brief Convert a vector to a string.
 *
 * @param[in] vec       The vector to be converted.
 *
 * @return A string representation of the vector.
 */
std::string pretty_vector(const std::vector<double>& vec) {
    std::string result = "[";
    char        temp[20];
    for (std::size_t i = 0; i < vec.size(); i++) {
        if (i != 0) { result += ", "; }
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
