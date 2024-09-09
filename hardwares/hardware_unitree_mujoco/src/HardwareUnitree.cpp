//
// Created by biao on 24-9-9.
//

#include <hardware_unitree_mujoco/crc32.h>
#include <hardware_unitree_mujoco/HardwareUnitree.h>
#include <rclcpp/logging.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::robot;
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    joint_effort_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    for (const auto &joint: info_.joints) {
        for (const auto &interface: joint.state_interfaces) {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }

    RCLCPP_INFO(get_logger(), "Initializing Unitree SDK2 Channel");
    ChannelFactory::Instance()->Init(0, "lo");
    lowcmd_publisher =
            std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_> >(
                TOPIC_LOWCMD);
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber =
            std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_> >(
                TOPIC_LOWSTATE);
    lowstate_subscriber->InitChannel(
        [this](auto &&PH1) {
            low_state_message_handler(std::forward<decltype(PH1)>(PH1));
        },
        1);
    init_low_cmd();
    RCLCPP_INFO(get_logger(), "Initialized Unitree SDK2 Channel");


    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name: joint_interfaces["position"]) {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["velocity"]) {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["effort"]) {
        state_interfaces.emplace_back(joint_name, "effort", &joint_velocities_[ind++]);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto &joint_name: joint_interfaces["position"]) {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["velocity"]) {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["effort"]) {
        command_interfaces.emplace_back(joint_name, "effort", &joint_effort_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    for (int i(0); i < 12; ++i) {
        joint_position_[i] = _lowState.motor_state()[i].q();
        joint_velocities_[i] = _lowState.motor_state()[i].dq();
        joint_effort_[i] = _lowState.motor_state()[i].tau_est();
    }
    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // send command
    for (int i(0); i < 12; ++i) {
        _lowCmd.motor_cmd()[i].mode() = 0x01;
        _lowCmd.motor_cmd()[i].q() = static_cast<float>(joint_position_command_[i]);
        _lowCmd.motor_cmd()[i].dq() = static_cast<float>(joint_velocities_command_[i]);
        _lowCmd.motor_cmd()[i].kp() = static_cast<float>(joint_kp_command_[i]);
        _lowCmd.motor_cmd()[i].kd() = static_cast<float>(joint_kd_command_[i]);
        _lowCmd.motor_cmd()[i].tau() = static_cast<float>(joint_effort_command_[i]);
    }

    _lowCmd.crc() = crc32_core(reinterpret_cast<uint32_t *>(&_lowCmd),
                               (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(_lowCmd);
    return return_type::OK;
}

void HardwareUnitree::init_low_cmd() {
    _lowCmd.head()[0] = 0xFE;
    _lowCmd.head()[1] = 0xEF;
    _lowCmd.level_flag() = 0xFF;
    _lowCmd.gpio() = 0;

    for (int i = 0; i < 20; i++) {
        _lowCmd.motor_cmd()[i].mode() =
                0x01; // motor switch to servo (PMSM) mode
        _lowCmd.motor_cmd()[i].q() = 0;
        _lowCmd.motor_cmd()[i].kp() = 0;
        _lowCmd.motor_cmd()[i].dq() = 0;
        _lowCmd.motor_cmd()[i].kd() = 0;
        _lowCmd.motor_cmd()[i].tau() = 0;
    }
}

void HardwareUnitree::low_state_message_handler(const void *messages) {
    _lowState = *(unitree_go::msg::dds_::LowState_ *) messages;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
