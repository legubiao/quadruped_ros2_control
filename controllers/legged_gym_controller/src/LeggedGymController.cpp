//
// Created by tlab-uav on 24-10-4.
//

#include "LeggedGymController.h"

namespace legged_gym_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration LeggedGymController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: command_interface_types_) {
                if (!command_prefix_.empty()) {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                } else {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LeggedGymController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto &interface_type: imu_interface_types_) {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        for (const auto &interface_type: foot_force_interface_types_) {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type LeggedGymController::
    update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        if (mode_ == FSMMode::NORMAL) {
            current_state_->run();
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name) {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        } else if (mode_ == FSMMode::CHANGE) {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn LeggedGymController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            feet_names_ = auto_declare<std::vector<std::string> >("feet_names", feet_names_);
            command_interface_types_ =
                    auto_declare<std::vector<std::string> >("command_interfaces", command_interface_types_);
            state_interface_types_ =
                    auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);

            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string> >("imu_interfaces", state_interface_types_);

            // rl config folder
            rl_config_folder_ = auto_declare<std::string>("config_folder", rl_config_folder_);

            get_node()->get_parameter("update_rate", ctrl_comp_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Update Rate: %d Hz", ctrl_comp_.frequency_);
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
                // Handle message
                ctrl_comp_.control_inputs_.command = msg->command;
                ctrl_comp_.control_inputs_.lx = msg->lx;
                ctrl_comp_.control_inputs_.ly = msg->ly;
                ctrl_comp_.control_inputs_.rx = msg->rx;
                ctrl_comp_.control_inputs_.ry = msg->ry;
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        // clear out vectors in case of restart
        ctrl_comp_.clear();

        // assign command interfaces
        for (auto &interface: command_interfaces_) {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos) {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            } else {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            if (interface.get_prefix_name() == imu_name_) {
                ctrl_comp_.imu_state_interface_.emplace_back(interface);
            } else if (interface.get_prefix_name() == foot_force_name_) {
                ctrl_comp_.foot_force_state_interface_.emplace_back(interface);
            } else {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_comp_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_comp_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_comp_);
        state_list_.rl = std::make_shared<StateRL>(ctrl_comp_, rl_config_folder_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_cleanup(previous_state);
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_shutdown(previous_state);
    }

    controller_interface::CallbackReturn LeggedGymController::on_error(const rclcpp_lifecycle::State &previous_state) {
        return ControllerInterface::on_error(previous_state);
    }

    std::shared_ptr<FSMState> LeggedGymController::getNextState(const FSMStateName stateName) const {
        switch (stateName) {
            case FSMStateName::INVALID:
                return state_list_.invalid;
            case FSMStateName::PASSIVE:
                return state_list_.passive;
            case FSMStateName::FIXEDDOWN:
                return state_list_.fixedDown;
            case FSMStateName::FIXEDSTAND:
                return state_list_.fixedStand;
            case FSMStateName::RL:
                return state_list_.rl;
            default:
                return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(legged_gym_controller::LeggedGymController, controller_interface::ControllerInterface);
