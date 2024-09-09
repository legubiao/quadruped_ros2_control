//
// Created by tlab-uav on 24-9-6.
//

#include <quadruped_ros2_control/QuadrupedController.h>

namespace quadruped_ros2_control {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration QuadrupedController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: command_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration QuadrupedController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type QuadrupedController::
    update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        if (stand_) {
            for (auto i: joint_effort_command_interface_) {
                i.get().set_value(30);
            }
        } else {
            for (auto i: joint_effort_command_interface_) {
                i.get().set_value(0);
            }
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn QuadrupedController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);
            command_interface_types_ =
                    auto_declare<std::vector<std::string> >("command_interfaces", command_interface_types_);
            state_interface_types_ =
                    auto_declare<std::vector<std::string> >("state_interfaces", state_interface_types_);

            // Initialize variables and pointers
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn QuadrupedController::on_configure(
        const rclcpp_lifecycle::State &previous_state) {
        stand_ = false;
        state_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
            "state_command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                // Handle message
                RCLCPP_INFO(get_node()->get_logger(), "Received command: %s", msg->data.c_str());
                if (msg.get()->data == "stand") {
                    stand_ = true;
                } else {
                    stand_ = false;
                }
            });
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    QuadrupedController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        // clear out vectors in case of restart
        joint_effort_command_interface_.clear();
        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        // assign command interfaces
        for (auto &interface: command_interfaces_) {
            command_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn QuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State &previous_state) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    QuadrupedController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn QuadrupedController::on_error(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    QuadrupedController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        return CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_ros2_control::QuadrupedController, controller_interface::ControllerInterface);
