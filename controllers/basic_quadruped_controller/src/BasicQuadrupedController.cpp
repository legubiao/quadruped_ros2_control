//
// Created by tlab-uav on 24-9-6.
//

#include "basic_quadruped_controller/BasicQuadrupedController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace basic_quadruped_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration BasicQuadrupedController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration BasicQuadrupedController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type BasicQuadrupedController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        ctrl_component_.robot_model_->update();
        ctrl_component_.wave_generator_->update(time);
        ctrl_component_.estimator_->update();

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            // Set WaveGenerator status based on FSM state
            if (ctrl_component_.wave_generator_)
            {
                ctrl_component_.wave_generator_->setStatusFromFSM(next_state_name_);
            }

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn BasicQuadrupedController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // Robot parameters - use robot_name to auto-generate package names
            robot_name_ = auto_declare<std::string>("robot_name", "unitree_go2");

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);


            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_gains_ = auto_declare<std::vector<double>>("stand_gains", stand_gains_);
            swing_gains_ = auto_declare<std::vector<double>>("swing_gains", swing_gains_);
            stable_gains_ = auto_declare<std::vector<double>>("stable_gains", stable_gains_);
            gait_height_ = auto_declare<double>("gait_height", gait_height_);

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // Initialize robot model with auto-generated URDF path
            initializeRobotModel();

            ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_, get_node());
            ctrl_component_.balance_ctrl_ = std::make_shared<BalanceCtrl>(ctrl_component_.robot_model_);
            // 使用当前时间作为启动时间
            ctrl_component_.wave_generator_ = std::make_shared<WaveGenerator>(
                0.45, 0.5, Vec4(0, 0.5, 0.5, 0), get_node()->now());
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    void BasicQuadrupedController::initializeRobotModel()
    {
        try
        {
            // Automatically build file paths and initialize interface
            const std::string robot_pkg = robot_name_ + "_description";
            const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

            std::string urdf_file = config_path + "/urdf/" + robot_name_ + ".urdf";
            // 准备默认站立关节角度
            Vec12 stand_joint_positions;
            for (int i = 0; i < 4; ++i)
            {
                stand_joint_positions.segment(3 * i, 3) = Eigen::Map<const Vec3>(&stand_pos_[3 * i]);
            }

            // Initialize robot model with default stand joint positions
            ctrl_component_.robot_model_ = std::make_shared<QuadrupedKinematic>(
                ctrl_interfaces_, urdf_file, joint_names_, stand_joint_positions);

            RCLCPP_INFO(get_node()->get_logger(), "Robot model initialized successfully for robot: %s",
                        robot_name_.c_str());
            RCLCPP_INFO(get_node()->get_logger(), "URDF file: %s", urdf_file.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize robot model: %s", e.what());
            throw;
        }
    }

    controller_interface::CallbackReturn BasicQuadrupedController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    BasicQuadrupedController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_gains_[0], stand_gains_[1]);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, ctrl_component_, stand_pos_,
                                                                   stand_gains_[0], stand_gains_[1]);
        state_list_.freeStand = std::make_shared<StateFreeStand>(ctrl_interfaces_, ctrl_component_, stand_gains_[0],
                                                                 stand_gains_[1]);
        state_list_.balanceTest = std::make_shared<StateBalanceTest>(ctrl_interfaces_, ctrl_component_, stable_gains_);
        state_list_.trotting = std::make_shared<StateTrotting>(ctrl_interfaces_, ctrl_component_, swing_gains_, stable_gains_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicQuadrupedController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    BasicQuadrupedController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    BasicQuadrupedController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    BasicQuadrupedController::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> BasicQuadrupedController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::FREESTAND:
            return state_list_.freeStand;
        case FSMStateName::TROTTING:
            return state_list_.trotting;
        case FSMStateName::BALANCETEST:
            return state_list_.balanceTest;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(basic_quadruped_controller::BasicQuadrupedController, controller_interface::ControllerInterface);
