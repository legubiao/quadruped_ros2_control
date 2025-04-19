// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_quadruped_hardware/gz_system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace gz_quadruped_hardware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // Forward declaration
    class GazeboSimSystemPrivate;

    // These class must inherit `gz_ros2_control::GazeboSimSystemInterface` which implements a
    // simulated `ros2_control` `hardware_interface::SystemInterface`.

    class GazeboSimSystem : public GazeboSimSystemInterface
    {
    public:
        // Documentation Inherited
        CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info)
        override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        // Documentation Inherited
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // Documentation Inherited
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Documentation Inherited
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        // Documentation Inherited
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        // Documentation Inherited
        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        // Documentation Inherited
        hardware_interface::return_type read(
            const rclcpp::Time& time,
            const rclcpp::Duration& period) override;

        // Documentation Inherited
        hardware_interface::return_type write(
            const rclcpp::Time& time,
            const rclcpp::Duration& period) override;

        // Documentation Inherited
        bool initSim(
            rclcpp::Node::SharedPtr& model_nh,
            std::map<std::string, sim::Entity>& joints,
            const hardware_interface::HardwareInfo& hardware_info,
            sim::EntityComponentManager& _ecm,
            int& update_rate) override;

    private:
        // Register a sensor (for now just IMUs)
        // \param[in] hardware_info hardware information where the data of
        // the sensors is extract.
        void registerSensors(
            const hardware_interface::HardwareInfo& hardware_info);

        /// \brief Private data class
        std::unique_ptr<GazeboSimSystemPrivate> dataPtr;
    };
}
