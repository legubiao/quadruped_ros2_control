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

#include <unistd.h>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>


#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include "gz_quadruped_hardware/gz_quadruped_plugin.hpp"
#include "gz_quadruped_hardware/gz_system.hpp"

namespace gz_quadruped_hardware {
    class GZResourceManager : public hardware_interface::ResourceManager {
    public:
        GZResourceManager(
            rclcpp::Node::SharedPtr &node,
            sim::EntityComponentManager &ecm,
            std::map<std::string, sim::Entity> enabledJoints)
            : ResourceManager(
                  node->get_node_clock_interface(), node->get_node_logging_interface()),
              gz_system_loader_("gz_quadruped_hardware", "gz_quadruped_hardware::GazeboSimSystemInterface"),
              logger_(node->get_logger().get_child("GZResourceManager")) {
            node_ = node;
            ecm_ = &ecm;
            enabledJoints_ = enabledJoints;
        }

        GZResourceManager(const GZResourceManager &) = delete;

        // Called from Controller Manager when robot description is initialized from callback
        bool load_and_initialize_components(
            const std::string &urdf,
            unsigned int update_rate) override {
            components_are_loaded_and_initialized_ = true;

            const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

            for (const auto &individual_hardware_info: hardware_info) {
                std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
                RCLCPP_DEBUG(
                    logger_, "Load hardware interface %s ...",
                    robot_hw_sim_type_str_.c_str());

                // Load hardware
                std::unique_ptr<GazeboSimSystemInterface> gzSimSystem;
                std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
                try {
                    gzSimSystem = std::unique_ptr<GazeboSimSystemInterface>(
                        gz_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
                } catch (pluginlib::PluginlibException &ex) {
                    RCLCPP_ERROR(
                        logger_,
                        "The plugin failed to load for some reason. Error: %s\n",
                        ex.what());
                    continue;
                }

                // initialize simulation requirements
                if (!gzSimSystem->initSim(
                    node_,
                    enabledJoints_,
                    individual_hardware_info,
                    *ecm_,
                    update_rate)) {
                    RCLCPP_FATAL(
                        logger_, "Could not initialize robot simulation interface");
                    components_are_loaded_and_initialized_ = false;
                    break;
                }
                RCLCPP_DEBUG(
                    logger_, "Initialized robot simulation interface %s!",
                    robot_hw_sim_type_str_.c_str());

                // initialize hardware
                import_component(std::move(gzSimSystem), individual_hardware_info);
            }

            return components_are_loaded_and_initialized_;
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        sim::EntityComponentManager *ecm_;
        std::map<std::string, sim::Entity> enabledJoints_;

        /// \brief Interface loader
        pluginlib::ClassLoader<GazeboSimSystemInterface> gz_system_loader_;

        rclcpp::Logger logger_;
    };

    //////////////////////////////////////////////////
    class GazeboSimQuadrupedPluginPrivate {
    public:
        /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
        /// joint names are specified in the plugin configuration, all valid 1-axis
        /// joints are returned
        /// \param[in] _entity Entity of the model that the plugin is being
        /// configured for
        /// \param[in] _ecm Gazebo Entity Component Manager
        /// \return List of entities containing all enabled joints
        std::map<std::string, sim::Entity> GetEnabledJoints(
            const sim::Entity &_entity,
            sim::EntityComponentManager &_ecm) const;

        /// \brief Entity ID for sensor within Gazebo.
        sim::Entity entity_;

        /// \brief Node Handles
        std::shared_ptr<rclcpp::Node> node_{nullptr};

        /// \brief Thread where the executor will spin
        std::thread thread_executor_spin_;

        /// \brief Executor to spin the controller
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

        /// \brief Timing
        rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

        /// \brief Controller manager
        std::shared_ptr<controller_manager::ControllerManager>
        controller_manager_{nullptr};

        /// \brief Last time the update method was called
        rclcpp::Time last_update_sim_time_ros_ =
                rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);

        /// \brief ECM pointer
        sim::EntityComponentManager *ecm{nullptr};

        /// \brief controller update rate
        int update_rate;
    };

    //////////////////////////////////////////////////
    std::map<std::string, sim::Entity>
    GazeboSimQuadrupedPluginPrivate::GetEnabledJoints(
        const sim::Entity &_entity,
        sim::EntityComponentManager &_ecm) const {
        std::map<std::string, sim::Entity> output;

        std::vector<std::string> enabledJoints;

        // Get all available joints
        auto jointEntities = _ecm.ChildrenByComponents(_entity, sim::components::Joint());

        // Iterate over all joints and verify whether they can be enabled or not
        for (const auto &jointEntity: jointEntities) {
            const auto jointName = _ecm.Component<sim::components::Name>(
                jointEntity)->Data();

            // Make sure the joint type is supported, i.e. it has exactly one
            // actuated axis
            const auto *jointType = _ecm.Component<sim::components::JointType>(jointEntity);
            switch (jointType->Data()) {
                case sdf::JointType::PRISMATIC:
                case sdf::JointType::REVOLUTE:
                case sdf::JointType::CONTINUOUS:
                case sdf::JointType::GEARBOX: {
                    // Supported joint type
                    break;
                }
                case sdf::JointType::FIXED: {
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "[gz_quadruped_hardware] Fixed joint [%s] (Entity=%lu)] is skipped",
                        jointName.c_str(), jointEntity);
                    continue;
                }
                case sdf::JointType::REVOLUTE2:
                case sdf::JointType::SCREW:
                case sdf::JointType::BALL:
                case sdf::JointType::UNIVERSAL: {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "[gz_quadruped_hardware] Joint [%s] (Entity=%lu)] is of unsupported type."
                        " Only joints with a single axis are supported.",
                        jointName.c_str(), jointEntity);
                    continue;
                }
                default: {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "[gz_quadruped_hardware] Joint [%s] (Entity=%lu)] is of unknown type",
                        jointName.c_str(), jointEntity);
                    continue;
                }
            }
            output[jointName] = jointEntity;
        }

        return output;
    }

    //////////////////////////////////////////////////
    GazeboSimQuadrupedPlugin::GazeboSimQuadrupedPlugin()
        : dataPtr(std::make_unique<GazeboSimQuadrupedPluginPrivate>()) {
    }

    //////////////////////////////////////////////////
    GazeboSimQuadrupedPlugin::~GazeboSimQuadrupedPlugin() {
        // Stop controller manager thread
        if (!this->dataPtr->controller_manager_) {
            return;
        }
        this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);
        this->dataPtr->executor_->cancel();
        this->dataPtr->thread_executor_spin_.join();
    }

    //////////////////////////////////////////////////
    void GazeboSimQuadrupedPlugin::Configure(
        const sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        sim::EntityComponentManager &_ecm,
        sim::EventManager &) {
        rclcpp::Logger logger = rclcpp::get_logger("GazeboSimROS2ControlPlugin");
        // Make sure the controller is attached to a valid model
        const auto model = sim::Model(_entity);
        if (!model.Valid(_ecm)) {
            RCLCPP_ERROR(
                logger,
                "[Gazebo ROS 2 Control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
                "Please make sure that Gazebo ROS 2 Control is attached to a valid model.",
                model.Name(_ecm).c_str(), _entity);
            return;
        }

        // Get params from SDF
        auto param_file_name = _sdf->Get<std::string>("parameters");

        if (param_file_name.empty()) {
            RCLCPP_ERROR(
                logger,
                "Gazebo quadruped ros2 control found an empty parameters file. Failed to initialize.");
            return;
        }

        // Get params from SDF
        std::vector<std::string> arguments = {"--ros-args"};

        auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

        sdf::ElementPtr argument_sdf = sdfPtr->GetElement("parameters");
        while (argument_sdf) {
            std::string argument = argument_sdf->Get<std::string>();
            arguments.push_back(RCL_PARAM_FILE_FLAG);
            arguments.push_back(argument);
            argument_sdf = argument_sdf->GetNextElement("parameters");
        }

        // Get controller manager node name
        std::string controllerManagerNodeName{"controller_manager"};

        if (sdfPtr->HasElement("controller_manager_name")) {
            controllerManagerNodeName = sdfPtr->GetElement("controller_manager_name")->Get<std::string>();
        }

        std::string ns = "/";

        if (sdfPtr->HasElement("ros")) {
            sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

            // Set namespace if tag is present
            if (sdfRos->HasElement("namespace")) {
                ns = sdfRos->GetElement("namespace")->Get<std::string>();
                // prevent exception: namespace must be absolute, it must lead with a '/'
                if (ns.empty() || ns[0] != '/') {
                    ns = '/' + ns;
                }
            }

            // Get list of remapping rules from SDF
            if (sdfRos->HasElement("remapping")) {
                sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

                arguments.push_back(RCL_ROS_ARGS_FLAG);
                while (argument_sdf) {
                    auto argument = argument_sdf->Get<std::string>();
                    arguments.push_back(RCL_REMAP_FLAG);
                    arguments.push_back(argument);
                    argument_sdf = argument_sdf->GetNextElement("remapping");
                }
            }
        }

        std::vector<const char *> argv;
        for (const auto &arg: arguments) {
            argv.push_back(arg.data());
        }
        // Create a default context, if not already
        if (!rclcpp::ok()) {
            init(
                static_cast<int>(argv.size()), argv.data(), rclcpp::InitOptions(),
                rclcpp::SignalHandlerOptions::None);
        }

        std::string node_name = "gz_quadruped_control";

        this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns);
        this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        this->dataPtr->executor_->add_node(this->dataPtr->node_);
        auto spin = [this]() {
            this->dataPtr->executor_->spin();
        };
        this->dataPtr->thread_executor_spin_ = std::thread(spin);

        RCLCPP_DEBUG_STREAM(
            this->dataPtr->node_->get_logger(), "[Gazebo Quadruped ROS2 Control] Setting up controller for [" <<
            model.Name(_ecm) << "] (Entity=" << _entity << ")].");

        // Get list of enabled joints
        auto enabledJoints = this->dataPtr->GetEnabledJoints(
            _entity,
            _ecm);

        if (enabledJoints.size() == 0) {
            RCLCPP_DEBUG_STREAM(
                this->dataPtr->node_->get_logger(),
                "[Gazebo Quadruped ROS2 Control] There are no available Joints.");
            return;
        }

        std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
                std::make_unique<GZResourceManager>(this->dataPtr->node_, _ecm, enabledJoints);

        // Create the controller manager
        RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
        rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
        arguments.push_back("-r");
        arguments.push_back("__node:=" + controllerManagerNodeName);
        arguments.push_back("-r");
        arguments.push_back("__ns:=" + ns);
        options.arguments(arguments);
        this->dataPtr->controller_manager_.reset(
            new controller_manager::ControllerManager(
                std::move(resource_manager_),
                this->dataPtr->executor_,
                controllerManagerNodeName,
                this->dataPtr->node_->get_namespace(), options));
        this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

        this->dataPtr->update_rate = this->dataPtr->controller_manager_->get_update_rate();
        this->dataPtr->control_period_ = rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

        // Force setting of use_sim_time parameter
        this->dataPtr->controller_manager_->set_parameter(
            rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

        // Wait for CM to receive robot description from the topic and then initialize Resource Manager
        while (!this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
            RCLCPP_WARN(
                this->dataPtr->node_->get_logger(),
                "Waiting RM to load and initialize hardware...");
            std::this_thread::sleep_for(std::chrono::microseconds(2000000));
        }

        this->dataPtr->entity_ = _entity;
    }

    //////////////////////////////////////////////////
    void GazeboSimQuadrupedPlugin::PreUpdate(
        const sim::UpdateInfo &_info,
        sim::EntityComponentManager & /*_ecm*/) {
        if (!this->dataPtr->controller_manager_) {
            return;
        }
        static bool warned{false};
        if (!warned) {
            rclcpp::Duration gazebo_period(_info.dt);

            // Check the period against the simulation period
            if (this->dataPtr->control_period_ < _info.dt) {
                RCLCPP_ERROR_STREAM(
                    this->dataPtr->node_->get_logger(),
                    "Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
                    " s) is faster than the gazebo simulation period (" <<
                    gazebo_period.seconds() << " s).");
            } else if (this->dataPtr->control_period_ > gazebo_period) {
                RCLCPP_WARN_STREAM(
                    this->dataPtr->node_->get_logger(),
                    " Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
                    " s) is slower than the gazebo simulation period (" <<
                    gazebo_period.seconds() << " s).");
            }
            warned = true;
        }

        const rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            _info.simTime).count(), RCL_ROS_TIME);
        const rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
        // Always set commands on joints, otherwise at low control frequencies the joints tremble
        // as they are updated at a fraction of gazebo sim time
        this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
    }

    //////////////////////////////////////////////////
    void GazeboSimQuadrupedPlugin::PostUpdate(
        const sim::UpdateInfo &_info,
        const sim::EntityComponentManager & /*_ecm*/) {
        if (!this->dataPtr->controller_manager_) {
            return;
        }
        // Get the simulation time and period
        const rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            _info.simTime).count(), RCL_ROS_TIME);
        const rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

        if (sim_period >= this->dataPtr->control_period_) {
            this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
            auto gz_controller_manager =
                    std::dynamic_pointer_cast<GazeboSimSystemInterface>(
                        this->dataPtr->controller_manager_);
            this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
            this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
        }
    }
} // namespace gz_quadruped_hardware

GZ_ADD_PLUGIN(
    gz_quadruped_hardware::GazeboSimQuadrupedPlugin,
    gz::sim::System,
    gz_quadruped_hardware::GazeboSimQuadrupedPlugin::ISystemConfigure,
    gz_quadruped_hardware::GazeboSimQuadrupedPlugin::ISystemPreUpdate,
    gz_quadruped_hardware::GazeboSimQuadrupedPlugin::ISystemPostUpdate)
