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
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>
#else
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#endif

#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include "gz_ros2_control/gz_ros2_control_plugin.hpp"
#include "gz_ros2_control/gz_system.hpp"

namespace gz_quadruped_hardware
{
//////////////////////////////////////////////////
class GazeboSimROS2ControlPluginPrivate
{
public:
  /// \brief Get the URDF XML from the parameter server
  std::string getURDF() const;

  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \return List of entities containing all enabled joints
  std::map<std::string, sim::Entity> GetEnabledJoints(
    const sim::Entity & _entity,
    sim::EntityComponentManager & _ecm) const;

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

  /// \brief Interface loader
  std::shared_ptr<pluginlib::ClassLoader<
      gz_ros2_control::GazeboSimSystemInterface>>
  robot_hw_sim_loader_{nullptr};

  /// \brief Controller manager
  std::shared_ptr<controller_manager::ControllerManager>
  controller_manager_{nullptr};

  /// \brief String with the robot description param_name
  std::string robot_description_ = "robot_description";

  /// \brief String with the name of the node that contains the robot_description
  std::string robot_description_node_ = "robot_state_publisher";

  /// \brief Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ =
    rclcpp::Time((int64_t)0, RCL_ROS_TIME);

  /// \brief ECM pointer
  sim::EntityComponentManager * ecm{nullptr};

  /// \brief controller update rate
  int update_rate;
};

//////////////////////////////////////////////////
std::map<std::string, sim::Entity>
GazeboSimROS2ControlPluginPrivate::GetEnabledJoints(
  const sim::Entity & _entity,
  sim::EntityComponentManager & _ecm) const
{
  std::map<std::string, sim::Entity> output;

  std::vector<std::string> enabledJoints;

  // Get all available joints
  auto jointEntities = _ecm.ChildrenByComponents(_entity, sim::components::Joint());

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto & jointEntity : jointEntities) {
    const auto jointName = _ecm.Component<sim::components::Name>(
      jointEntity)->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto * jointType = _ecm.Component<sim::components::JointType>(jointEntity);
    switch (jointType->Data()) {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
        {
          // Supported joint type
          break;
        }
      case sdf::JointType::FIXED:
        {
          RCLCPP_INFO(
            node_->get_logger(),
            "[gz_ros2_control] Fixed joint [%s] (Entity=%lu)] is skipped",
            jointName.c_str(), jointEntity);
          continue;
        }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "[gz_ros2_control] Joint [%s] (Entity=%lu)] is of unsupported type."
            " Only joints with a single axis are supported.",
            jointName.c_str(), jointEntity);
          continue;
        }
      default:
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "[gz_ros2_control] Joint [%s] (Entity=%lu)] is of unknown type",
            jointName.c_str(), jointEntity);
          continue;
        }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
std::string GazeboSimROS2ControlPluginPrivate::getURDF() const
{
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, robot_description_node_);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
        robot_description_node_.c_str());
      return 0;
    }
    RCLCPP_ERROR(
      node_->get_logger(), "%s service not available, waiting again...",
      robot_description_node_.c_str());
  }

  RCLCPP_INFO(
    node_->get_logger(), "connected to service!! %s asking for %s",
    robot_description_node_.c_str(),
    this->robot_description_.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    RCLCPP_DEBUG(
      node_->get_logger(), "param_name %s",
      this->robot_description_.c_str());

    try {
      auto f = parameters_client->get_parameters({this->robot_description_});
      f.wait();
      std::vector<rclcpp::Parameter> values = f.get();
      urdf_string = values[0].as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "gz_ros2_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.",
        this->robot_description_.c_str());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

  return urdf_string;
}

//////////////////////////////////////////////////
GazeboSimROS2ControlPlugin::GazeboSimROS2ControlPlugin()
: dataPtr(std::make_unique<GazeboSimROS2ControlPluginPrivate>())
{
}

//////////////////////////////////////////////////
GazeboSimROS2ControlPlugin::~GazeboSimROS2ControlPlugin()
{
  // Stop controller manager thread
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::Configure(
  const sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  sim::EntityComponentManager & _ecm,
  sim::EventManager &)
{
  rclcpp::Logger logger = rclcpp::get_logger("GazeboSimROS2ControlPlugin");
  // Make sure the controller is attached to a valid model
  const auto model = sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    RCLCPP_ERROR(
      logger,
      "[gz_ros2_control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
      "Please make sure that gz_ros2_control is attached to a valid model.",
      model.Name(_ecm).c_str(), _entity);
    return;
  }

  // Get params from SDF
  std::string paramFileName = _sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    RCLCPP_ERROR(
      logger,
      "gz_ros2_control found an empty parameters file. Failed to initialize.");
    return;
  }

  // Get params from SDF
  std::string robot_param_node = _sdf->Get<std::string>("robot_param_node");
  if (!robot_param_node.empty()) {
    this->dataPtr->robot_description_node_ = robot_param_node;
  }
  RCLCPP_INFO(
    logger,
    "robot_param_node is %s", this->dataPtr->robot_description_node_.c_str());

  std::string robot_description = _sdf->Get<std::string>("robot_param");
  if (!robot_description.empty()) {
    this->dataPtr->robot_description_ = robot_description;
  }
  RCLCPP_INFO(
    logger,
    "robot_param_node is %s", this->dataPtr->robot_description_.c_str());

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
      if (ns.length() > 1) {
        this->dataPtr->robot_description_node_ = ns + "/" + this->dataPtr->robot_description_node_;
      }
    }

    // Get list of remapping rules from SDF
    if (sdfRos->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("remapping");
      }
    }
  }

  // Create a default context, if not already
  if (!rclcpp::ok()) {
    RCLCPP_DEBUG_STREAM(logger, "Create default context");
    std::vector<const char *> argv;
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  std::string node_name = "gz_ros2_control";
  this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns);
  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->node_);
  auto spin = [this]()
    {
      this->dataPtr->executor_->spin();
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(logger, "Create node " << node_name);

  // Read urdf from ros parameter server
  const auto urdf_string = this->dataPtr->getURDF();
  if (urdf_string.empty()) {
    RCLCPP_ERROR_STREAM(this->dataPtr->node_->get_logger(), "An empty URDF was passed. Exiting.");
    return;
  }

  // set the robot description as argument to propagate it among controller manager and controllers
  // Remove all comments via regex pattern to match XML comments, including newlines
  const std::regex comment_pattern(R"(<!--[\s\S]*?-->)");
  const auto rb_arg = std::string("robot_description:=") + std::regex_replace(
    urdf_string,
    comment_pattern, "");
  arguments.push_back(RCL_PARAM_FLAG);
  arguments.push_back(rb_arg);

  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }

  // set the arguments into rcl context
  rcl_arguments_t rcl_args = rcl_get_zero_initialized_arguments();
  rcl_ret_t rcl_ret = rcl_parse_arguments(
    static_cast<int>(argv.size()),
    argv.data(), rcl_get_default_allocator(), &rcl_args);
  auto rcl_context =
    this->dataPtr->node_->get_node_base_interface()->get_context()->get_rcl_context();
  rcl_context->global_arguments = rcl_args;
  if (rcl_ret != RCL_RET_OK) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node_->get_logger(), "Argument parser error: " << rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  if (rcl_arguments_get_param_files_count(&rcl_args) < 1) {
    RCLCPP_ERROR(this->dataPtr->node_->get_logger(), "Failed to parse input yaml file(s)");
    return;
  }

  RCLCPP_DEBUG_STREAM(
    this->dataPtr->node_->get_logger(), "[gz_ros2_control] Setting up controller for [" <<
      model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  // Get list of enabled joints
  auto enabledJoints = this->dataPtr->GetEnabledJoints(
    _entity,
    _ecm);

  if (enabledJoints.size() == 0) {
    RCLCPP_DEBUG_STREAM(
      this->dataPtr->node_->get_logger(),
      "[gz_ros2_control] There are no available Joints.");
    return;
  }

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node_->get_logger(),
      "Error parsing URDF in gz_ros2_control plugin, plugin not active : " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<hardware_interface::ResourceManager>();

  try {
    resource_manager_->load_urdf(urdf_string, false, false);
  } catch (...) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Error initializing URDF to resource manager!");
  }
  try {
    this->dataPtr->robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>(
        "gz_ros2_control",
        "gz_ros2_control::GazeboSimSystemInterface"));
  } catch (pluginlib::LibraryLoadException & ex) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Failed to create robot simulation interface loader: %s ",
      ex.what());
    return;
  }

  for (unsigned int i = 0; i < control_hardware_info.size(); ++i) {
    std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_class_type;
    std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface> gzSimSystem;
    RCLCPP_DEBUG(
      this->dataPtr->node_->get_logger(), "Load hardware interface %s ...",
      robot_hw_sim_type_str_.c_str());

    try {
      gzSimSystem = std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>(
        this->dataPtr->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "The plugin failed to load for some reason. Error: %s\n",
        ex.what());
      continue;
    }
    if (!gzSimSystem->initSim(
        this->dataPtr->node_,
        enabledJoints,
        control_hardware_info[i],
        _ecm,
        this->dataPtr->update_rate))
    {
      RCLCPP_FATAL(
        this->dataPtr->node_->get_logger(), "Could not initialize robot simulation interface");
      return;
    }
    RCLCPP_DEBUG(
      this->dataPtr->node_->get_logger(), "Initialized robot simulation interface %s!",
      robot_hw_sim_type_str_.c_str());

    resource_manager_->import_component(std::move(gzSimSystem), control_hardware_info[i]);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager_->set_component_state(control_hardware_info[i].name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
  this->dataPtr->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      this->dataPtr->executor_,
      controllerManagerNodeName,
      this->dataPtr->node_->get_namespace()));
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  if (!this->dataPtr->controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node_->get_logger(),
      "controller manager doesn't have an update_rate parameter");
    return;
  }

  this->dataPtr->update_rate =
    this->dataPtr->controller_manager_->get_parameter("update_rate").as_int();
  this->dataPtr->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Force setting of use_sim_time parameter
  this->dataPtr->controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::PreUpdate(
  const sim::UpdateInfo & _info,
  sim::EntityComponentManager & /*_ecm*/)
{
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

  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::PostUpdate(
  const sim::UpdateInfo & _info,
  const sim::EntityComponentManager & /*_ecm*/)
{
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  // Get the simulation time and period
  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  if (sim_period >= this->dataPtr->control_period_) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    auto gz_controller_manager =
      std::dynamic_pointer_cast<gz_ros2_control::GazeboSimSystemInterface>(
      this->dataPtr->controller_manager_);
    this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }
}
}  // namespace gz_ros2_control

#ifdef GZ_HEADERS
GZ_ADD_PLUGIN(
  gz_ros2_control::GazeboSimROS2ControlPlugin,
  sim::System,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemConfigure,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPreUpdate,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(
  gz_ros2_control::GazeboSimROS2ControlPlugin,
  "ign_ros2_control::IgnitionROS2ControlPlugin")
#else
IGNITION_ADD_PLUGIN(
  gz_ros2_control::GazeboSimROS2ControlPlugin,
  sim::System,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemConfigure,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPreUpdate,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(
  gz_ros2_control::GazeboSimROS2ControlPlugin,
  "ign_ros2_control::IgnitionROS2ControlPlugin")
#endif
