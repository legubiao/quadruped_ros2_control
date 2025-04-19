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

#include <memory>

#ifdef GZ_HEADERS
#include <gz/sim/System.hh>
namespace sim = gz::sim;
#else
#include <ignition/gazebo/System.hh>
namespace sim = ignition::gazebo;
#endif

namespace gz_quadruped_hardware
{
    // Forward declarations.
    class GazeboSimQuadrupedPluginPrivate;

    class GazeboSimQuadrupedPlugin
        : public sim::System,
          public sim::ISystemConfigure,
          public sim::ISystemPreUpdate,
          public sim::ISystemPostUpdate
    {
    public:
        /// \brief Constructor
        GazeboSimQuadrupedPlugin();

        /// \brief Destructor
        ~GazeboSimQuadrupedPlugin() override;

        // Documentation inherited
        void Configure(
            const sim::Entity& _entity,
            const std::shared_ptr<const sdf::Element>& _sdf,
            sim::EntityComponentManager& _ecm,
            sim::EventManager& _eventMgr) override;

        // Documentation inherited
        void PreUpdate(
            const sim::UpdateInfo& _info,
            sim::EntityComponentManager& _ecm) override;

        void PostUpdate(
            const sim::UpdateInfo& _info,
            const sim::EntityComponentManager& _ecm) override;

    private:
        /// \brief Private data pointer.
        std::unique_ptr<GazeboSimQuadrupedPluginPrivate> dataPtr;
    };
}
