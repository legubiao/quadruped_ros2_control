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

#ifdef GZ_HEADERS
#include <gz/sim/System.hh>
namespace sim = gz::sim;
#else
#include <ignition/gazebo/System.hh>
namespace sim = ignition::gazebo;
#endif

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gz_quadruped_hardware
{
    /// \brief This class allows us to handle flags easily, instead of using strings
    ///
    /// For example
    /// enum ControlMethod_
    /// {
    ///   NONE      = 0,
    ///   POSITION  = (1 << 0),
    ///   VELOCITY  = (1 << 1),
    ///   EFFORT    = (1 << 2),
    /// };
    /// typedef SafeEnum<enum ControlMethod_> ControlMethod;
    ///
    /// ControlMethod foo;
    /// foo |= POSITION  // Foo has the position flag active
    /// foo & POSITION -> True  // Check if position is active in the flag
    /// foo & VELOCITY -> False  // Check if velocity is active in the flag

    template <class ENUM, class UNDERLYING = typename std::underlying_type<ENUM>::type>
    class SafeEnum
    {
    public:
        SafeEnum()
            : mFlags(0)
        {
        }

        explicit SafeEnum(ENUM singleFlag)
            : mFlags(singleFlag)
        {
        }

        SafeEnum(const SafeEnum& original)
            : mFlags(original.mFlags)
        {
        }

        SafeEnum& operator|=(ENUM addValue)
        {
            mFlags |= addValue;
            return *this;
        }

        SafeEnum operator|(ENUM addValue)
        {
            SafeEnum result(*this);
            result |= addValue;
            return result;
        }

        SafeEnum& operator&=(ENUM maskValue)
        {
            mFlags &= maskValue;
            return *this;
        }

        SafeEnum operator&(ENUM maskValue)
        {
            SafeEnum result(*this);
            result &= maskValue;
            return result;
        }

        SafeEnum operator~()
        {
            SafeEnum result(*this);
            result.mFlags = ~result.mFlags;
            return result;
        }

        explicit operator bool() { return mFlags != 0; }

    protected:
        UNDERLYING mFlags;
    };

    // SystemInterface provides API-level access to read and command joint properties.
    class GazeboSimSystemInterface
        : public hardware_interface::SystemInterface
    {
    public:
        /// \brief Initialize the system interface
        /// param[in] model_nh Pointer to the ros2 node
        /// param[in] joints Map with the name of the joint as the key and the value is
        /// related with the entity in Gazebo
        /// param[in] hardware_info structure with data from URDF.
        /// param[in] _ecm Entity-component manager.
        /// param[in] update_rate controller update rate
        virtual bool initSim(
            rclcpp::Node::SharedPtr& model_nh,
            std::map<std::string, sim::Entity>& joints,
            const hardware_interface::HardwareInfo& hardware_info,
            sim::EntityComponentManager& _ecm,
            int& update_rate) = 0;

        // Methods used to control a joint.
        enum ControlMethod_
        {
            NONE = 0,
            POSITION = (1 << 0),
            VELOCITY = (1 << 1),
            EFFORT = (1 << 2),
        };

        typedef SafeEnum<enum ControlMethod_> ControlMethod;

    protected:
        rclcpp::Node::SharedPtr nh_;
    };
}
