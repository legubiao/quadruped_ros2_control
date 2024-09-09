//
// Created by tlab-uav on 24-9-6.
//

#include <quadruped_ros2_control/FSM/FSMState.h>

#include <utility>

FSMState::FSMState(const FSMStateName stateName, std::string stateNameString)
    : state_name(stateName), state_name_string(std::move(stateNameString)) {
}
