//
// Created by tlab-uav on 24-9-26.
//

#include <utility>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <rclcpp/logging.hpp>

namespace ocs2::legged_robot
{
    GaitManager::GaitManager(CtrlInterfaces& ctrl_interfaces,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr)
        : ctrl_interfaces_(ctrl_interfaces),
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),
          target_gait_({0.0, 1.0}, {STANCE})
    {
    }

    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t& /**currentState**/,
                                   const ReferenceManagerInterface& /**referenceManager**/)
    {
        getTargetGait();
        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime,
                                                           timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::init(const std::string& gait_file)
    {
        gait_name_list_.clear();
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        gait_list_.clear();
        for (const auto& name : gait_name_list_)
        {
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    void GaitManager::getTargetGait()
    {
        if (ctrl_interfaces_.control_inputs_.command == 0) return;
        if (ctrl_interfaces_.control_inputs_.command == last_command_) return;
        last_command_ = ctrl_interfaces_.control_inputs_.command;
        const int command = std::max(0, ctrl_interfaces_.control_inputs_.command - 2);
        target_gait_ = gait_list_[command];
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s",
                    gait_name_list_[command].c_str());
        gait_updated_ = true;
    }
}
