//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::legged_robot
{
    class GaitManager final : public SolverSynchronizedModule
    {
    public:
        GaitManager(CtrlInterfaces& ctrl_interfaces,
                    std::shared_ptr<GaitSchedule> gait_schedule_ptr);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution&/**primalSolution**/) override
        {
        }

        void init(const std::string& gait_file);

    private:
        void getTargetGait();

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;

        ModeSequenceTemplate target_gait_;
        int last_command_ = 0;
        bool gait_updated_{false};
        bool verbose_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;
    };
}


#endif //GAITMANAGER_H
