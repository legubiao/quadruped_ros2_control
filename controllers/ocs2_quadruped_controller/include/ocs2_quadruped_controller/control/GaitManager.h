//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "CtrlComponent.h"

namespace ocs2::legged_robot {
    class GaitManager final : public SolverSynchronizedModule {
    public:
        GaitManager(CtrlComponent &ctrl_component,
                    std::shared_ptr<GaitSchedule> gait_schedule_ptr);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t &currentState,
                          const ReferenceManagerInterface &referenceManager) override;

        void postSolverRun(const PrimalSolution &primalSolution) override {
        }

        void init(const std::string &gait_file);

    private:
        void getTargetGait();

        CtrlComponent &ctrl_component_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;

        ModeSequenceTemplate target_gait_;
        bool gait_updated_{false};
        bool verbose_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;
    };
}


#endif //GAITMANAGER_H
