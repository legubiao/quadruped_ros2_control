//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_quadruped_controller/wbc/HierarchicalWbc.h"
#include "ocs2_quadruped_controller/wbc/HoQp.h"

namespace ocs2::legged_robot
{
    vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                     const vector_t& rbdStateMeasured, size_t mode,
                                     scalar_t period)
    {
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
            formulateNoContactMotionTask();
        Task task1 = formulateBaseAccelTask(stateDesired, inputDesired, period) + formulateSwingLegTask();
        Task task2 = formulateContactForceTask(inputDesired);
        HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

        return hoQp.getSolutions();
    }
} // namespace legged
