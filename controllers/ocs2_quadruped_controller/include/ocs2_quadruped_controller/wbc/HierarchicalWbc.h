//
// Created by qiayuan on 22-12-23.
//
#ifndef HIERARCHICALWBC_H
#define HIERARCHICALWBC_H

#include "WbcBase.h"

namespace ocs2::legged_robot
{
    class HierarchicalWbc final : public WbcBase
    {
    public:
        using WbcBase::WbcBase;

        vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                        size_t mode,
                        scalar_t period) override;
    };
} // namespace legged
#endif // HIERARCHICALWBC_H
