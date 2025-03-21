//
// Created by biao on 3/21/25.
//

#pragma once

#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2::legged_robot
{
    class FootPlacementConstraint final : public StateConstraint
    {
    public:
        struct Parameter
        {
            matrix_t a;
            vector_t b;
        };

        FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                                size_t numVertices);

        ~FootPlacementConstraint() override = default;
        FootPlacementConstraint* clone() const override { return new FootPlacementConstraint(*this); }

        bool isActive(scalar_t time) const override;
        size_t getNumConstraints(scalar_t /*time*/) const override { return numVertices_; }
        vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;

    private:
        FootPlacementConstraint(const FootPlacementConstraint& rhs);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        const size_t contactPointIndex_;
        const size_t numVertices_;
    };
} // namespace legged
