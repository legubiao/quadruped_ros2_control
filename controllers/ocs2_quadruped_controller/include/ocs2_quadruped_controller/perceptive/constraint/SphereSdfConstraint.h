//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace ocs2::legged_robot
{
    class SphereSdfConstraint final : public StateConstraint
    {
    public:
        SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics,
                            std::shared_ptr<grid_map::SignedDistanceField> sdfPtr);

        /** Default destructor */
        ~SphereSdfConstraint() override = default;

        SphereSdfConstraint* clone() const override { return new SphereSdfConstraint(*this); }

        size_t getNumConstraints(scalar_t /*time*/) const override { return numConstraints_; }

        vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;

    private:
        SphereSdfConstraint(const SphereSdfConstraint& rhs);

        std::unique_ptr<PinocchioSphereKinematics> sphereKinematicsPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
        size_t numConstraints_{};
    };
} // namespace legged
