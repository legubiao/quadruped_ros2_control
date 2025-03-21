//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

namespace ocs2::legged_robot
{
    SphereSdfConstraint::SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics,
                                             std::shared_ptr<grid_map::SignedDistanceField> sdfPtr)
        : StateConstraint(ConstraintOrder::Linear),
          sphereKinematicsPtr_(sphereKinematics.clone()),
          sdfPtr_(std::move(sdfPtr)),
          numConstraints_(sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal())
    {
    }

    vector_t SphereSdfConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                           const PreComputation& preComp) const
    {
        vector_t value(numConstraints_);

        sphereKinematicsPtr_->setPinocchioInterface(
            cast<PerceptiveLeggedPrecomputation>(preComp).getPinocchioInterface());
        auto position = sphereKinematicsPtr_->getPosition(state);
        auto radius = sphereKinematicsPtr_->getPinocchioSphereInterface().getSphereRadii();
        for (int i = 0; i < numConstraints_; ++i)
        {
            value(i) = sdfPtr_->getDistanceAt(grid_map::Position3(position[i])) - radius[i];
        }
        return value;
    }

    VectorFunctionLinearApproximation SphereSdfConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numConstraints_, state.size(), 0);
        approx.f = getValue(time, state, preComp);

        auto position = sphereKinematicsPtr_->getPosition(state);
        auto sphereApprox = sphereKinematicsPtr_->getPositionLinearApproximation(state);

        for (int i = 0; i < numConstraints_; ++i)
        {
            vector_t sdfGradient = sdfPtr_->getDistanceGradientAt(grid_map::Position3(position[i]));
            approx.dfdx.row(i) = sdfGradient.transpose() * sphereApprox[i].dfdx;
        }
        return approx;
    }

    SphereSdfConstraint::SphereSdfConstraint(const SphereSdfConstraint& rhs)
        : StateConstraint(rhs),
          sphereKinematicsPtr_(rhs.sphereKinematicsPtr_->clone()),
          sdfPtr_(rhs.sdfPtr_),
          numConstraints_(rhs.numConstraints_)
    {
    }
} // namespace legged
