//
// Created by biao on 3/21/25.
//
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"

namespace ocs2::legged_robot
{
    FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                     std::shared_ptr<grid_map::SignedDistanceField> sdfPtr,
                                                     size_t contactPointIndex,
                                                     scalar_t clearance)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          sdfPtr_(std::move(sdfPtr)),
          contactPointIndex_(contactPointIndex),
          clearance_(clearance)
    {
    }

    FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& rhs)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          sdfPtr_(rhs.sdfPtr_),
          contactPointIndex_(rhs.contactPointIndex_),
          clearance_(rhs.clearance_)
    {
    }

    bool FootCollisionConstraint::isActive(scalar_t time) const
    {
        scalar_t offset = 0.05;
        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&
            !referenceManagerPtr_->getContactFlags(time + 0.5 * offset)[contactPointIndex_] &&
            !referenceManagerPtr_->getContactFlags(time - offset)[contactPointIndex_];
    }

    vector_t FootCollisionConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                               const PreComputation& /*preComp*/) const
    {
        vector_t value(1);
        value(0) = sdfPtr_->getDistanceAt(grid_map::Position3(endEffectorKinematicsPtr_->getPosition(state).front())) -
            clearance_;
        return value;
    }

    VectorFunctionLinearApproximation FootCollisionConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComp) const
    {
        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
        approx.f = getValue(time, state, preComp);
        approx.dfdx = sdfPtr_->getDistanceGradientAt(grid_map::Position3(endEffectorKinematicsPtr_->getPosition(state).front())).
                               transpose() *
            endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front().dfdx;
        return approx;
    }
} // namespace legged
