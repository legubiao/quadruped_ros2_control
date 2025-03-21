//
// Created by biao on 3/21/25.
//

#pragma once

#include <ocs2_quadruped_controller/interface/LeggedRobotPreComputation.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"

namespace ocs2::legged_robot
{
    /** Callback for caching and reference update */
    class PerceptiveLeggedPrecomputation : public LeggedRobotPreComputation
    {
    public:
        PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                       const ConvexRegionSelector& convexRegionSelector);
        ~PerceptiveLeggedPrecomputation() override = default;

        PerceptiveLeggedPrecomputation* clone() const override { return new PerceptiveLeggedPrecomputation(*this); }

        void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

        const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const
        {
            return footPlacementConParameters_;
        }

        PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs);

    private:
        std::pair<matrix_t, vector_t> getPolygonConstraint(
            const convex_plane_decomposition::CgalPolygon2d& polygon) const;

        const ConvexRegionSelector* convexRegionSelectorPtr_;

        std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
    };
} // namespace ocs2::legged_robot
