//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

namespace ocs2::legged_robot
{
    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface,
                                                                   const CentroidalModelInfo& info,
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                   ModelSettings settings,
                                                                   const ConvexRegionSelector& convexRegionSelector)
        : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
          convexRegionSelectorPtr_(&convexRegionSelector)
    {
        footPlacementConParameters_.resize(info.numThreeDofContacts);
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
        : LeggedRobotPreComputation(rhs), convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_)
    {
        footPlacementConParameters_.resize(rhs.footPlacementConParameters_.size());
    }

    void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
    {
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
        {
            return;
        }
        LeggedRobotPreComputation::request(request, t, x, u);

        if (request.contains(Request::Constraint))
        {
            for (size_t i = 0; i < footPlacementConParameters_.size(); i++)
            {
                FootPlacementConstraint::Parameter params;

                auto projection = convexRegionSelectorPtr_->getProjection(i, t);
                if (projection.regionPtr == nullptr)
                {
                    // Swing leg
                    continue;
                }

                matrix_t polytopeA;
                vector_t polytopeB;
                std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegionSelectorPtr_->getConvexPolygon(i, t));
                matrix_t p = (matrix_t(2, 3) << // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
                params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
                params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().
                                                              head(2);

                footPlacementConParameters_[i] = params;
            }
        }
    }

    std::pair<matrix_t, vector_t> PerceptiveLeggedPrecomputation::getPolygonConstraint(
        const convex_plane_decomposition::CgalPolygon2d& polygon) const
    {
        size_t numVertices = polygon.size();
        matrix_t polytopeA = matrix_t::Zero(numVertices, 2);
        vector_t polytopeB = vector_t::Zero(numVertices);

        for (size_t i = 0; i < numVertices; i++)
        {
            size_t j = i + 1;
            if (j == numVertices)
            {
                j = 0;
            }
            size_t k = j + 1;
            if (k == numVertices)
            {
                k = 0;
            }
            const auto point_a = polygon.vertex(i);
            const auto point_b = polygon.vertex(j);
            const auto point_c = polygon.vertex(k);

            polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
            polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
            if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0)
            {
                polytopeA.row(i) *= -1;
                polytopeB(i) *= -1;
            }
        }

        return {polytopeA, polytopeB};
    }
} // namespace legged
