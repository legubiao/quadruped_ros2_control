//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"

#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot
{
    class PerceptiveLeggedReferenceManager : public SwitchedModelReferenceManager
    {
    public:
        PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                         std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                         scalar_t comHeight);

        const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

        contact_flag_t getFootPlacementFlags(scalar_t time) const;

    protected:
        void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                              TargetTrajectories& targetTrajectories,
                              ModeSchedule& modeSchedule) override;

        virtual void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                  ModeSchedule& modeSchedule);

        void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                               const std::vector<bool>& contactFlagStocks,
                               std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

        std::pair<scalar_array_t, scalar_array_t> getHeights(const std::vector<bool>& contactFlagStocks,
                                                             const std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections);

        const CentroidalModelInfo info_;
        feet_array_t<vector3_t> lastLiftoffPos_;

        std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        scalar_t comHeight_;
    };
} // namespace legged
