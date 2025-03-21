//
// Created by biao on 3/21/25.
//

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_quadruped_controller/perceptive/constraint/FootCollisionConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/SphereSdfConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <memory>

namespace ocs2::legged_robot
{
    void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                               const std::string& referenceFile, bool verbose)
    {
        planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

        double width{5.0}, height{5.0};
        convex_plane_decomposition::PlanarRegion plannerRegion;
        plannerRegion.transformPlaneToWorld.setIdentity();
        plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2);
        convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));
        boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));
        plannerRegion.boundaryWithInset.boundary = boundary;
        convex_plane_decomposition::CgalPolygonWithHoles2d insets;
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, +width / 2 - 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, -width / 2 + 0.01));
        insets.outer_boundary().push_back(
            convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, -width / 2 + 0.01));
        plannerRegion.boundaryWithInset.insets.push_back(insets);
        planarTerrainPtr_->planarRegions.push_back(plannerRegion);

        std::string layer = "elevation_before_postprocess";
        planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
        planarTerrainPtr_->gridMap.add(layer, 0);
        planarTerrainPtr_->gridMap.add("smooth_planar", 0);
        signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>();
        signedDistanceFieldPtr_->calculateSignedDistanceField(planarTerrainPtr_->gridMap, layer, 0.1);

        LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

        for (size_t i = 0; i < centroidal_model_info_.numThreeDofContacts; i++)
        {
            const std::string& footName = modelSettings().contactNames3DoF[i];
            std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

            std::unique_ptr<PenaltyBase> placementPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
            std::unique_ptr<PenaltyBase> collisionPenalty(
                new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

            // For foot placement
            std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
                new FootPlacementConstraint(*reference_manager_ptr_, *eeKinematicsPtr, i, numVertices_));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footPlacement",
                std::make_unique<StateSoftConstraint>(std::move(footPlacementConstraint), std::move(placementPenalty)));

            // For foot Collision
            std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
                new FootCollisionConstraint(*reference_manager_ptr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i,
                                            0.03));
            problem_ptr_->stateSoftConstraintPtr->add(
                footName + "_footCollision",
                std::make_unique<StateSoftConstraint>(std::move(footCollisionConstraint), std::move(collisionPenalty)));
        }

        // For collision avoidance
        scalar_t thighExcess = 0.025;
        scalar_t calfExcess = 0.02;

        std::vector<std::string> collisionLinks = {"LF_calf", "RF_calf", "LH_calf", "RH_calf"};
        const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess, calfExcess, calfExcess};

        pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(
            *pinocchio_interface_ptr_, collisionLinks, maxExcesses, 0.6);

        CentroidalModelPinocchioMapping pinocchioMapping(centroidal_model_info_);
        auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(
            *pinocchioSphereInterfacePtr_, pinocchioMapping);

        std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
            new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));

        //  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
        //  problem_ptr_->stateSoftConstraintPtr->add(
        //      "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
    }

    void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                          const std::string& referenceFile, bool verbose)
    {
        auto swingTrajectoryPlanner =
            std::make_unique<SwingTrajectoryPlanner>(
                loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr(
            {model_settings_.contactNames3DoF}, "ALL_FOOT");
        auto convexRegionSelector =
            std::make_unique<ConvexRegionSelector>(centroidal_model_info_, planarTerrainPtr_, *eeKinematicsPtr,
                                                   numVertices_);

        scalar_t comHeight = 0;
        loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
        reference_manager_ptr_.reset(new PerceptiveLeggedReferenceManager(
            centroidal_model_info_, loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
            *eeKinematicsPtr, comHeight));
    }

    void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/,
                                                        const std::string& /*urdfFile*/,
                                                        const std::string& /*referenceFile*/, bool /*verbose*/)
    {
        problem_ptr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
            *pinocchio_interface_ptr_, centroidal_model_info_, *reference_manager_ptr_->getSwingTrajectoryPlanner(),
            model_settings_,
            *dynamic_cast<PerceptiveLeggedReferenceManager&>(*reference_manager_ptr_).getConvexRegionSelectorPtr());
    }
} // namespace legged
