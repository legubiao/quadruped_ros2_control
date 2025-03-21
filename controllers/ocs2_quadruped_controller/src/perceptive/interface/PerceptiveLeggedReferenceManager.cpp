//
// Created by biao on 3/21/25.
//

#include <utility>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info,
                                                                       std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                       std::shared_ptr<SwingTrajectoryPlanner>
                                                                       swingTrajectoryPtr,
                                                                       std::shared_ptr<ConvexRegionSelector>
                                                                       convexRegionSelectorPtr,
                                                                       const EndEffectorKinematics<scalar_t>&
                                                                       endEffectorKinematics,
                                                                       scalar_t comHeight)
        : info_(std::move(info)),
          SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
          convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          comHeight_(comHeight)
    {
    }

    void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                            const vector_t& initState,
                                                            TargetTrajectories& targetTrajectories,
                                                            ModeSchedule& modeSchedule)
    {
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

        TargetTrajectories newTargetTrajectories;
        int nodeNum = 11;
        for (size_t i = 0; i < nodeNum; ++i)
        {
            scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
            vector_t state = targetTrajectories.getDesiredState(time);
            vector_t input = targetTrajectories.getDesiredState(time);

            const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
            vector_t pos = centroidal_model::getBasePose(state, info_).head(3);

            // Base Orientation
            scalar_t step = 0.3;
            grid_map::Vector3 normalVector;
            normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                    map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                (2 * step);
            normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                    map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                (2 * step);
            normalVector(2) = 1;
            normalVector.normalize();
            matrix3_t R;
            scalar_t z = centroidal_model::getBasePose(state, info_)(3);
            R << cos(z), -sin(z), 0, // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
            vector_t v = R.transpose() * normalVector;
            centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

            // Base Z Position
            centroidal_model::getBasePose(state, info_)(2) =
                map.atPosition("smooth_planar", pos) + comHeight_ / cos(centroidal_model::getBasePose(state, info_)(4));

            newTargetTrajectories.timeTrajectory.push_back(time);
            newTargetTrajectories.stateTrajectory.push_back(state);
            newTargetTrajectories.inputTrajectory.push_back(input);
        }
        targetTrajectories = newTargetTrajectories;

        // Footstep
        convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

        // Swing trajectory
        updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
    }

    void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                        ModeSchedule& modeSchedule)
    {
        const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
        feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

            auto projections = convexRegionSelectorPtr_->getProjections(leg);
            modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

            scalar_array_t liftOffHeights, touchDownHeights;
            std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], projections);
            liftOffHeightSequence[leg] = liftOffHeights;
            touchDownHeightSequence[leg] = touchDownHeights;
        }
        swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    }

    void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg,
                                                             size_t initIndex,
                                                             const std::vector<bool>& contactFlagStocks,
                                                             std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections)
    {
        if (contactFlagStocks[initIndex])
        {
            lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
            lastLiftoffPos_[leg].z() -= 0.02;
            for (int i = initIndex; i < projections.size(); ++i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
            for (int i = initIndex; i >= 0; --i)
            {
                if (!contactFlagStocks[i])
                {
                    break;
                }
                projections[i].positionInWorld = lastLiftoffPos_[leg];
            }
        }
        if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg])
        {
            for (int i = initIndex; i >= 0; --i)
            {
                if (contactFlagStocks[i])
                {
                    projections[i].positionInWorld = lastLiftoffPos_[leg];
                }
                if (!contactFlagStocks[i] && !contactFlagStocks[i + 1])
                {
                    break;
                }
            }
        }
        //    for (int i = 0; i < numPhases; ++i) {
        //      if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
        //    }
        //    std::cerr << std::endl;
    }

    std::pair<scalar_array_t, scalar_array_t> PerceptiveLeggedReferenceManager::getHeights(
        const std::vector<bool>& contactFlagStocks,
        const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections)
    {
        scalar_array_t liftOffHeights, touchDownHeights;
        const size_t numPhases = projections.size();

        liftOffHeights.clear();
        liftOffHeights.resize(numPhases);
        touchDownHeights.clear();
        touchDownHeights.resize(numPhases);

        for (size_t i = 1; i < numPhases; ++i)
        {
            if (!contactFlagStocks[i])
            {
                liftOffHeights[i] = contactFlagStocks[i - 1]
                                        ? projections[i - 1].positionInWorld.z()
                                        : liftOffHeights[i - 1];
            }
        }
        for (int i = numPhases - 2; i >= 0; --i)
        {
            if (!contactFlagStocks[i])
            {
                touchDownHeights[i] = contactFlagStocks[i + 1]
                                          ? projections[i + 1].positionInWorld.z()
                                          : touchDownHeights[i + 1];
            }
        }

        //  for (int i = 0; i < numPhases; ++i) {
        //    std::cerr << std::setprecision(3) << liftOffHeights[i] << "\t";
        //  }
        //  std::cerr << std::endl;
        //  for (int i = 0; i < numPhases; ++i) {
        //    std::cerr << std::setprecision(3) << contactFlagStocks[i] << "\t";
        //  }
        //  std::cerr << std::endl;

        return {liftOffHeights, touchDownHeights};
    }

    contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const
    {
        contact_flag_t flag;
        const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
        for (int i = 0; i < flag.size(); ++i)
        {
            flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
        }
        return flag;
    }
} // namespace legged
