//
// Created by biao on 3/21/25.
//

#include <ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>

namespace ocs2::legged_robot
{
    ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                               std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                               planarTerrainPtr,
                                               const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                               size_t numVertices)
        : info_(std::move(info)),
          numVertices_(numVertices),
          planarTerrainPtr_(std::move(planarTerrainPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone())
    {
    }

    convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(
        size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return feetProjections_[leg][index];
    }

    convex_plane_decomposition::CgalPolygon2d ConvexRegionSelector::getConvexPolygon(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return convexPolygons_[leg][index];
    }

    vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return nominalFootholds_[leg][index];
    }

    void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                                      TargetTrajectories& targetTrajectories)
    {
        planarTerrain_ = *planarTerrainPtr_;
        // Need copy storage it since PlanarTerrainProjection.regionPtr is a pointer
        const auto& modeSequence = modeSchedule.modeSequence;
        const auto& eventTimes = modeSchedule.eventTimes;
        const auto contactFlagStocks = extractContactFlags(modeSequence);
        const size_t numPhases = modeSequence.size();

        // Find start and final index of time for legs
        feet_array_t<std::vector<int>> startIndices;
        feet_array_t<std::vector<int>> finalIndices;
        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            startIndices[leg] = std::vector<int>(numPhases, 0);
            finalIndices[leg] = std::vector<int>(numPhases, 0);
            // find the startTime and finalTime indices for swing feet
            for (size_t i = 0; i < numPhases; i++)
            {
                // skip if it is a stance leg
                if (contactFlagStocks[leg][i])
                {
                    std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            feetProjections_[leg].clear();
            convexPolygons_[leg].clear();
            nominalFootholds_[leg].clear();
            feetProjections_[leg].resize(numPhases);
            convexPolygons_[leg].resize(numPhases);
            nominalFootholds_[leg].resize(numPhases);
            middleTimes_[leg].clear();
            initStandFinalTime_[leg] = 0;

            scalar_t lastStandMiddleTime = NAN;
            // Stand leg foot
            for (size_t i = 0; i < numPhases; ++i)
            {
                if (contactFlagStocks[leg][i])
                {
                    const int standStartIndex = startIndices[leg][i];
                    const int standFinalIndex = finalIndices[leg][i];
                    const scalar_t standStartTime = eventTimes[standStartIndex];
                    const scalar_t standFinalTime = eventTimes[standFinalIndex];
                    const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2;

                    if (!numerics::almost_eq(standMiddleTime, lastStandMiddleTime))
                    {
                        vector3_t footPos = getNominalFoothold(leg, standMiddleTime, initState, targetTrajectories);
                        auto penaltyFunction = [](const vector3_t& /*projectedPoint*/) { return 0.0; };
                        const auto projection = getBestPlanarRegionAtPositionInWorld(
                            footPos, planarTerrain_.planarRegions, penaltyFunction);
                        scalar_t growthFactor = 1.05;
                        const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                            projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame,
                            numVertices_, growthFactor);

                        feetProjections_[leg][i] = projection;
                        convexPolygons_[leg][i] = convexRegion;
                        nominalFootholds_[leg][i] = footPos;
                        middleTimes_[leg].push_back(standMiddleTime);
                    }
                    else
                    {
                        feetProjections_[leg][i] = feetProjections_[leg][i - 1];
                        convexPolygons_[leg][i] = convexPolygons_[leg][i - 1];
                        nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
                    }

                    if (standStartTime < initTime && initTime < standFinalTime)
                    {
                        initStandFinalTime_[leg] = standFinalTime;
                    }
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            timeEvents_[leg] = eventTimes;
        }
    }

    feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(
        const std::vector<size_t>& phaseIDsStock) const
    {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool>> contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++)
        {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < info_.numThreeDofContacts; j++)
            {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }

    std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        if (!contactFlagStock[index])
        {
            return {0, 0};
        }

        // find the starting time
        int startTimesIndex = 0;
        for (int ip = index - 1; ip >= 0; ip--)
        {
            if (!contactFlagStock[ip])
            {
                startTimesIndex = ip;
                break;
            }
        }
        // find the final time
        int finalTimesIndex = numPhases - 2;
        for (size_t ip = index + 1; ip < numPhases; ip++)
        {
            if (!contactFlagStock[ip])
            {
                finalTimesIndex = ip - 1;
                break;
            }
        }
        return {startTimesIndex, finalTimesIndex};
    }

    vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                                       TargetTrajectories& targetTrajectories)
    {
        scalar_t height = 0.4;

        vector_t desiredState = targetTrajectories.getDesiredState(time);
        vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
        vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

        auto feedback = (vector3_t() << (std::sqrt(height / 9.81) * (measuredVel - desiredVel)).head(2), 0).finished();
        vector_t zyx = centroidal_model::getBasePose(desiredState, info_).tail(3);
        scalar_t offset = tan(-zyx(1)) * height;
        vector3_t offsetVector(offset * cos(-zyx(1)), 0, offset * sin(-zyx(1)));
        matrix3_t R;
        scalar_t z = zyx(0);
        R << cos(z), -sin(z), 0, // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
        //  return endEffectorKinematicsPtr_->getPosition(targetTrajectories.getDesiredState(time))[leg] + feedback;
        return endEffectorKinematicsPtr_->getPosition(targetTrajectories.getDesiredState(time))[leg] - R.transpose() *
            offsetVector;
    }
} // namespace legged
