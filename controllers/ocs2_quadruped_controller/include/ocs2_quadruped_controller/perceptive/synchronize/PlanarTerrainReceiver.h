//
// Created by biao on 3/21/25.
//

#ifndef PLANARTERRAINRECEIVER_H
#define PLANARTERRAINRECEIVER_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::legged_robot
{
    class PlanarTerrainReceiver : public SolverSynchronizedModule
    {
    public:
        PlanarTerrainReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                              const std::shared_ptr<convex_plane_decomposition::PlanarTerrain>& planarTerrainPtr,
                              const std::shared_ptr<grid_map::SignedDistanceField>& signedDistanceFieldPtr,
                              const std::string& mapTopic,
                              const std::string& sdfElevationLayer);

        void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution& primalSolution) override
        {
        }

    private:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr subscription_;

        convex_plane_decomposition::PlanarTerrain planarTerrain_;

        std::string sdfElevationLayer_;

        std::mutex mutex_;
        std::atomic_bool updated_ = false;

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
    };
}


#endif //PLANARTERRAINRECEIVER_H
