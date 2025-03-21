//
// Created by biao on 3/21/25.
//

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ocs2::legged_robot
{
    class SphereVisualization
    {
    public:
        SphereVisualization(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                            const PinocchioSphereInterface& sphereInterface,
                            const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                            scalar_t maxUpdateFrequency = 100.0);
        void update(const SystemObservation& observation);

    private:
        PinocchioInterface pinocchio_interface_;
        const CentroidalModelInfo centroidal_model_info_;
        const PinocchioSphereInterface& sphere_interface_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

        scalar_t last_time_;
        scalar_t min_publish_time_difference_;
    };
} // namespace legged
