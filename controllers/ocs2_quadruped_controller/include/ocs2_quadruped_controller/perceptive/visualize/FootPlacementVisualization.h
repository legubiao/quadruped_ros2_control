//
// Created by biao on 3/21/25.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

#include <visualization_msgs/msg/marker.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ocs2::legged_robot
{
    class FootPlacementVisualization
    {
    public:
        FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot,
                                   const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                   scalar_t maxUpdateFrequency = 20.0);

        void update(const SystemObservation& observation);

    private:
        visualization_msgs::msg::Marker to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                                      const Eigen::Isometry3d& transformPlaneToWorld,
                                                      const std_msgs::msg::Header& header, Color color,
                                                      float alpha, size_t i) const;

        scalar_t line_width_ = 0.008;
        scalar_t foot_marker_diameter_ = 0.02;
        std::vector<Color> feet_color_map_ = {Color::blue, Color::orange, Color::yellow, Color::purple};

        const ConvexRegionSelector& convex_region_selector_;

        size_t num_foot_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        scalar_t last_time_;
        scalar_t min_publish_time_difference_;
    };
} // namespace legged
