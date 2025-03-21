//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/visualize/FootPlacementVisualization.h"
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace ocs2::legged_robot
{
    FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector,
                                                           size_t numFoot,
                                                           const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                                           scalar_t maxUpdateFrequency)
        : convex_region_selector_(convexRegionSelector),
          num_foot_(numFoot),
          last_time_(std::numeric_limits<scalar_t>::lowest()),
          min_publish_time_difference_(1.0 / maxUpdateFrequency)
    {
        marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("foot_placement", 1);
    }

    void FootPlacementVisualization::update(const SystemObservation& observation)
    {
        if (observation.time - last_time_ > min_publish_time_difference_)
        {
            last_time_ = observation.time;

            std_msgs::msg::Header header;
            //    header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
            header.frame_id = "odom";

            visualization_msgs::msg::MarkerArray makerArray;

            size_t i = 0;
            for (int leg = 0; leg < num_foot_; ++leg)
            {
                auto middleTimes = convex_region_selector_.getMiddleTimes(leg);

                int kStart = 0;
                for (int k = 0; k < middleTimes.size(); ++k)
                {
                    const auto projection = convex_region_selector_.getProjection(leg, middleTimes[k]);
                    if (projection.regionPtr == nullptr)
                    {
                        continue;
                    }
                    if (middleTimes[k] < observation.time)
                    {
                        kStart = k + 1;
                        continue;
                    }
                    auto color = feet_color_map_[leg];
                    float alpha = 1 - static_cast<float>(k - kStart) / static_cast<float>(middleTimes.size() - kStart);
                    // Projections
                    auto projectionMaker = getArrowAtPointMsg(
                        projection.regionPtr->transformPlaneToWorld.linear() * vector3_t(0, 0, 0.1),
                        projection.positionInWorld, color);
                    projectionMaker.header = header;
                    projectionMaker.ns = "Projections";
                    projectionMaker.id = i;
                    projectionMaker.color.a = alpha;
                    makerArray.markers.push_back(projectionMaker);

                    // Convex Region
                    const auto convexRegion = convex_region_selector_.getConvexPolygon(leg, middleTimes[k]);
                    auto convexRegionMsg =
                        convex_plane_decomposition::to3dRosPolygon(convexRegion,
                                                                   projection.regionPtr->transformPlaneToWorld, header);
                    makerArray.markers.push_back(to3dRosMarker(convexRegion,
                                                               projection.regionPtr->transformPlaneToWorld, header,
                                                               color, alpha, i));

                    // Nominal Footholds
                    const auto nominal = convex_region_selector_.getNominalFootholds(leg, middleTimes[k]);
                    auto nominalMarker = getFootMarker(nominal, true, color, foot_marker_diameter_, 1.);
                    nominalMarker.header = header;
                    nominalMarker.ns = "Nominal Footholds";
                    nominalMarker.id = i;
                    nominalMarker.color.a = alpha;
                    makerArray.markers.push_back(nominalMarker);

                    i++;
                }
            }

            marker_publisher_->publish(makerArray);
        }
    }

    visualization_msgs::msg::Marker FootPlacementVisualization::to3dRosMarker(
        const convex_plane_decomposition::CgalPolygon2d& polygon,
        const Eigen::Isometry3d& transformPlaneToWorld,
        const std_msgs::msg::Header& header, Color color, float alpha, size_t i) const
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = "Convex Regions";
        marker.id = i;
        marker.header = header;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = line_width_;
        marker.color = getColor(color, alpha);
        if (!polygon.is_empty())
        {
            marker.points.reserve(polygon.size() + 1);
            for (const auto& point : polygon)
            {
                const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    point, transformPlaneToWorld);
                geometry_msgs::msg::Point point_ros;
                point_ros.x = pointInWorld.x();
                point_ros.y = pointInWorld.y();
                point_ros.z = pointInWorld.z();
                marker.points.push_back(point_ros);
            }
            // repeat the first point to close to polygon
            const auto pointInWorld =
                convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
                    polygon.vertex(0), transformPlaneToWorld);
            geometry_msgs::msg::Point point_ros;
            point_ros.x = pointInWorld.x();
            point_ros.y = pointInWorld.y();
            point_ros.z = pointInWorld.z();
            marker.points.push_back(point_ros);
        }
        marker.pose.orientation.w = 1.0;
        return marker;
    }
} // namespace legged
