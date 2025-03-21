//
// Created by biao on 3/21/25.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>

#include "ocs2_quadruped_controller/perceptive/visualize/SphereVisualization.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace ocs2::legged_robot
{
    SphereVisualization::SphereVisualization(PinocchioInterface pinocchioInterface,
                                             CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioSphereInterface& sphereInterface,
                                             const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                             const scalar_t maxUpdateFrequency)
        : pinocchio_interface_(std::move(pinocchioInterface)),
          centroidal_model_info_(std::move(centroidalModelInfo)),
          sphere_interface_(sphereInterface),
          last_time_(std::numeric_limits<scalar_t>::lowest()),
          min_publish_time_difference_(1.0 / maxUpdateFrequency)
    {
        marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("sphere_markers", 1);
    }

    void SphereVisualization::update(const SystemObservation& observation)
    {
        if (observation.time - last_time_ > min_publish_time_difference_)
        {
            last_time_ = observation.time;

            visualization_msgs::msg::MarkerArray markers;

            const auto& model = pinocchio_interface_.getModel();
            auto& data = pinocchio_interface_.getData();
            forwardKinematics(model, data,
                              centroidal_model::getGeneralizedCoordinates(
                                  observation.state, centroidal_model_info_));

            auto positions = sphere_interface_.computeSphereCentersInWorldFrame(pinocchio_interface_);
            auto numSpheres = sphere_interface_.getNumSpheres();
            auto rads = sphere_interface_.getSphereRadii();

            int k = 0;
            for (int i = 0; i < numSpheres.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.id = i;
                marker.header.frame_id = "odom";
                marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                marker.color = getColor(Color::red, 0.5);
                marker.pose.orientation = ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
                marker.scale.x = rads[k];
                marker.scale.y = marker.scale.x;
                marker.scale.z = marker.scale.x;
                for (int j = 0; j < numSpheres[i]; ++j)
                {
                    marker.points.push_back(ros_msg_helpers::getPointMsg(positions[k + j]));
                }
                k += numSpheres[i];

                markers.markers.push_back(marker);
            }
            marker_publisher_->publish(markers);
        }
    }
} // namespace legged
