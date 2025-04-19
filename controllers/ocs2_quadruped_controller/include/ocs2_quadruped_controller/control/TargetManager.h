//
// Created by tlab-uav on 24-9-30.
//

#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H


#include <memory>
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_tools/realtime_buffer.hpp>

namespace ocs2::legged_robot
{
    class TargetManager
    {
    public:
        TargetManager(CtrlInterfaces& ctrl_component,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                      const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                      const std::string& task_file,
                      const std::string& reference_file);

        ~TargetManager() = default;

        void update(SystemObservation& observation);

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose,
                                                          const SystemObservation& observation,
                                                          const scalar_t& targetReachingTime)
        {
            // desired time trajectory
            const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

            // desired state trajectory
            vector_t currentPose = observation.state.segment<6>(6);
            currentPose(2) = command_height_;
            currentPose(4) = 0;
            currentPose(5) = 0;
            vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
            stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_;
            stateTrajectory[1] << vector_t::Zero(6), targetPose, default_joint_state_;

            // desired input trajectory (just right dimensions, they are not used)
            const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

            return {timeTrajectory, stateTrajectory, inputTrajectory};
        }

        CtrlInterfaces& ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> buffer_;
        int twist_count = 0;

        vector_t default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_{};
        scalar_t target_rotation_velocity_{};
    };
}

#endif //TARGETMANAGER_H
