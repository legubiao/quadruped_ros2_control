//
// Created by tlab-uav on 24-9-24.
//

#include "Ocs2QuadrupedController.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot {
    controller_interface::CallbackReturn Ocs2QuadrupedController::on_init() {
        verbose_ = false;
        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);

        setUpLeggedInterface();
        setupMpc();
        setupMrt();

        // Visualization
        CentroidalModelPinocchioMapping pinocchioMapping(legged_interface_->getCentroidalModelInfo());
        eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchioMapping,
            legged_interface_->modelSettings().contactNames3DoF);
        // robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
        //                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // State estimation
        setupStateEstimate();

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(),
                                             legged_interface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_);
        wbc_->loadTasksSetting(task_file_, verbose_);

        // Safety Checker
        safetyChecker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

        return CallbackReturn::SUCCESS;
    }

    void Ocs2QuadrupedController::setUpLeggedInterface() {
        legged_interface_ = std::make_shared<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);
    }

    void Ocs2QuadrupedController::setupMpc() {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());
        rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                          legged_interface_->getCentroidalModelInfo());

        const std::string robotName = "legged_robot";
        // Gait receiver
        auto gaitReceiverPtr =
                std::make_shared<GaitReceiver>(this->get_node(),
                                               legged_interface_->getSwitchedModelReferenceManagerPtr()->
                                               getGaitSchedule(), robotName);
        // ROS ReferenceManager
        auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
            robotName, legged_interface_->getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(this->get_node());
        mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
        observationPublisher_ = nh.advertise<ocs2_msgs::msg::mpc_observation>(robotName + "_mpc_observation", 1);
    }

    void Ocs2QuadrupedController::setupMrt() {
        mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
        mpcMrtInterface_->initRollout(&legged_interface_->getRollout());
        mpcTimer_.reset();

        controllerRunning_ = true;
        mpcThread_ = std::thread([&]() {
            while (controllerRunning_) {
                try {
                    executeAndSleep(
                        [&]() {
                            if (mpcRunning_) {
                                mpcTimer_.startTimer();
                                mpcMrtInterface_->advanceMpc();
                                mpcTimer_.endTimer();
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                } catch (const std::exception &e) {
                    controllerRunning_ = false;
                    RCLCPP_WARN(get_node()->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpcThread_);
    }

    void Ocs2QuadrupedController::setupStateEstimate() {
        stateEstimate_ = std::make_shared<KalmanFilterEstimate>(legged_interface_->getPinocchioInterface(),
                                                                legged_interface_->getCentroidalModelInfo(),
                                                                *eeKinematicsPtr_, this->get_node());
        dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(task_file_, verbose_);
        currentObservation_.time = 0;
    }
}
