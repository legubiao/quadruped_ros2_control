//
// Created by qiayuan on 2022/7/1.
//

#include "ocs2_quadruped_controller/wbc/WbcBase.h"

#include <utility>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::legged_robot
{
    WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics& eeKinematics)
        : pinocchio_interface_measured_(pinocchioInterface),
          pinocchio_interface_desired_(pinocchioInterface),
          info_(std::move(info)),
          ee_kinematics_(eeKinematics.clone()),
          mapping_(info_),
          input_last_(vector_t::Zero(info_.inputDim))
    {
        num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
        q_measured_ = vector_t(info_.generalizedCoordinatesNum);
        v_measured_ = vector_t(info_.generalizedCoordinatesNum);
    }

    vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t /*period*/)
    {
        contact_flag_ = modeNumber2StanceLeg(mode);
        num_contacts_ = 0;
        for (const bool flag : contact_flag_)
        {
            if (flag)
            {
                num_contacts_++;
            }
        }

        updateMeasured(rbdStateMeasured);
        updateDesired(stateDesired, inputDesired);

        return {};
    }

    void WbcBase::updateMeasured(const vector_t& rbdStateMeasured)
    {
        q_measured_.head<3>() = rbdStateMeasured.segment<3>(3);
        q_measured_.segment<3>(3) = rbdStateMeasured.head<3>();
        q_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
        v_measured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
        v_measured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            q_measured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
        v_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(
            info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

        const auto& model = pinocchio_interface_measured_.getModel();
        auto& data = pinocchio_interface_measured_.getData();

        // For floating base EoM task
        forwardKinematics(model, data, q_measured_, v_measured_);
        computeJointJacobians(model, data);
        updateFramePlacements(model, data);
        crba(model, data, q_measured_);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        nonLinearEffects(model, data, q_measured_, v_measured_);
        j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED,
                             jac);
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        // For not contact motion task
        computeJointJacobiansTimeVariation(model, data, q_measured_, v_measured_);
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                          pinocchio::LOCAL_WORLD_ALIGNED, jac);
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }
    }

    void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired)
    {
        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();

        mapping_.setPinocchioInterface(pinocchio_interface_desired_);
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        forwardKinematics(model, data, qDesired);
        computeJointJacobians(model, data, qDesired);
        updateFramePlacements(model, data);
        updateCentroidalDynamics(pinocchio_interface_desired_, info_, qDesired);
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
        forwardKinematics(model, data, qDesired, vDesired);
    }

    Task WbcBase::formulateFloatingBaseEomTask()
    {
        const auto& data = pinocchio_interface_measured_.getData();

        matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
        s.block(0, 0, info_.actuatedDofNum, 6).setZero();
        s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

        matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, num_decision_vars_) << data.M, -j_.transpose(), -s.
            transpose()).finished();
        vector_t b = -data.nle;

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask()
    {
        matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);
        d.setZero();
        matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
        d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
                info_.actuatedDofNum) = i;
        d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
                info_.actuatedDofNum,
                info_.actuatedDofNum) = -i;
        vector_t f(2 * info_.actuatedDofNum);
        for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
        {
            f.segment<3>(3 * l) = torque_limits_;
        }

        return {matrix_t(), vector_t(), d, f};
    }

    Task WbcBase::formulateNoContactMotionTask()
    {
        matrix_t a(3 * num_contacts_, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; i++)
        {
            if (contact_flag_[i])
            {
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateFrictionConeTask()
    {
        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        a.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])
            {
                a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            }
        }
        vector_t b(a.rows());
        b.setZero();

        matrix_t frictionPyramic(5, 3); // clang-format off
        frictionPyramic << 0, 0, -1,
                1, 0, -friction_coeff_,
                -1, 0, -friction_coeff_,
                0, 1, -friction_coeff_,
                0, -1, -friction_coeff_; // clang-format on

        matrix_t d(5 * num_contacts_ + 3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        d.setZero();
        j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contact_flag_[i])
            {
                d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
            }
        }
        vector_t f = Eigen::VectorXd::Zero(d.rows());

        return {a, b, d, f};
    }

    Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
    {
        matrix_t a(6, num_decision_vars_);
        a.setZero();
        a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

        vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - input_last_, info_) / period;
        input_last_ = inputDesired;
        mapping_.setPinocchioInterface(pinocchio_interface_desired_);

        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

        const auto& A = getCentroidalMomentumMatrix(pinocchio_interface_desired_);
        const Matrix6 Ab = A.template leftCols<6>();
        const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
        const auto Aj = A.rightCols(info_.actuatedDofNum);
        const auto ADot = dccrba(model, data, qDesired, vDesired);
        Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(
            pinocchio_interface_desired_, info_, inputDesired);
        centroidalMomentumRate.noalias() -= ADot * vDesired;
        centroidalMomentumRate.noalias() -= Aj * jointAccel;

        Vector6 b = AbInv * centroidalMomentumRate;

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateSwingLegTask()
    {
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        std::vector<vector3_t> posMeasured = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_desired_);
        std::vector<vector3_t> posDesired = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velDesired = ee_kinematics_->getVelocity(vector_t(), vector_t());

        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])
            {
                vector3_t accel = swing_kp_ * (posDesired[i] - posMeasured[i]) + swing_kd_ * (
                    velDesired[i] - velMeasured[i]);
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const
    {
        matrix_t a(3 * info_.numThreeDofContacts, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();

        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
        b = inputDesired.head(a.rows());

        return {a, b, matrix_t(), vector_t()};
    }

    void WbcBase::loadTasksSetting(const std::string& taskFile, const bool verbose)
    {
        // Load task file
        torque_limits_ = vector_t(info_.actuatedDofNum / 4);
        loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torque_limits_);
        if (verbose)
        {
            std::cerr << "\n #### Torque Limits Task:";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "\n #### Hip_joint Thigh_joint Calf_joint: " << torque_limits_.transpose() << "\n";
            std::cerr << " #### =============================================================================\n";
        }
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        std::string prefix = "frictionConeTask.";
        if (verbose)
        {
            std::cerr << "\n #### Friction Cone Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, friction_coeff_, prefix + "frictionCoefficient", verbose);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }
        prefix = "swingLegTask.";
        if (verbose)
        {
            std::cerr << "\n #### Swing Leg Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, swing_kp_, prefix + "kp", verbose);
        loadData::loadPtreeValue(pt, swing_kd_, prefix + "kd", verbose);
    }
} // namespace legged
