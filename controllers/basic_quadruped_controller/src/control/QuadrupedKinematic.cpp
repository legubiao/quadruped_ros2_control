//
// Created by biao on 24-9-12.
//

#include <iostream>
#include <Eigen/Dense>
#include "controller_common/CtrlInterfaces.h"
#include "basic_quadruped_controller/control/QuadrupedKinematic.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <cmath>

QuadrupedKinematic::QuadrupedKinematic(CtrlInterfaces &ctrl_interfaces,
                                       const std::string &urdf_path,
                                       const std::vector<std::string> &feet_names,
                                       const std::vector<std::string> &joint_names,
                                       const Vec12& default_stand_joint_positions)
    : ctrl_interfaces_(ctrl_interfaces), feet_names_(feet_names), joint_names_(joint_names) {
    initializeModel(urdf_path, feet_names);

    // 初始化缓存
    cached_foot_positions_.resize(4);
    cached_jacobians_.resize(4);
    
    // 初始化几何参数缓存
    cached_link_lengths_.resize(4);
    cached_hip_offsets_.resize(4);

    // 初始化关节状态
    current_joint_pos_ = Vec12::Zero();
    current_joint_vel_ = Vec12::Zero();

    // 初始化几何参数缓存
    initializeGeometryCache();
    
    // 根据默认站立关节角度计算正常站立时的足端位置
    computeNormalStandFootPositionsFromJoints(default_stand_joint_positions);
}

void QuadrupedKinematic::initializeModel(const std::string &urdf_path,
                                         const std::vector<std::string> &feet_names) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        // 获取足端帧ID
        foot_frame_ids_.resize(feet_names.size());
        for (size_t i = 0; i < feet_names.size(); ++i) {
            foot_frame_ids_[i] = model_.getFrameId(feet_names[i]);
            if (foot_frame_ids_[i] >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
                throw std::runtime_error("Frame " + feet_names[i] + " not found in model");
            }
        }

        // 计算总质量
        mass_ = 0;
        for (const auto &joint: model_.inertias) {
            mass_ += joint.mass();
        }

        // 获取关节限制
        joint_lower_limits_ = model_.lowerPositionLimit;
        joint_upper_limits_ = model_.upperPositionLimit;
    } catch (const std::exception &e) {
        std::cerr << "Error initializing Pinocchio model: " << e.what() << std::endl;
        throw;
    }
}

void QuadrupedKinematic::computeAllFootKinematics() const {
    if (cache_valid_) return;

    // 计算前向运动学
    pinocchio::forwardKinematics(model_, data_, current_joint_pos_);
    pinocchio::updateFramePlacements(model_, data_);

    // 计算所有足端位置和雅可比矩阵
    for (int i = 0; i < 4; i++) {
        cached_foot_positions_[i] = data_.oMf[foot_frame_ids_[i]];
        Eigen::MatrixXd jacobian(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, current_joint_pos_, foot_frame_ids_[i], jacobian);
        cached_jacobians_[i] = jacobian;
    }

    cache_valid_ = true;
}

Vec12 QuadrupedKinematic::getQ(const std::vector<pinocchio::SE3> &pEe_list) const {
    Vec12 q_result = Vec12::Zero();

    for (int i = 0; i < 4; ++i) {
        Vec3 target_pos = pEe_list[i].translation();

        // 使用解析IK求解关节角度
        Vec3 q_leg = solveLegInverseKinematics(target_pos, i);
        q_result.segment(3 * i, 3) = q_leg;
    }

    return q_result;
}

Vec12 QuadrupedKinematic::getQ(const Vec34 &vecP) const {
    Vec12 q = Vec12::Zero();

    for (int i = 0; i < 4; ++i) {
        // 创建目标足端位置
        Eigen::Vector3d target_pos = vecP.col(i);

        // 使用当前关节位置作为初始猜测
        Eigen::VectorXd q_init = current_joint_pos_.segment(3 * i, 3);

        // 使用解析IK求解关节角度
        Vec3 q_leg = solveLegInverseKinematics(target_pos, i);
        q.segment(3 * i, 3) = q_leg;
    }

    return q;
}

Vec12 QuadrupedKinematic::getQd(const std::vector<pinocchio::SE3> &pos, const Vec34 &vel) {
    Vec12 qd = Vec12::Zero();

    // 先计算目标关节角度
    Vec12 q_target = getQ(pos);

    for (int i = 0; i < 4; ++i) {
        // 获取雅可比矩阵（基于目标关节位置）
        Eigen::MatrixXd jacobian(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q_target, foot_frame_ids_[i], jacobian);
        Eigen::MatrixXd pos_jacobian = jacobian.topRows(3);

        // 计算关节速度
        Eigen::Vector3d foot_velocity = vel.col(i);
        Eigen::Vector3d joint_velocity = pos_jacobian.inverse() * foot_velocity;

        qd.segment(3 * i, 3) = joint_velocity;
    }

    return qd;
}

Vec12 QuadrupedKinematic::getQd(const Vec12 &q, const Vec34 &vel) const {
    Vec12 qd = Vec12::Zero();

    for (int i = 0; i < 4; ++i) {
        // 获取雅可比矩阵（基于指定关节角度）
        Eigen::MatrixXd jacobian(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q, foot_frame_ids_[i], jacobian);
        Eigen::MatrixXd pos_jacobian = jacobian.topRows(3);

        // 计算关节速度
        Eigen::Vector3d foot_velocity = vel.col(i);
        Eigen::Vector3d joint_velocity = pos_jacobian.inverse() * foot_velocity;

        qd.segment(3 * i, 3) = joint_velocity;
    }

    return qd;
}

std::vector<pinocchio::SE3> QuadrupedKinematic::getFeet2BPositions() const {
    computeAllFootKinematics();
    return cached_foot_positions_;
}

pinocchio::SE3 QuadrupedKinematic::getFeet2BPositions(const int index) const {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Foot index out of range");
    }

    computeAllFootKinematics();
    return cached_foot_positions_[index];
}

Eigen::MatrixXd QuadrupedKinematic::getJacobian(const int index) const {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Foot index out of range");
    }

    computeAllFootKinematics();
    return cached_jacobians_[index];
}

Eigen::VectorXd QuadrupedKinematic::getTorque(const Vec3 &force, int index) const {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Foot index out of range");
    }

    Eigen::MatrixXd jacobian = getJacobian(index);
    Eigen::MatrixXd pos_jacobian = jacobian.topRows(3);

    // 计算关节力矩
    Eigen::VectorXd torque = pos_jacobian.transpose() * force;
    return torque;
}

Eigen::Vector3d QuadrupedKinematic::getFeet2BVelocities(const int index) const {
    if (index < 0 || index >= 4) {
        throw std::out_of_range("Foot index out of range");
    }

    Eigen::MatrixXd jacobian = getJacobian(index);
    Eigen::MatrixXd pos_jacobian = jacobian.topRows(3);

    // 计算足端速度
    Eigen::Vector3d foot_velocity = pos_jacobian * current_joint_vel_.segment(3 * index, 3);
    return foot_velocity;
}

std::vector<Eigen::Vector3d> QuadrupedKinematic::getFeet2BVelocities() const {
    std::vector<Eigen::Vector3d> result;
    result.resize(4);

    for (int i = 0; i < 4; i++) {
        result[i] = getFeet2BVelocities(i);
    }

    return result;
}

void QuadrupedKinematic::update() {
    if (mass_ == 0) return;

    // 从控制接口更新关节位置和速度
    for (int i = 0; i < 4; i++) {
        // 更新关节位置
        current_joint_pos_(i * 3) = ctrl_interfaces_.joint_position_state_interface_[i * 3].get().get_optional().
                value();
        current_joint_pos_(i * 3 + 1) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 1].get().get_optional()
                .value();
        current_joint_pos_(i * 3 + 2) = ctrl_interfaces_.joint_position_state_interface_[i * 3 + 2].get().get_optional()
                .value();

        // 更新关节速度
        current_joint_vel_(i * 3) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3].get().get_optional().
                value();
        current_joint_vel_(i * 3 + 1) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 1].get().get_optional()
                .value();
        current_joint_vel_(i * 3 + 2) = ctrl_interfaces_.joint_velocity_state_interface_[i * 3 + 2].get().get_optional()
                .value();
    }

    // 清除缓存，因为关节位置已更新
    cache_valid_ = false;
}

std::vector<Eigen::MatrixXd> QuadrupedKinematic::getAllFootJacobians() const {
    computeAllFootKinematics();
    return cached_jacobians_;
}

Vec12 QuadrupedKinematic::getJointTorques(const Vec34 &foot_forces) const {
    Vec12 joint_torques = Vec12::Zero();

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d force = foot_forces.col(i);
        Eigen::VectorXd leg_torques = getTorque(force, i);
        joint_torques.segment(3 * i, 3) = leg_torques;
    }

    return joint_torques;
}

Vec34 QuadrupedKinematic::getAllFootVelocitiesMatrix() const {
    Vec34 foot_velocities;

    for (int i = 0; i < 4; ++i) {
        foot_velocities.col(i) = getFeet2BVelocities(i);
    }

    return foot_velocities;
}

Vec12 QuadrupedKinematic::solveInverseKinematics(const Vec34 &target_feet_positions) const {
    // 实现多足端逆运动学求解器
    // 使用加权最小二乘法求解所有足端的目标位置

    Vec12 q_result = current_joint_pos_; // 使用当前关节位置作为初始值

    const int max_iterations = 50;
    const double tolerance = 1e-5;
    const double step_size = 0.05;

    // 权重矩阵：可以调整不同足端的重要性
    Eigen::Vector4d weights = Eigen::Vector4d::Ones(); // 所有足端权重相等

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前足端位置
        pinocchio::forwardKinematics(model_, data_, q_result);
        pinocchio::updateFramePlacements(model_, data_);

        // 计算所有足端的位置误差
        std::vector<Vec3> position_errors(4);
        double total_error = 0.0;

        for (int i = 0; i < 4; ++i) {
            pinocchio::updateFramePlacements(model_, data_);
            pinocchio::SE3 current_foot_pose = data_.oMf[foot_frame_ids_[i]];
            Vec3 current_pos = current_foot_pose.translation();
            Vec3 target_pos = target_feet_positions.col(i);

            position_errors[i] = target_pos - current_pos;
            total_error += weights(i) * position_errors[i].squaredNorm();
        }

        // 如果总误差足够小，退出
        if (total_error < tolerance) {
            break;
        }

        // 计算所有足端的雅可比矩阵
        std::vector<Eigen::MatrixXd> jacobians(4);
        for (int i = 0; i < 4; ++i) {
            Eigen::MatrixXd jacobian(6, model_.nv);
            pinocchio::computeFrameJacobian(model_, data_, q_result, foot_frame_ids_[i], jacobian);
            jacobians[i] = jacobian.topRows(3); // 只取位置部分
        }

        // 构建加权雅可比矩阵和误差向量
        Eigen::MatrixXd weighted_jacobian(12, model_.nv); // 4足端 × 3维位置
        Eigen::VectorXd weighted_error(12);

        for (int i = 0; i < 4; ++i) {
            weighted_jacobian.block(3 * i, 0, 3, model_.nv) = weights(i) * jacobians[i];
            weighted_error.segment(3 * i, 3) = weights(i) * position_errors[i];
        }

        // 使用伪逆求解关节角度增量
        Eigen::VectorXd delta_q = weighted_jacobian.transpose() *
                                  (weighted_jacobian * weighted_jacobian.transpose() +
                                   0.01 * Eigen::MatrixXd::Identity(12, 12)).inverse() * weighted_error;

        // 更新关节角度
        q_result += step_size * delta_q;

        // 应用关节限制
        for (int i = 0; i < 12; ++i) {
            q_result(i) = std::max(joint_lower_limits_(i),
                                   std::min(joint_upper_limits_(i), q_result(i)));
        }
    }

    return q_result;
}

Vec12 QuadrupedKinematic::solveInverseKinematics(const Vec3 &target_position, int foot_index) const {
    // 实现基于雅可比矩阵的数值逆运动学求解器
    // 针对特定足端的IK求解

    if (foot_index < 0 || foot_index >= 4) {
        throw std::out_of_range("Foot index out of range");
    }

    Vec12 q_result = current_joint_pos_;

    // 使用数值优化方法求解IK
    const int max_iterations = 100;
    const double tolerance = 1e-6;
    const double step_size = 0.1;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 计算当前足端位置
        pinocchio::forwardKinematics(model_, data_, q_result);
        pinocchio::updateFramePlacements(model_, data_);

        // 获取指定足端的当前位置
        pinocchio::SE3 current_foot_pose = data_.oMf[foot_frame_ids_[foot_index]];
        Vec3 current_pos = current_foot_pose.translation();

        // 计算位置误差
        Vec3 position_error = target_position - current_pos;

        // 如果误差足够小，退出
        if (position_error.norm() < tolerance) {
            break;
        }

        // 计算雅可比矩阵
        Eigen::MatrixXd jacobian(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q_result, foot_frame_ids_[foot_index], jacobian);
        Eigen::MatrixXd pos_jacobian = jacobian.topRows(3);

        // 使用伪逆求解关节角度增量
        Eigen::VectorXd delta_q = pos_jacobian.transpose() *
                                  (pos_jacobian * pos_jacobian.transpose() +
                                   0.01 * Eigen::Matrix3d::Identity()).inverse() * position_error;

        // 更新关节角度
        q_result += step_size * delta_q;

        // 应用关节限制
        for (int i = 0; i < 12; ++i) {
            q_result(i) = std::max(joint_lower_limits_(i),
                                   std::min(joint_upper_limits_(i), q_result(i)));
        }
    }

    return q_result;
}

Vec3 QuadrupedKinematic::solveLegInverseKinematics(const Vec3 &target_position, int leg_index) const {
    // 基于几何方法的单腿解析逆运动学求解
    // 适用于3自由度串联机械腿：Abad-Hip-Knee

    if (leg_index < 0 || leg_index >= 4) {
        throw std::out_of_range("Leg index out of range");
    }

    // 直接从缓存获取髋关节偏移量
    Vec3 hip_offset = cached_hip_offsets_[leg_index];
    
    // 计算足端相对于髋关节的位置
    Vec3 p_ee_hip = target_position - hip_offset;
    
    // 直接从缓存获取连杆长度
    double abad_length = cached_link_lengths_[leg_index][0];  // hip
    double hip_length = cached_link_lengths_[leg_index][1];   // thigh
    double knee_length = cached_link_lengths_[leg_index][2];  // calf



    // 解析IK求解
    double q1, q2, q3;

    // 步骤1：求解q1 (Abad关节)
    double py = p_ee_hip(1);
    double pz = p_ee_hip(2);
    double l1 = (leg_index == 0 || leg_index == 2) ? abad_length : -abad_length; // 左右腿符号不同

    double L = sqrt(py * py + pz * pz - l1 * l1);
    if (L < 0) {
        // 目标位置超出工作空间，使用最近的有效位置
        double max_reach = sqrt(hip_length * hip_length + knee_length * knee_length);
        if (sqrt(py * py + pz * pz) > max_reach + l1) {
            // 完全超出工作空间，使用默认姿态
            return Vec3(0.0, 0.67, -1.3); // 默认站立姿态
        }
        // 调整到工作空间边界
        double scale = (max_reach + l1) / sqrt(py * py + pz * pz);
        py *= scale;
        pz *= scale;
        L = sqrt(py * py + pz * pz - l1 * l1);
    }

    q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);

    // 步骤2：求解q3 (膝关节)
    double px = p_ee_hip(0);
    double b3z = -hip_length;
    double b4z = -knee_length;
    double c = sqrt(px * px + py * py + pz * pz); // 总距离
    double b = sqrt(c * c - l1 * l1); // 足端到肩部的距离

    double temp = (b3z * b3z + b4z * b4z - b * b) / (2 * fabs(b3z * b4z));
    if (temp > 1) temp = 1;
    if (temp < -1) temp = -1;

    q3 = acos(temp);
    q3 = -(M_PI - q3); // 限制在0~180度

    // 检查解的有效性
    if (std::isnan(q3) || std::isinf(q3)) {
        // 使用默认姿态
        return Vec3(0.0, 0.67, -1.3);
    }

    // 步骤3：求解q2 (髋关节)
    double a1 = py * sin(q1) - pz * cos(q1);
    double a2 = px;
    double m1 = b4z * sin(q3);
    double m2 = b3z + b4z * cos(q3);

    q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

    // 应用关节限制
    Vec3 joint_limits_lower = joint_lower_limits_.segment(3 * leg_index, 3);
    Vec3 joint_limits_upper = joint_upper_limits_.segment(3 * leg_index, 3);

    q1 = std::max(joint_limits_lower(0), std::min(joint_limits_upper(0), q1));
    q2 = std::max(joint_limits_lower(1), std::min(joint_limits_upper(1), q2));
    q3 = std::max(joint_limits_lower(2), std::min(joint_limits_upper(2), q3));

    // 最终验证：检查解是否在合理范围内
    if (std::abs(q1) > M_PI || std::abs(q2) > M_PI || std::abs(q3) > M_PI) {
        // 角度超出合理范围，使用默认姿态
        return Vec3(0.0, 0.67, -1.3);
    }

    return Vec3(q1, q2, q3);
}

bool QuadrupedKinematic::validateJointLimits(const Vec12 &joint_positions) const {
    for (int i = 0; i < 12; ++i) {
        if (joint_positions(i) < joint_lower_limits_(i) ||
            joint_positions(i) > joint_upper_limits_(i)) {
            return false;
        }
    }
    return true;
}



void QuadrupedKinematic::initializeGeometryCache() {
    // 在模型加载时计算并缓存所有几何参数
    // 这些参数是固定的，只需要计算一次
    
    for (int leg_index = 0; leg_index < 4; ++leg_index) {
        // 计算髋关节偏移量
        // 从URDF中动态获取，适应不同机器人型号
        const int joints_per_leg = 3;
        const int hip_joint_index = 1 + leg_index * joints_per_leg;
        
        // 获取髋关节名称
        std::string hip_joint_name = model_.names[hip_joint_index];
        
        // 使用Pinocchio的frame功能获取髋关节相对于基座的位置
        try {
            // 计算前向运动学到髋关节
            Vec12 q_temp = Vec12::Zero();
            
            // 更新模型状态
            pinocchio::forwardKinematics(model_, data_, q_temp);
            pinocchio::updateFramePlacements(model_, data_);
            
            // 获取髋关节的位置
            pinocchio::JointIndex hip_joint_id = hip_joint_index;
            pinocchio::SE3 hip_pose = data_.oMi[hip_joint_id];
            
            // 提取偏移量
            cached_hip_offsets_[leg_index] = hip_pose.translation().cast<double>();
            
        } catch (const std::exception& e) {
            // 如果计算失败，使用默认值
            switch (leg_index) {
                case 0: cached_hip_offsets_[leg_index] = Vec3(0.1934, 0.0465, 0.0); break;
                case 1: cached_hip_offsets_[leg_index] = Vec3(0.1934, -0.0465, 0.0); break;
                case 2: cached_hip_offsets_[leg_index] = Vec3(-0.1934, 0.0465, 0.0); break;
                case 3: cached_hip_offsets_[leg_index] = Vec3(-0.1934, -0.0465, 0.0); break;
            }
        }
        
        // 计算连杆长度
        const int start_joint_index = 1 + leg_index * joints_per_leg;
        
        // Hip连杆：从Hip关节到Thigh关节的距离
        if (start_joint_index + 1 < model_.njoints) {
            pinocchio::SE3 hip_to_thigh = model_.jointPlacements[start_joint_index + 1];
            cached_link_lengths_[leg_index][0] = hip_to_thigh.translation().norm();
        }
        
        // Thigh连杆：从Thigh关节到Calf关节的距离
        if (start_joint_index + 2 < model_.njoints) {
            pinocchio::SE3 thigh_to_calf = model_.jointPlacements[start_joint_index + 2];
            cached_link_lengths_[leg_index][1] = thigh_to_calf.translation().norm();
        }
        
        // Calf连杆：从Calf关节到足端的距离
        // 通过足端帧相对于Calf关节的位置来计算
        try {
            // 获取足端帧ID
            pinocchio::FrameIndex foot_frame_id = foot_frame_ids_[leg_index];
            
            // 计算足端相对于Calf关节的位置
            // 需要先计算前向运动学到Calf关节
            Vec12 q_temp = Vec12::Zero();
            q_temp.segment(start_joint_index, 3) = Vec3(0, 0, 0); // 设置Calf关节为0度
            
            // 更新模型状态
            pinocchio::forwardKinematics(model_, data_, q_temp);
            pinocchio::updateFramePlacements(model_, data_);
            
            // 获取Calf关节和足端的位置
            pinocchio::JointIndex calf_joint_id = start_joint_index + 2;
            pinocchio::SE3 calf_pose = data_.oMi[calf_joint_id];
            pinocchio::SE3 foot_pose = data_.oMf[foot_frame_id];
            
            // 计算Calf关节到足端的距离
            Vec3 calf_to_foot = foot_pose.translation() - calf_pose.translation();
            cached_link_lengths_[leg_index][2] = calf_to_foot.norm();
            
        } catch (const std::exception& e) {
            // 如果计算失败，使用URDF中的标准值
            cached_link_lengths_[leg_index][2] = 0.213;
        }
    }
    
    // 输出缓存的几何参数（用于验证）
    std::cout << "=== Geometry Parameters Cached ===" << std::endl;
    for (int leg_index = 0; leg_index < 4; ++leg_index) {
        std::string leg_name;
        switch (leg_index) {
            case 0: leg_name = "FL"; break;
            case 1: leg_name = "FR"; break;
            case 2: leg_name = "RL"; break;
            case 3: leg_name = "RR"; break;
        }
        std::cout << leg_name << " - Hip Offset: " << cached_hip_offsets_[leg_index].transpose() 
                  << ", Link Lengths: [" << cached_link_lengths_[leg_index][0] 
                  << ", " << cached_link_lengths_[leg_index][1] 
                  << ", " << cached_link_lengths_[leg_index][2] << "]" << std::endl;
    }
    std::cout << "=================================" << std::endl;
}

void QuadrupedKinematic::computeNormalStandFootPositionsFromJoints(const Vec12& stand_joint_positions) {
    // 保存当前的关节状态
    Vec12 original_joint_pos = current_joint_pos_;
    
    // 设置站立关节角度
    current_joint_pos_ = stand_joint_positions;
    
    // 计算前向运动学
    pinocchio::forwardKinematics(model_, data_, current_joint_pos_);
    pinocchio::updateFramePlacements(model_, data_);
    
    // 计算每条腿的足端位置
    for (int i = 0; i < 4; i++) {
        // 获取足端帧的位置
        pinocchio::SE3 foot_pose = data_.oMf[foot_frame_ids_[i]];
        Vec3 foot_pos = foot_pose.translation().cast<double>();
        
        // 存储到feet_pos_normal_stand_矩阵中
        feet_pos_normal_stand_.col(i) = foot_pos;
    }
    
    // 恢复原来的关节状态
    current_joint_pos_ = original_joint_pos;
    
    // 输出计算得到的足端位置（用于验证）
    std::cout << "=== Normal Stand Foot Positions (Computed from Joint Angles) ===" << std::endl;
    std::cout << "FL: " << feet_pos_normal_stand_.col(0).transpose() << std::endl;
    std::cout << "FR: " << feet_pos_normal_stand_.col(1).transpose() << std::endl;
    std::cout << "RL: " << feet_pos_normal_stand_.col(2).transpose() << std::endl;
    std::cout << "RR: " << feet_pos_normal_stand_.col(3).transpose() << std::endl;
    std::cout << "=============================================================" << std::endl;
}


