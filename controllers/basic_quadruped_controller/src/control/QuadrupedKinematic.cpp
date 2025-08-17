//
// Created by biao on 24-9-12.
// Simplified quadruped kinematics based on Unitree algorithms
// 保留Pinocchio几何参数计算，使用Unitree解析运动学
//

#include <iostream>
#include <Eigen/Dense>
#include "controller_common/CtrlInterfaces.h"
#include "basic_quadruped_controller/control/QuadrupedKinematic.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <cmath>

QuadrupedKinematic::QuadrupedKinematic(CtrlInterfaces& ctrl_interfaces,
                                       const std::string& urdf_path,
                                       const std::vector<std::string>& joint_names,
                                       const Vec12& default_stand_joint_positions)
    : ctrl_interfaces_(ctrl_interfaces), joint_names_(joint_names)
{
    // 初始化关节状态
    current_joint_pos_ = Vec12::Zero();
    current_joint_vel_ = Vec12::Zero();

    // 从URDF初始化几何参数
    initializeFromURDF(urdf_path);

    // 计算站立时的足端位置
    computeStandFootPositions(default_stand_joint_positions);

    std::cout << "=== 简化运动学系统初始化完成 ===" << std::endl;
    std::cout << "质量: " << mass_ << " kg" << std::endl;
    std::cout << "重心偏移: [" << pcb_.transpose() << "] m" << std::endl;
    std::cout << "惯性张量对角线: [" << inertia_tensor_.diagonal().transpose() << "]" << std::endl;
}

void QuadrupedKinematic::initializeFromURDF(const std::string& urdf_path)
{
    try
    {
        // 加载URDF模型
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        // 计算总质量
        mass_ = 0;
        for (const auto& inertia : model_.inertias)
        {
            mass_ += inertia.mass();
        }

        // 使用零位置计算重心和惯性张量
        Vec12 q_zero = Vec12::Zero();
        pinocchio::forwardKinematics(model_, data_, q_zero);

        // 计算整体重心
        pcb_ = pinocchio::centerOfMass(model_, data_, q_zero);
                                // 使用CCRBA算法计算重心处的惯性张量
                                const Vec12 v_zero = Vec12::Zero();
                                pinocchio::ccrba(model_, data_, q_zero, v_zero);
                                inertia_tensor_ = data_.Ig.inertia().matrix();

        // 计算并缓存几何参数
        initializeGeometryParameters();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing from URDF: " << e.what() << std::endl;
        throw;
    }
}

void QuadrupedKinematic::initializeGeometryParameters()
{
    pinocchio::forwardKinematics(model_, data_, Vec12::Zero());
    pinocchio::updateFramePlacements(model_, data_);

    try
    {
        // 使用第一条腿的关节名称获取关节变换矩阵
        constexpr int start_joint_index = 0; // 第一条腿的起始索引

        // Abad连杆长度：从髋关节到髋关节的偏移
        pinocchio::JointIndex thigh_joint_id = model_.getJointId(joint_names_[start_joint_index + 1]);
        pinocchio::JointIndex calf_joint_id = model_.getJointId(joint_names_[start_joint_index + 2]);

        // 获取关节变换矩阵
        pinocchio::SE3 hip_to_thigh = model_.jointPlacements[thigh_joint_id];
        link_lengths_[0] = std::abs(hip_to_thigh.translation().y()); // 侧向偏移

        // Hip连杆长度：大腿长度
        pinocchio::SE3 thigh_to_calf = model_.jointPlacements[calf_joint_id];
        link_lengths_[1] = std::sqrt(
            thigh_to_calf.translation().x() * thigh_to_calf.translation().x() +
            thigh_to_calf.translation().z() * thigh_to_calf.translation().z()
        );

        // Knee连杆长度：小腿长度
        // 通过遍历模型树动态找到每条腿的末端，不依赖命名规律
        double knee_length = link_lengths_[1]; // 默认值

        try
        {
            // 找到第一条腿的末端
            // 通过遍历Frame，找到属于第一条腿的所有Frame，最后一个就是末端

            // 获取第一条腿的关节ID（FL_hip_joint）
            pinocchio::JointIndex first_leg_hip = model_.getJointId(joint_names_[0]);

            // 遍历所有Frame，找到属于第一条腿的Frame
            std::vector<pinocchio::FrameIndex> first_leg_frames;
            for (size_t i = 0; i < model_.nframes; ++i)
            {
                // 检查Frame是否属于第一条腿的关节链
                pinocchio::JointIndex frame_parent = model_.frames[i].parentJoint;
                if (frame_parent == first_leg_hip ||
                    frame_parent == thigh_joint_id ||
                    frame_parent == calf_joint_id)
                {
                    first_leg_frames.push_back(i);
                }
            }

            // 如果找到了Frame，最后一个就是末端
            if (!first_leg_frames.empty())
            {
                const pinocchio::FrameIndex end_frame_id = first_leg_frames.back();
                pinocchio::SE3 end_pose = data_.oMf[end_frame_id];

                // 计算从小腿关节到末端的距离
                pinocchio::SE3 calf_pose = data_.oMi[calf_joint_id];
                Vec3 calf_to_end = end_pose.translation() - calf_pose.translation();

                knee_length = std::sqrt(
                    calf_to_end.x() * calf_to_end.x() +
                    calf_to_end.z() * calf_to_end.z()
                );

                // 验证数值合理性
                if (knee_length < 0.01 || knee_length > 0.5)
                {
                    link_lengths_[2] = link_lengths_[1];
                }

                std::cout << "找到第一条腿末端Frame: " << model_.frames[end_frame_id].name
                    << ", 小腿长度: " << knee_length << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            // 如果计算失败，使用大腿长度作为默认值
            link_lengths_[2] = link_lengths_[1];
        }

        link_lengths_[2] = knee_length;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error calculating link lengths: " << e.what() << std::endl;
        throw;
    }

    // 使用关节名称直接获取髋关节位置，避免索引转换问题
    for (int leg_index = 0; leg_index < 4; ++leg_index)
    {
        // 每条腿的髋关节是第一个关节（abad关节）
        const int hip_joint_index = leg_index * 3;
        pinocchio::JointIndex hip_joint_id = model_.getJointId(joint_names_[hip_joint_index]);
        pinocchio::SE3 hip_pose = data_.oMi[hip_joint_id];
        hip_offsets_[leg_index] = hip_pose.translation().cast<double>();

        // 输出几何参数（显示控制器顺序的腿名称）
        std::string leg_names[4] = {"FR", "FL", "RR", "RL"};
        std::cout << "Leg " << leg_names[leg_index] << " (controller_idx=" << leg_index
            << "): Hip offset=[" << hip_offsets_[leg_index].transpose()
            << "], Links=[" << link_lengths_[0] << ", "
            << link_lengths_[1] << ", " << link_lengths_[2] << "]" << std::endl;
    }
}

void QuadrupedKinematic::computeStandFootPositions(const Vec12& default_stand_joint_positions)
{
    // 计算每条腿的站立足端位置
    for (int leg_index = 0; leg_index < 4; leg_index++)
    {
        Vec3 q_leg = default_stand_joint_positions.segment(3 * leg_index, 3);
        Vec3 foot_pos = calcLegFK(q_leg, leg_index);
        feet_pos_normal_stand_.col(leg_index) = foot_pos;
    }

    std::cout << "=== 站立足端位置 ===" << std::endl;
    std::cout << "FR: " << feet_pos_normal_stand_.col(0).transpose() << std::endl;
    std::cout << "FL: " << feet_pos_normal_stand_.col(1).transpose() << std::endl;
    std::cout << "RR: " << feet_pos_normal_stand_.col(2).transpose() << std::endl;
    std::cout << "RL: " << feet_pos_normal_stand_.col(3).transpose() << std::endl;
}

// ========== 核心运动学接口实现 ==========

Vec12 QuadrupedKinematic::getQ(const Vec34& feet_positions, FrameType frame) const
{
    Vec12 joint_angles = Vec12::Zero();

    for (int leg_index = 0; leg_index < 4; ++leg_index)
    {
        Vec3 target_pos = feet_positions.col(leg_index);
        Vec3 q_leg = calcQ(target_pos, frame, leg_index);
        joint_angles.segment(3 * leg_index, 3) = q_leg;
    }

    return joint_angles;
}

Vec12 QuadrupedKinematic::getQd(const Vec34& pos, const Vec34& vel, FrameType frame) const
{
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        // 直接调用原版的 calcQd 逻辑
        qd.segment(3*i, 3) = calcLegQd(pos.col(i), vel.col(i), frame, i);
    }
    return qd;
}

Vec34 QuadrupedKinematic::getFeet2BPositions() const
{
    Vec34 feetPos;
    for(int i(0); i<4; ++i){
        feetPos.col(i) = getFootPosition(i, FrameType::BODY);
    }
    return feetPos;
}

Vec3 QuadrupedKinematic::getFootPosition(int leg_id, FrameType frame) const
{
    if (leg_id < 0 || leg_id >= 4)
    {
        throw std::out_of_range("Invalid leg index");
    }

    Vec3 q_leg = current_joint_pos_.segment(3 * leg_id, 3);

    if(frame == FrameType::BODY){
        return calcLegFK(q_leg, leg_id);
    }else if(frame == FrameType::HIP){
        // 相对于髋关节的位置
        Vec3 hip_offset = hip_offsets_[leg_id];
        Vec3 p_ee_hip = calcLegFK(q_leg, leg_id) - hip_offset;
        return p_ee_hip;
    }else{
        throw std::runtime_error("The frame of function: getFootPosition can only be BODY or HIP.");
    }
}

Vec3 QuadrupedKinematic::getFeet2BVelocity(int leg_index) const
{
    if (leg_index < 0 || leg_index >= 4)
    {
        throw std::out_of_range("Invalid leg index");
    }

    Vec3 q_leg = current_joint_pos_.segment(3 * leg_index, 3);
    Vec3 qd_leg = current_joint_vel_.segment(3 * leg_index, 3);

    Mat3 jacobian = calcLegJacobian(q_leg, leg_index);
    return jacobian * qd_leg;
}

Vec3 QuadrupedKinematic::getTorque(const Vec3& foot_force, int leg_index) const
{
    if (leg_index < 0 || leg_index >= 4)
    {
        throw std::out_of_range("Invalid leg index");
    }

    Vec3 q_leg = current_joint_pos_.segment(3 * leg_index, 3);
    Mat3 jacobian = calcLegJacobian(q_leg, leg_index);

    // 逆动力学：tau = J^T * f
    return jacobian.transpose() * foot_force;
}

Vec3 QuadrupedKinematic::getTorqueAnalytical(const Vec3& foot_force, int leg_index) const
{
    // 直接调用getTorque方法，因为已经是解析方法
    return getTorque(foot_force, leg_index);
}

Vec34 QuadrupedKinematic::getFeet2BVelocities() const
{
    Vec34 feet_velocities;

    for (int leg_index = 0; leg_index < 4; ++leg_index)
    {
        feet_velocities.col(leg_index) = getFeet2BVelocity(leg_index);
    }

    return feet_velocities;
}

Mat3 QuadrupedKinematic::getJacobian(const Vec3& q_leg, int leg_index) const
{
    return calcLegJacobian(q_leg, leg_index);
}

// ========== 核心运动学算法（基于Unitree原版）==========

Vec3 QuadrupedKinematic::solveLegIK(const Vec3& target_pos, int leg_index) const
{
    // 基于Unitree原版的解析逆运动学算法

    // 获取腿部几何参数
    Vec3 hip_offset = hip_offsets_[leg_index];

    // 计算足端相对于髋关节的位置
    Vec3 p_ee_hip = target_pos - hip_offset;

    double px = p_ee_hip(0);
    double py = p_ee_hip(1);
    double pz = p_ee_hip(2);

    // 确定侧向符号（控制器顺序：FR(0) FL(1) RR(2) RL(3)）
    int side_sign = leg_index == 1 || leg_index == 3 ? 1 : -1; // FL,RL为1, FR,RR为-1
    double l1 = side_sign * link_lengths_[0];

    // 步骤1：求解q1 (Abad关节) - Unitree原版算法
    double L_squared = py * py + pz * pz - l1 * l1;
    if (L_squared < 0)
    {
        // 超出工作空间，返回默认姿态
        return Vec3(0.0, 0.67, -1.3);
    }
    double L = sqrt(L_squared);
    double q1 = atan2(pz * l1 + py * L, py * l1 - pz * L);

    // 步骤2：求解q3 (膝关节) - Unitree原版算法
    double b3z = -link_lengths_[1];
    double b4z = -link_lengths_[2];
    double c = sqrt(px * px + py * py + pz * pz);
    double b = sqrt(c * c - l1 * l1);

    double temp = (b3z * b3z + b4z * b4z - b * b) / (2 * fabs(b3z * b4z));
    temp = std::max(-1.0, std::min(1.0, temp)); // 限制在[-1,1]
    double q3 = acos(temp);
    q3 = -(M_PI - q3); // 转换到Unitree约定

    // 步骤3：求解q2 (髋关节) - Unitree原版算法
    double a1 = py * sin(q1) - pz * cos(q1);
    double a2 = px;
    double m1 = b4z * sin(q3);
    double m2 = b3z + b4z * cos(q3);
    double q2 = atan2(m1 * a1 + m2 * a2, m1 * a2 - m2 * a1);

    return Vec3(q1, q2, q3);
}

Vec3 QuadrupedKinematic::calcLegFK(const Vec3& q_leg, int leg_index) const
{
    Vec3 hip_offset = hip_offsets_[leg_index];

    // 确定侧向符号
    int side_sign = leg_index == 1 || leg_index == 3 ? 1 : -1;
    double l1 = side_sign * link_lengths_[0];
    double l2 = -link_lengths_[1];
    double l3 = -link_lengths_[2];

    // 三角函数
    double s1 = std::sin(q_leg(0)), c1 = std::cos(q_leg(0));
    double s2 = std::sin(q_leg(1)), c2 = std::cos(q_leg(1));
    double s3 = std::sin(q_leg(2)), c3 = std::cos(q_leg(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    // Unitree原版正运动学公式
    Vec3 p_ee_hip;
    p_ee_hip(0) = l3 * s23 + l2 * s2;
    p_ee_hip(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    p_ee_hip(2) = l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return hip_offset + p_ee_hip;
}

Mat3 QuadrupedKinematic::calcLegJacobian(const Vec3& q_leg, int leg_index) const
{
    // 基于Unitree原版的解析雅可比矩阵算法

    // 确定侧向符号
    int side_sign = leg_index == 1 || leg_index == 3 ? 1 : -1;
    double l1 = side_sign * link_lengths_[0];
    double l2 = -link_lengths_[1];
    double l3 = -link_lengths_[2];

    // 三角函数
    double s1 = std::sin(q_leg(0)), c1 = std::cos(q_leg(0));
    double s2 = std::sin(q_leg(1)), c2 = std::cos(q_leg(1));
    double s3 = std::sin(q_leg(2)), c3 = std::cos(q_leg(2));

    double c23 = c2 * c3 - s2 * s3;
    double s23 = s2 * c3 + c2 * s3;

    // Unitree原版雅可比矩阵公式
    Mat3 jacobian;
    jacobian(0, 0) = 0;
    jacobian(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jacobian(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    jacobian(0, 1) = l3 * c23 + l2 * c2;
    jacobian(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jacobian(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    jacobian(0, 2) = l3 * c23;
    jacobian(1, 2) = l3 * s1 * s23;
    jacobian(2, 2) = -l3 * c1 * s23;

    return jacobian;
}

Vec3 QuadrupedKinematic::getHipOffset(int leg_index) const
{
    if (leg_index < 0 || leg_index >= 4)
    {
        throw std::out_of_range("Invalid leg index");
    }
    return hip_offsets_[leg_index];
}

Vec3 QuadrupedKinematic::calcQ(const Vec3& pEe, FrameType frame, int leg_index) const
{
    // 原版逻辑：支持两种坐标系
    Vec3 pEe2H;
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - hip_offsets_[leg_index];
    else
        throw std::runtime_error("Frame type must be HIP or BODY");
    
    // 调用现有的解析逆运动学
    return solveLegIK(pEe2H + hip_offsets_[leg_index], leg_index);
}

Vec3 QuadrupedKinematic::calcLegQd(const Vec3& pEe, const Vec3& vEe, FrameType frame, int leg_index) const
{
    // 原版逻辑：先计算关节角度，再计算关节速度
    Vec3 q = calcQ(pEe, frame, leg_index);
    return calcLegJacobian(q, leg_index).inverse() * vEe;
}

// ========== 更新接口 ==========

void QuadrupedKinematic::update()
{
    // 从控制接口读取关节状态
    for (int i = 0; i < 12; i++)
    {
        current_joint_pos_(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value();
        current_joint_vel_(i) = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_optional().value();
    }
}
