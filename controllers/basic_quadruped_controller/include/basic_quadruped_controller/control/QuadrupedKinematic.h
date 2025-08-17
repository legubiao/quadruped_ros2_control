//
// Created by biao on 25-8-15.
// Simplified quadruped kinematics based on Unitree algorithms
// 保留Pinocchio几何参数计算，使用Unitree解析运动学
//
#pragma once
#include <string>
#include <vector>
#include <pinocchio/algorithm/frames.hpp>
#include <controller_common/common/enumClass.h>
#include <basic_quadruped_controller/common/mathTypes.h>

struct CtrlInterfaces;

/**
 * 简化的四足机器人运动学类
 * - 使用Pinocchio从URDF自动计算几何参数（连杆长度、髋关节偏移等）
 * - 使用Unitree原版解析算法进行运动学计算（高效、稳定）
 */
class QuadrupedKinematic
{
public:
    explicit QuadrupedKinematic(CtrlInterfaces& ctrl_interfaces,
                                const std::string& urdf_path,
                                const std::vector<std::string>& joint_names,
                                const Vec12& default_stand_joint_positions);

    ~QuadrupedKinematic() = default;

    // ========== 核心运动学接口（基于Unitree原版设计）==========

    /**
     * 逆运动学：根据足端位置计算关节角度（原版接口）
     * @param feet_positions 足端位置矩阵
     * @param frame 坐标系类型（默认BODY）
     * @return 关节角度（12x1向量）
     */
    [[nodiscard]] Vec12 getQ(const Vec34& feet_positions, FrameType frame = FrameType::BODY) const;

    /**
     * 微分逆运动学：根据足端位置和速度计算关节速度（原版接口）
     * @param pos 足端位置矩阵
     * @param vel 足端速度矩阵
     * @param frame 坐标系类型（默认BODY）
     * @return 关节速度（12x1向量）
     */
    [[nodiscard]] Vec12 getQd(const Vec34& pos, const Vec34& vel, FrameType frame = FrameType::BODY) const;

    /**
     * 正运动学：计算所有足端位置
     * @return 足端位置矩阵（3x4，控制器顺序：FR FL RR RL）
     */
    [[nodiscard]] Vec34 getFeet2BPositions() const;

    /**
     * 正运动学：计算单个足端位置（原版接口）
     * @param leg_id 腿索引（控制器顺序：0=FR, 1=FL, 2=RR, 3=RL）
     * @param frame 坐标系类型（默认BODY）
     * @return 足端位置（3x1向量）
     */
    [[nodiscard]] Vec3 getFootPosition(int leg_id, FrameType frame = FrameType::BODY) const;

    /**
     * 微分正运动学：计算足端速度
     * @param leg_index 腿索引
     * @return 足端速度（3x1向量）
     */
    [[nodiscard]] Vec3 getFeet2BVelocity(int leg_index) const;

    /**
     * 微分正运动学：计算所有足端速度
     * @return 足端速度矩阵（3x4，控制器顺序：FR FL RR RL）
     */
    [[nodiscard]] Vec34 getFeet2BVelocities() const;

    /**
     * 逆动力学：根据足端力计算关节力矩
     * @param foot_force 足端力
     * @param leg_index 腿索引
     * @return 关节力矩（3x1向量）
     */
    [[nodiscard]] Vec3 getTorque(const Vec3& foot_force, int leg_index) const;

    /**
     * 计算雅可比矩阵
     * @param q_leg 单腿关节角度（3x1向量）
     * @param leg_index 腿索引
     * @return 雅可比矩阵（3x3）
     */
    [[nodiscard]] Mat3 getJacobian(const Vec3& q_leg, int leg_index) const;

    /**
     * 单腿逆运动学（原版接口）
     * @param pEe 足端位置
     * @param frame 坐标系类型
     * @param leg_index 腿索引
     * @return 关节角度
     */
    [[nodiscard]] Vec3 calcQ(const Vec3& pEe, FrameType frame, int leg_index) const;

    /**
     * 单腿微分逆运动学（原版接口）
     * @param pEe 足端位置
     * @param vEe 足端速度
     * @param frame 坐标系类型
     * @param leg_index 腿索引
     * @return 关节速度
     */
    [[nodiscard]] Vec3 calcLegQd(const Vec3& pEe, const Vec3& vEe, FrameType frame, int leg_index) const;

    // ========== 机器人参数接口 ==========
    /**
     * 获取机器人质量
     */
    [[nodiscard]] double getRobMass() const { return mass_; }

    /**
     * 获取机器人重心偏移
     */
    [[nodiscard]] Vec3 getPcb() const { return pcb_; }

    /**
     * 获取机器人惯性张量
     */
    [[nodiscard]] Mat3 getRobInertial() const { return inertia_tensor_; }

    /**
     * 更新关节状态（从控制接口读取）
     */
    void update();

public:
    // 公共成员变量（保持向后兼容）
    Vec12 current_joint_pos_;
    Vec12 current_joint_vel_;
    Vec34 feet_pos_normal_stand_;
    double mass_ = 0;

private:
    // ========== 核心运动学算法（基于Unitree原版）==========

    /**
     * 单腿解析正运动学（Unitree原版算法）
     * @param q_leg 关节角度（3x1向量）
     * @param leg_index 腿索引
     * @return 足端位置（髋关节坐标系，与 calcPEe2H 保持一致）
     */
    [[nodiscard]] Vec3 calcLegFK(const Vec3& q_leg, int leg_index) const;

    /**
     * 单腿解析雅可比矩阵（Unitree原版算法）
     * @param q_leg 关节角度（3x1向量）
     * @param leg_index 腿索引
     * @return 雅可比矩阵（3x3）
     */
    [[nodiscard]] Mat3 calcLegJacobian(const Vec3& q_leg, int leg_index) const;

    // ========== 原版解析逆运动学辅助函数 ==========
    
    /**
     * 计算q1关节角度（Abad关节）- 原版算法
     * @param py 足端Y坐标（髋关节坐标系）
     * @param pz 足端Z坐标（髋关节坐标系）
     * @param l1 Abad连杆长度
     * @return q1关节角度
     */
    [[nodiscard]] double q1_ik(double py, double pz, double l1) const;

    /**
     * 计算q3关节角度（膝关节）- 原版算法
     * @param b3z 髋关节Z偏移
     * @param b4z 膝关节Z偏移
     * @param b 足端到肩部的距离
     * @return q3关节角度
     */
    [[nodiscard]] double q3_ik(double b3z, double b4z, double b) const;

    /**
     * 计算q2关节角度（髋关节）- 原版算法
     * @param q1 q1关节角度
     * @param q3 q3关节角度
     * @param px 足端X坐标（髋关节坐标系）
     * @param py 足端Y坐标（髋关节坐标系）
     * @param pz 足端Z坐标（髋关节坐标系）
     * @param b3z 髋关节Z偏移
     * @param b4z 膝关节Z偏移
     * @return q2关节角度
     */
    [[nodiscard]] double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z) const;


    // ========== 几何参数计算（使用Pinocchio从URDF获取）==========

    /**
     * 获取髋关节偏移量
     * @param leg_index 腿索引
     * @return 髋关节相对于机身的偏移量
     */
    [[nodiscard]] Vec3 getHipOffset(int leg_index) const;

private:
    // ========== 成员变量 ==========
    CtrlInterfaces& ctrl_interfaces_;

    // Pinocchio模型（仅用于几何参数计算）
    pinocchio::Model model_;
    pinocchio::Data data_;

    // 几何参数缓存（从URDF动态计算一次）
    Vec3 link_lengths_; // [abad, hip, knee] - 所有腿使用相同的长度参数
    std::array<Vec3, 4> hip_offsets_; // [leg_index] 髋关节偏移

    // 关节名称（用于直接获取关节位置）
    std::vector<std::string> joint_names_;

    // 机器人整体参数
    Vec3 pcb_; // 重心偏移
    Mat3 inertia_tensor_; // 惯性张量

    /**
     * 从URDF初始化几何参数（使用Pinocchio）
     */
    void initializeFromURDF(const std::string& urdf_path);

    /**
     * 初始化几何参数（从Pinocchio模型提取）
     */
    void initializeGeometryParameters();

    /**
     * 计算站立时的足端位置
     */
    void computeStandFootPositions(const Vec12& default_stand_joint_positions);
};
