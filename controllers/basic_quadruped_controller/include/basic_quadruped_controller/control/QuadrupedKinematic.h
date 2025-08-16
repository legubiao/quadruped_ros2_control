//
// Created by biao on 25-8-15.
//
#pragma once
#include <string>
#include <vector>
#include <pinocchio/algorithm/jacobian.hpp>
#include <basic_quadruped_controller/common/mathTypes.h>

struct CtrlInterfaces;

class QuadrupedKinematic {
public:
    explicit QuadrupedKinematic(CtrlInterfaces &ctrl_interfaces, 
                                const std::string &urdf_path,
                                const std::vector<std::string> &feet_names, 
                                const std::vector<std::string> &joint_names,
                                const Vec12& default_stand_joint_positions);

    ~QuadrupedKinematic() = default;

    // ========== 外部接口（保持兼容性）==========
    
               /**
            * 根据足端位置计算关节角度 - 外部接口
            */
    [[nodiscard]] Vec12 getQ(const std::vector<pinocchio::SE3> &pEe_list) const;

    /**
     * 根据足端位置矩阵计算关节角度 - 外部接口
     */
    [[nodiscard]] Vec12 getQ(const Vec34 &vecP) const;

               /**
            * 根据足端位置和速度计算关节速度 - 外部接口
            */
           Vec12 getQd(const std::vector<pinocchio::SE3> &pos, const Vec34 &vel);

           /**
            * 根据关节角度和足端速度计算关节速度 - 外部接口
            */
           Vec12 getQd(const Vec12 &q, const Vec34 &vel) const;

    /**
     * 计算所有足端位置 - 外部接口
     */
    [[nodiscard]] std::vector<pinocchio::SE3> getFeet2BPositions() const;

    /**
     * 计算指定足端位置 - 外部接口
     */
    [[nodiscard]] pinocchio::SE3 getFeet2BPositions(const int index) const;

    /**
     * 计算指定足端雅可比矩阵 - 外部接口
     */
    [[nodiscard]] Eigen::MatrixXd getJacobian(const int index) const;

    /**
     * 计算关节力矩 - 外部接口
     */
    [[nodiscard]] Eigen::VectorXd getTorque(const Vec3 &force, int index) const;

    /**
     * 计算足端速度 - 外部接口
     */
    [[nodiscard]] Eigen::Vector3d getFeet2BVelocities(const int index) const;

    /**
     * 计算所有足端速度 - 外部接口
     */
    [[nodiscard]] std::vector<Eigen::Vector3d> getFeet2BVelocities() const;

    /**
     * 更新机器人状态 - 外部接口
     */
    void update();

    // ========== 新增的高效批量接口 ==========
    
    /**
     * 批量计算所有足端雅可比矩阵 - 新增高效接口
     */
    [[nodiscard]] std::vector<Eigen::MatrixXd> getAllFootJacobians() const;

    /**
     * 批量计算所有足端力矩 - 新增高效接口
     */
    [[nodiscard]] Vec12 getJointTorques(const Vec34 &foot_forces) const;

    /**
     * 批量计算所有足端速度 - 新增高效接口
     */
    [[nodiscard]] Vec34 getAllFootVelocitiesMatrix() const;

    /**
     * 使用解析方法计算单足端速度（更稳定的替代方案）
     * @param index 足端索引
     * @return 足端速度
     */
    Eigen::Vector3d getFeet2BVelocitiesAnalytical(const int index) const;

    // ========== 公共成员变量（保持兼容性）==========
    double mass_ = 0;
    Vec34 feet_pos_normal_stand_;
    Vec12 current_joint_pos_;
    Vec12 current_joint_vel_;

private:
    CtrlInterfaces &ctrl_interfaces_;
    
    // Pinocchio模型和数据
    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    
    // 足端帧ID和名称
    std::vector<pinocchio::FrameIndex> foot_frame_ids_;
    std::vector<std::string> feet_names_;
    
    // 关节名称
    std::vector<std::string> joint_names_;
    
    // 缓存数据（优化性能）
    mutable std::vector<pinocchio::SE3> cached_foot_positions_;
    mutable std::vector<Eigen::MatrixXd> cached_jacobians_;
    mutable bool cache_valid_ = false;
    
    // 几何参数缓存（初始化时计算一次）
    std::vector<Vec3> cached_hip_offsets_;                   // 4个腿的髋关节偏移 [leg_index]
    std::vector<std::array<double, 3>> cached_link_lengths_;  // 4个腿x3个连杆长度 [leg_index][hip, thigh, calf]
    
    // 关节限制
    Vec12 joint_lower_limits_;
    Vec12 joint_upper_limits_;
    
    /**
     * 初始化Pinocchio模型
     */
    void initializeModel(const std::string &urdf_path,
                         const std::vector<std::string> &feet_names);
    
    /**
     * 批量计算前向运动学和雅可比矩阵（内部优化方法）
     */
    void computeAllFootKinematics() const;
    
    /**
     * 批量计算逆运动学（内部优化方法）
     */
    Vec12 solveInverseKinematics(const Vec34 &target_feet_positions) const;
    
               /**
            * 单足端逆运动学求解（内部优化方法）
            */
           Vec12 solveInverseKinematics(const Vec3 &target_position, int foot_index) const;

           /**
            * 单腿解析逆运动学求解（基于几何方法）
            */
           Vec3 solveLegInverseKinematics(const Vec3 &target_position, int leg_index) const;
    
               /**
            * 验证关节角度是否在限制范围内
            */
           bool validateJointLimits(const Vec12 &joint_positions) const;

       private:
               /**
     * 初始化几何参数缓存（在模型加载时调用一次）
     */
    void initializeGeometryCache();
    
    /**
     * 根据默认站立关节角度计算正常站立时的足端位置（内部方法）
     * @param stand_joint_positions 默认站立关节角度
     */
    void computeNormalStandFootPositionsFromJoints(const Vec12& stand_joint_positions);
};
