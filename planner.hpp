#pragma once

// STD
#include <memory>
#include <deque>

// ROS2
#include <rclcpp/rclcpp.hpp>

// Messages
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

// Eigen
#include <Eigen/Dense>

// TinyMPC
#include "tinympc/tiny_api.hpp"

namespace armor {

/**
 * @brief MPC 规划器，用于计算云台控制指令
 */
class Planner {
public:
    /**
     * @brief 创建规划器实例
     * @param bullet_speed 弹速 (m/s)
     * @param flytime_offset 飞行时间偏移
     * @param attack_range 攻击范围
     * @param attack_range_cof 攻击范围系数
     * @return 规划器智能指针
     */
    static std::shared_ptr<Planner> create(
        const double& bullet_speed,
        const double& flytime_offset,
        const double& attack_range,
        const double& attack_range_cof
    );

    ~Planner();

    /**
     * @brief 更新规划器状态
     * @param armors_msg 装甲板检测消息
     */
    void Update(const auto_aim_interfaces::msg::Armors::SharedPtr& armors_msg);

    /**
     * @brief 设置当前云台状态（从串口反馈获取）
     * @param current_yaw 当前 yaw 角度 (rad)
     * @param current_pitch 当前 pitch 角度 (rad)
     * @param timestamp 时间戳
     */

    // 考虑删除 
    void SetCurrentGimbalState(double current_yaw, double current_pitch, const rclcpp::Time& timestamp);

    /**
     * @brief 设置目标状态（从 tracker 获取）
     * @param target_yaw 目标 yaw 角度 (rad)
     * @param target_pitch 目标 pitch 角度 (rad)
     * @param tracker_info 追踪器信息
     */
    void SetTargetState(double target_yaw, double target_pitch, 
                       const auto_aim_interfaces::msg::TrackerInfo& tracker_info);

    /**
     * @brief 设置目标状态（仅位置）
     * @param target_yaw 目标 yaw 角度 (rad)
     * @param target_pitch 目标 pitch 角度 (rad)
     */
    void SetTargetState(double target_yaw, double target_pitch);

    /**
     * @brief 获取优化后的 yaw 控制指令
     */
    double GetYaw();

    /**
     * @brief 获取优化后的 pitch 控制指令
     */
    double GetPitch();

    /**
     * @brief 求解 MPC 优化问题
     * @return 是否求解成功
     */
    bool Solve();

    /**
     * @brief 构造函数
     */
    Planner(const double& bullet_speed, const double& flytime_offset,
            const double& attack_range, const double& attack_range_cof);

private:

    /**
     * @brief 初始化 TinyMPC 求解器
     */
    void InitializeMPC();

    /**
     * @brief 更新系统动力学矩阵
     */
    void UpdateDynamics();

    /**
     * @brief 估计速度（使用历史数据）
     */
    void EstimateVelocity();

    // 状态维度定义
    static constexpr int STATE_DIM = 4;   // [yaw, pitch, yaw_vel, pitch_vel]
    static constexpr int CONTROL_DIM = 2; // [yaw_acc_cmd, pitch_acc_cmd]
    static constexpr int HORIZON = 20;    // 预测时域长度（未测试过，需要依据实际情况进行拟合）

    // 参数
    double bullet_speed_;
    double flytime_offset_;
    double attack_range_;
    double attack_range_cof_;
    double dt_;  // 时间步长

    // TinyMPC 求解器
    TinySolver* solver_;
    bool solver_initialized_;

    // 当前状态 [yaw, pitch, yaw_vel, pitch_vel]
    Eigen::Matrix<double, STATE_DIM, 1> current_state_;
    
    // 目标状态
    Eigen::Matrix<double, STATE_DIM, 1> target_state_;

    // 控制指令 [yaw_cmd, pitch_cmd]
    Eigen::Matrix<double, CONTROL_DIM, 1> control_output_;

    // 历史数据队列（用于速度估计）
    struct StateHistory {
        double yaw;
        double pitch;
        rclcpp::Time timestamp;
    };
    std::deque<StateHistory> gimbal_history_;
    static constexpr size_t MAX_HISTORY_SIZE = 10;

    // 权重矩阵
    Eigen::Matrix<double, STATE_DIM, 1> Q_diag_;     // 状态权重
    Eigen::Matrix<double, CONTROL_DIM, 1> R_diag_;   // 控制权重

    // 系统动力学矩阵
    tinyMatrix A_dyn_;  // STATE_DIM x STATE_DIM
    tinyMatrix B_dyn_;  // STATE_DIM x CONTROL_DIM
    tinyVector f_dyn_;  // STATE_DIM x 1
};

} // namespace armor
