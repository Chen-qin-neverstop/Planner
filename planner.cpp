#include "planner.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <cmath>
#include <algorithm>

namespace armor {

// ==================== 构造与析构 ====================

Planner::Planner(const double& bullet_speed, const double& flytime_offset,
                 const double& attack_range, const double& attack_range_cof)
    : bullet_speed_(bullet_speed),
      flytime_offset_(flytime_offset),
      attack_range_(attack_range),
      attack_range_cof_(attack_range_cof),
      dt_(0.01),  // 默认 10ms 时间步长
    solver_(nullptr),
    solver_initialized_(false) {
    
    // 初始化状态为零
    current_state_.setZero();
    target_state_.setZero();
    control_output_.setZero();

    // 设置权重矩阵（基于电机特性和控制目标调整）
    // Q 权重: 状态偏差惩罚
    Q_diag_ << 120.0, 120.0,  // yaw, pitch 位置权重（高精度跟踪）
               8.0, 8.0;      // yaw_vel, pitch_vel 速度权重（平滑运动）
    
    // R 权重: 控制代价惩罚（加速度指令）
    R_diag_ << 0.02, 0.02;    // yaw_acc_cmd, pitch_acc_cmd

    // 初始化 TinyMPC
    InitializeMPC();
}

Planner::~Planner() {
    // 清理 TinyMPC 求解器（如果有析构函数）
    if (solver_) {
        // TinyMPC 可能需要手动释放内存
        delete solver_->solution;
        delete solver_->settings;
        delete solver_->cache;
        delete solver_->work;
        delete solver_;
    }
}

std::shared_ptr<Planner> Planner::create(
    const double& bullet_speed, const double& flytime_offset,
    const double& attack_range, const double& attack_range_cof) {
    return std::shared_ptr<Planner>(
        new Planner(bullet_speed, flytime_offset, attack_range, attack_range_cof));
}

// ==================== TinyMPC 初始化 ====================

void Planner::InitializeMPC() {
    // 分配内存
    solver_ = new TinySolver();
    solver_->solution = new TinySolution();
    solver_->settings = new TinySettings();
    solver_->cache = new TinyCache();
    solver_->work = new TinyWorkspace();

    // 初始化动力学矩阵
    A_dyn_.resize(STATE_DIM, STATE_DIM);
    B_dyn_.resize(STATE_DIM, CONTROL_DIM);
    f_dyn_.resize(STATE_DIM);

    UpdateDynamics();

    // 将 Eigen 权重转换为 TinyMPC 格式
    tinyMatrix Q = tinyMatrix::Zero(STATE_DIM, STATE_DIM);
    tinyMatrix R = tinyMatrix::Zero(CONTROL_DIM, CONTROL_DIM);
    Q.diagonal() = Q_diag_;
    R.diagonal() = R_diag_;

    // 设置 TinyMPC 求解器
    tiny_setup(&solver_, A_dyn_, B_dyn_, f_dyn_, Q, R,
               1.0,         // rho (ADMM 参数)
               STATE_DIM,   // nx
               CONTROL_DIM, // nu
               HORIZON,     // N
               0);          // verbose

    // 设置默认设置
    tiny_set_default_settings(solver_->settings);
    solver_->settings->max_iter = 100;
    solver_->settings->abs_pri_tol = 1e-4;
    solver_->settings->abs_dua_tol = 1e-4;

    // 设置约束（根据实际电机能力）
    // 状态约束 x_min, x_max (4×HORIZON)
    tinyMatrix x_min(STATE_DIM, HORIZON);
    tinyMatrix x_max(STATE_DIM, HORIZON);
    
    // 位置约束（云台机械限位，单位: rad）
    x_min.row(0).setConstant(-M_PI);      // yaw: -180°
    x_max.row(0).setConstant(M_PI);       // yaw: +180°
    x_min.row(1).setConstant(-M_PI / 4);  // pitch: -45°
    x_max.row(1).setConstant(M_PI / 4);   // pitch: +45°
    
    // 速度约束（基于电机额定转速 120rpm = 12.57 rad/s，留30%裕度）
    const double max_vel = 12.57 * 0.7;   // 8.8 rad/s
    x_min.row(2).setConstant(-max_vel);   // yaw_vel
    x_max.row(2).setConstant(max_vel);
    x_min.row(3).setConstant(-max_vel);   // pitch_vel
    x_max.row(3).setConstant(max_vel);
    
    // 控制输入约束 u_min, u_max (2×HORIZON-1)
    // 单位: rad/s²，对应电机能产生的最大角加速度
    tinyMatrix u_min(CONTROL_DIM, HORIZON - 1);
    tinyMatrix u_max(CONTROL_DIM, HORIZON - 1);
    const double max_acc_yaw = 140.0 * 0.7;     // 98 rad/s²
    const double max_acc_pitch = 350.0 * 0.7;   // 245 rad/s²
    u_min.row(0).setConstant(-max_acc_yaw);     // yaw_cmd
    u_max.row(0).setConstant(max_acc_yaw);
    u_min.row(1).setConstant(-max_acc_pitch);   // pitch_cmd
    u_max.row(1).setConstant(max_acc_pitch);
    
    tiny_set_bound_constraints(solver_, x_min, x_max, u_min, u_max);

    solver_initialized_ = true;
}

void Planner::UpdateDynamics() {
    // 二阶离散模型
    // 状态: x = [yaw, pitch, yaw_vel, pitch_vel]
    // 控制: u = [yaw_acc_cmd, pitch_acc_cmd]

    A_dyn_.setIdentity();

    // 位置积分
    A_dyn_(0, 2) = dt_;
    A_dyn_(1, 3) = dt_;

    // 速度保持（上一行 setIdentity 已设置 1）

    B_dyn_.setZero();
    const double half_dt2 = 0.5 * dt_ * dt_;

    // 控制对位置的影响 (0.5 * dt^2)
    B_dyn_(0, 0) = half_dt2;
    B_dyn_(1, 1) = half_dt2;

    // 控制对速度的影响 (dt)
    B_dyn_(2, 0) = dt_;
    B_dyn_(3, 1) = dt_;

    f_dyn_.setZero();
}

// ==================== 状态更新 ====================

void Planner::Update(const auto_aim_interfaces::msg::Armors::SharedPtr& armors_msg) {
    // 这个函数暂时不用，实际通过 SetCurrentGimbalState 和 SetTargetState 更新
    (void)armors_msg;
}

void Planner::SetCurrentGimbalState(double current_yaw, double current_pitch, 
                                   const rclcpp::Time& timestamp) {
    // 添加到历史队列
    StateHistory history;
    history.yaw = current_yaw;
    history.pitch = current_pitch;
    history.timestamp = timestamp;
    gimbal_history_.push_back(history);
    
    if (gimbal_history_.size() > MAX_HISTORY_SIZE) {
        gimbal_history_.pop_front();
    }

    // 更新当前状态
    current_state_(0) = current_yaw;
    current_state_(1) = current_pitch;
    
    // 估计速度
    EstimateVelocity();

}

void Planner::SetTargetState(double target_yaw, double target_pitch,
                            const auto_aim_interfaces::msg::TrackerInfo& tracker_info) {
    // 设置目标状态（期望跟踪的位置）
    target_state_(0) = target_yaw;
    target_state_(1) = target_pitch;
    
    // 目标速度可以从 tracker_info 获取
    target_state_(2) = tracker_info.v_yaw;  // tracker 提供的 yaw 角速度
    target_state_(3) = 0.0;                // pitch 角速度（暂无数据，设为0）
}

void Planner::SetTargetState(double target_yaw, double target_pitch) {
    // 设置云台目标状态（期望跟踪的位置）
    target_state_(0) = target_yaw;
    target_state_(1) = target_pitch;
    
    // 云台目标初始速度暂设为0
    target_state_(2) = 0.0;
    target_state_(3) = 0.0;
}

// 估计当前速度
void Planner::EstimateVelocity() {
    if (gimbal_history_.size() < 2) {
        current_state_(2) = 0.0;
        current_state_(3) = 0.0;
        return;
    }

    auto& latest = gimbal_history_.back();
    auto& previous = gimbal_history_[gimbal_history_.size() - 2];

    double dt = (latest.timestamp - previous.timestamp).seconds();
    if (dt < 1e-6) {
        return;
    }

    current_state_(2) = (latest.yaw - previous.yaw) / dt;
    current_state_(3) = (latest.pitch - previous.pitch) / dt;
}

// ==================== 求解 MPC ====================

bool Planner::Solve() {
    if (!solver_initialized_) {
        return false;
    }

    // 设置初始状态
    tinyVector x0(STATE_DIM);
    for (int i = 0; i < STATE_DIM; ++i) {
        x0(i) = current_state_(i);
    }
    tiny_set_x0(solver_, x0);

    // 设置参考轨迹（所有时间步使用相同的目标状态）
    tinyMatrix x_ref(STATE_DIM, HORIZON);
    for (int k = 0; k < HORIZON; ++k) {
        x_ref.col(k) = target_state_;
    }
    tiny_set_x_ref(solver_, x_ref);

    // 设置参考控制（期望为0）
    tinyMatrix u_ref(CONTROL_DIM, HORIZON - 1);
    u_ref.setZero();
    tiny_set_u_ref(solver_, u_ref);

    // 求解 
    int status = tiny_solve(solver_);
    // 这个肯定有问题，速度的计算需要重新考虑
    if (status == 1) {
        const double u_yaw = solver_->solution->u(0, 0);
        const double u_pitch = solver_->solution->u(1, 0);

        const double yaw = current_state_(0);
        const double pitch = current_state_(1);
        const double yaw_vel = current_state_(2);
        const double pitch_vel = current_state_(3);

        const double next_yaw = yaw + yaw_vel * dt_ + 0.5 * u_yaw * dt_ * dt_;
        const double next_pitch = pitch + pitch_vel * dt_ + 0.5 * u_pitch * dt_ * dt_;

        control_output_(0) = next_yaw;
        control_output_(1) = next_pitch;
        return true;
    }

    return false;
}

double Planner::GetYaw() {
    return control_output_(0);
}

double Planner::GetPitch() {
    return control_output_(1);
}

} // namespace armor
