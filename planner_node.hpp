#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

// Messages
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "communicate_2025/msg/autoaim.hpp"
#include "communicate_2025/msg/serial_info.hpp"

// Planner
#include "planner.hpp"

namespace armor {

/**
 * @brief MPC 规划器节点示例
 */
class PlannerNode : public rclcpp::Node {
public:
    explicit PlannerNode(const rclcpp::NodeOptions& options);

private:
    // 回调函数
    void ShooterInfoCallback(const communicate_2025::msg::SerialInfo::SharedPtr msg);
    void GimbalFeedbackCallback(const communicate_2025::msg::Autoaim::SharedPtr msg);
    
    // 定时器回调
    void ControlTimerCallback();

    // 订阅者
    rclcpp::Subscription<communicate_2025::msg::SerialInfo>::SharedPtr shooter_info_sub_;
    rclcpp::Subscription<communicate_2025::msg::Autoaim>::SharedPtr gimbal_feedback_sub_;

    // 发布者 发布给下位机的控制指令
    rclcpp::Publisher<communicate_2025::msg::SerialInfo>::SharedPtr control_pub_;  // 发布到 /shoot_info

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 规划器
    std::shared_ptr<Planner> planner_;

    // 缓存最新数据
    communicate_2025::msg::SerialInfo::SharedPtr latest_shooter_info_;
    communicate_2025::msg::Autoaim::SharedPtr latest_gimbal_feedback_;
    
    // 参数
    double control_rate_;  // 控制频率 (Hz)
    double bullet_speed_;
    double flytime_offset_;
    double attack_range_;
    double attack_range_cof_;
    std::string gimbal_feedback_topic_;
};

} // namespace armor
