#include "planner_node.hpp"

namespace armor {

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options) : Node("planner_node", options) {
    RCLCPP_INFO(this->get_logger(), "正在启动 MPC Planner 节点...");


    // 声明参数
    control_rate_ = this->declare_parameter<double>("control_rate", 100.0);  // 100Hz 控制频率
    bullet_speed_ = this->declare_parameter<double>("bullet_speed", 28.0);
    flytime_offset_ = this->declare_parameter<double>("flytime_offset", 0.0);
    attack_range_ = this->declare_parameter<double>("attack_range", 10.0);
    attack_range_cof_ = this->declare_parameter<double>("attack_range_cof", 1.0);
    gimbal_feedback_topic_ = this->declare_parameter<std::string>(
        "gimbal_feedback_topic", "/communicate/autoaim");

    // 创建规划器
    planner_ = std::make_shared<Planner>(bullet_speed_, flytime_offset_, 
                                          attack_range_, attack_range_cof_);

    // 订阅
    shooter_info_sub_ = this->create_subscription<communicate_2025::msg::SerialInfo>(
        "shooter/info",
        rclcpp::SensorDataQoS(),
        std::bind(&PlannerNode::ShooterInfoCallback, this, std::placeholders::_1)
    );

    gimbal_feedback_sub_ = this->create_subscription<communicate_2025::msg::Autoaim>(
        gimbal_feedback_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&PlannerNode::GimbalFeedbackCallback, this, std::placeholders::_1)
    );
    
    // 发布到 /shoot_info 以供下位机使用
    control_pub_ = this->create_publisher<communicate_2025::msg::SerialInfo>(
        "/shoot_info",
        rclcpp::SensorDataQoS()
    );

    // 创建定时器
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
        std::bind(&PlannerNode::ControlTimerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "MPC Planner 节点已启动，控制频率: %.1f Hz", control_rate_);
}

void PlannerNode::ShooterInfoCallback(const communicate_2025::msg::SerialInfo::SharedPtr msg) {
    latest_shooter_info_ = msg;
    
    // 如果有云台反馈和目标信息，更新目标状态
    if (latest_gimbal_feedback_ && latest_shooter_info_) {
        // 从 SerialInfo 获取目标位置
        double target_yaw = static_cast<double>(msg->yaw);
        double target_pitch = static_cast<double>(msg->pitch);
        
        planner_->SetTargetState(target_yaw, target_pitch);
        
        RCLCPP_DEBUG(this->get_logger(), "收到 ShooterInfo: yaw=%.3f, pitch=%.3f", 
                     target_yaw, target_pitch);
    }
}

void PlannerNode::GimbalFeedbackCallback(const communicate_2025::msg::Autoaim::SharedPtr msg) {
    latest_gimbal_feedback_ = msg;
    
    // 更新当前云台状态
    // Autoaim 消息提供云台 yaw/pitch
    double yaw = static_cast<double>(msg->high_gimbal_yaw);
    double pitch = static_cast<double>(msg->pitch);
    
    planner_->SetCurrentGimbalState(yaw, pitch, this->now());
    
    RCLCPP_DEBUG(this->get_logger(), "云台反馈: yaw=%.3f, pitch=%.3f", yaw, pitch);
}

void PlannerNode::ControlTimerCallback() {
    // 检查是否有必要的数据
    if (!latest_gimbal_feedback_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "等待云台反馈数据...");
        return;
    }

    if (!latest_shooter_info_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "等待 ShooterInfo 数据...");
        return;
    }

    // 检查是否找到目标
    // SerialInfo 中的 is_find 是 std_msgs/Char，需要检查其 data
    //  '1' 或 1 表示找到目标，
    // 这里暂时不检查 is_find，或者假设只要有消息就是有效的
    // 我们主要是对云台进行控制，只针对shooter info中的目标位置进行处理，所以是否找到目标不作为控制的前提条件

    // 求解 MPC
    bool solved = planner_->Solve();
    
    if (solved) {
        // 获取控制输出
        double cmd_yaw = planner_->GetYaw();
        double cmd_pitch = planner_->GetPitch();
        
        RCLCPP_DEBUG(this->get_logger(), "MPC 控制: yaw=%.3f, pitch=%.3f", cmd_yaw, cmd_pitch);
        
        // 发布控制指令到 /shoot_info
        auto control_msg = communicate_2025::msg::SerialInfo();
        control_msg.yaw = static_cast<float>(cmd_yaw);
        control_msg.pitch = static_cast<float>(cmd_pitch);
        // 从 shooter/info 传递 is_find
        if (latest_shooter_info_) {
            control_msg.is_find = latest_shooter_info_->is_find;
        } else {
            control_msg.is_find.data = 0; // 或者其他默认值
        }
        control_pub_->publish(control_msg);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "MPC 求解失败");
    }
}

} // namespace armor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(armor::PlannerNode)
