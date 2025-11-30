# MPC Planner 模块

基于 TinyMPC 的模型预测控制器，用于云台轨迹规划。

## 功能特性

- **4 维状态空间**: [yaw, pitch, yaw_vel, pitch_vel]
- **2 维控制输入**: [yaw_acc_cmd, pitch_acc_cmd]
- **ADMM 求解器**: 快速收敛的 MPC 优化
- **速度估计**: 基于历史数据的有限差分估计
- **离散动力学**: 双积分器模型，适合云台控制

## 编译

```bash
cd /path/to/auto_aim_2026
source install/local_setup.bash
colcon build --symlink-install --packages-select planner
```

## 运行

### 方式 1: 使用启动文件

```bash
source install/local_setup.bash
ros2 launch planner planner_launch.py
```

### 方式 2: 直接运行节点

```bash
source install/local_setup.bash
ros2 run planner planner_node
```

## 参数配置

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `control_rate` | 100.0 | 控制频率 (Hz) |
| `bullet_speed` | 28.0 | 弹丸速度 (m/s) |
| `flytime_offset` | 0.0 | 飞行时间偏移量 |
| `attack_range` | 10.0 | 攻击范围 (m) |
| `attack_range_cof` | 1.0 | 攻击范围系数 |

### 修改参数示例

```bash
ros2 launch planner planner_launch.py control_rate:=150.0 bullet_speed:=30.0
```

## 话题接口

### 订阅话题

- `/detector/armors` ([auto_aim_interfaces/msg/Armors])
  - 装甲板检测结果

- `/tracker/info` ([auto_aim_interfaces/msg/TrackerInfo])
  - 跟踪器状态信息
  - 包含目标速度、装甲板数量等

- `/gimbal_feedback` ([communicate_2025/msg/SerialInfo])
  - 云台当前状态反馈
  - 包含当前 yaw/pitch 角度

### 发布话题

- `/shoot_info` ([communicate_2025/msg/SerialInfo])
  - MPC 计算的云台控制指令
  - 包含目标 yaw/pitch 角度
  - **与 shooter 使用相同话题，可直接替换**

- `/planner/target` ([auto_aim_interfaces/msg/Target])
  - 规划器的目标信息
  - 用于调试和可视化

## 使用说明

### 1. 基本使用流程

```
检测器 → 跟踪器 → MPC规划器 → 云台控制
   ↓         ↓         ↓           ↓
armors → tracker_info → shoot_info → 下位机
                      ↑
                 gimbal_feedback
```

### 2. 集成到现有系统

在你的主控制节点中：

```cpp
#include "planner.hpp"

// 创建规划器
auto planner = std::make_shared<armor::Planner>();

// 在控制循环中
void ControlCallback() {
    // 1. 更新当前云台状态
    planner->SetCurrentGimbalState(current_yaw, current_pitch, timestamp);
    
    // 2. 设置目标状态
    planner->SetTargetState(target_yaw, target_pitch, timestamp);
    
    // 3. 求解 MPC
    if (planner->Solve()) {
        double cmd_yaw = planner->GetYaw();
        double cmd_pitch = planner->GetPitch();
        // 发送控制指令...
    }
}
```

### 3. 调试模式

查看详细日志：

```bash
ros2 run planner planner_node --ros-args --log-level debug
```

## MPC 参数调优

### 权重矩阵

在 `planner.cpp` 中修改：

```cpp
// 状态权重 Q_diag: [yaw, pitch, yaw_vel, pitch_vel]
Q_diag_ << 150, 150, 20, 20;

// 控制权重 R_diag: [yaw_cmd, pitch_cmd]
R_diag_ << 0.1, 0.1;
```

**调优建议:**
- **增大 Q_diag[0:1]**: 提高位置跟踪精度，但可能增加抖动
- **增大 Q_diag[2:3]**: 抑制角速度，使运动更平滑
- **增大 R_diag**: 减小控制输出幅度，适合控制能力有限的系统
- **减小 R_diag**: 允许更激进的控制，适合高性能云台

### 预测范围

```cpp
static constexpr int HORIZON = 20;  // 预测步数
```

- 更大的 HORIZON: 考虑更长未来，计算量更大
- 更小的 HORIZON: 计算快，但预见性差

### 时间步长

```cpp
double dt_ = 0.01;  // 10ms
```

需要与控制频率匹配。

## 性能优化

### 1. 求解器参数

在 `InitializeMPC()` 中调整 ADMM 参数：

```cpp
solver_->settings->max_iter = 100;      // 最大迭代次数
solver_->settings->abs_pri_tol = 1e-4;  // 收敛容差
solver_->settings->rho = 1.0;           // ADMM 惩罚参数
```

### 2. 控制频率

- **100 Hz**: 平衡性能和计算负载（推荐）
- **200 Hz**: 高性能云台，需要强计算能力
- **50 Hz**: 资源受限场景

## 故障排除

### 问题 1: MPC 求解失败

**症状**: 日志显示 "MPC 求解失败"

**解决方案**:
1. 检查状态是否在合理范围内
2. 降低收敛容差 `abs_pri_tol`
3. 增加最大迭代次数 `max_iter`
4. 检查动力学矩阵 A, B 是否正确

### 问题 2: 控制输出抖动

**症状**: 云台运动不平滑

**解决方案**:
1. 增大速度权重 `Q_diag[2:3]`
2. 收紧输入约束（降低 `max_yaw_acc` / `max_pitch_acc`）
3. 增大控制权重 `R_diag`
4. 检查速度估计是否准确

### 问题 3: 跟踪滞后

**症状**: 云台跟不上目标

**解决方案**:
1. 增大预测范围 `HORIZON`
2. 减小控制权重 `R_diag`
3. 增大位置权重 `Q_diag[0:1]`
4. 检查控制频率是否足够高

### 问题 4: 编译错误

**找不到 TinyMPC**:
```bash
# 确保 TinyMPC 子目录存在
ls tinympc/
```

**找不到 Eigen3**:
```bash
sudo apt install libeigen3-dev
```

**找不到 auto_aim_interfaces**:
```bash
colcon build --packages-select auto_aim_interfaces
source install/local_setup.bash
```

## 开发说明

### 文件结构

```
planner/
├── CMakeLists.txt          # 构建配置
├── package.xml             # ROS2 包清单
├── README.md               # 本文件
├── planner.hpp             # Planner 类头文件
├── planner.cpp             # Planner 类实现
├── planner_node.hpp        # ROS2 节点头文件
├── planner_node.cpp        # ROS2 节点实现
├── launch/
│   └── planner_launch.py   # 启动文件
└── tinympc/                # TinyMPC 库
```

### 扩展功能

**添加新的状态变量:**
1. 修改 `STATE_DIM` 常量
2. 更新 `UpdateDynamics()` 中的 A, B 矩阵
3. 调整 Q 权重矩阵维度

**添加约束:**
1. 在 `InitializeMPC()` 中设置约束
2. 使用 `tiny_set_x_bound()`, `tiny_set_u_bound()`

**切换求解器:**
- TinyMPC 支持不同的求解算法
- 参考 TinyMPC 文档修改求解器配置

## 参考资料

- [TinyMPC GitHub](https://github.com/TinyMPC/tinympc)
- [ADMM 算法原理](https://stanford.edu/~boyd/admm.html)
- ROS2 官方文档: https://docs.ros.org/

## 许可证

与 auto_aim_2026 项目保持一致。
