## Planner
***MPC(Model Predictive Control) 云台控制规划器，为了让目标云台轨迹和实际云台运动轨迹尽可能一致***

### 1. 流程概述
#### Overview
```
检测器 → 跟踪器 → 弹道解算(Shooter) → MPC规划器(Planner) → 下位机
   ↓         ↓            ↓                  ↓
armors → tracker_info → shooter/info → /shoot_info
                                         ↑
                                  /communicate/autoaim
```
### 2.文件结构
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
### 3. 具体实现
1. 订阅shooter/info发送的yaw和pitch作为输入，同时订阅/communicate/autoaim获取当前云台状态，同样包括yaw和pitch
2. 将当前云台状态(current_state)和目标状态(target_state)传入MPC规划器进行求解，一定时间通过数组来保存历史状态
3. tinympc内部通过admm算法求解优化问题
4. 发布/shoot/info话题，供下位机使用

### 4.话题接口

### 订阅话题

- `shooter/info` ([communicate_2025/msg/SerialInfo])
  - Shooter 节点计算出的目标云台角度 (yaw/pitch)
  - 包含 `is_find` 标志

- `/communicate/autoaim` ([communicate_2025/msg/Autoaim])
  - 云台当前状态反馈
  - 包含当前 yaw/pitch 角度

### 发布话题

- `/shoot_info` ([communicate_2025/msg/SerialInfo])
  - MPC 计算的云台控制指令
  - 包含优化后的 yaw/pitch 角度
  - 透传 `is_find` 标志
  - **直接发送给下位机通信节点**

## 5.TODO
1. 物理约束作用未知
2. 参数未调优
3. 速度的计算可能还有问题，现在是用有限差分计算的（就是求导），可靠性未知，需要考虑对应的delta time，如果太小的话噪声的影响很大，但是太大的话响应会变慢，是否需要滤波？（考虑用卡尔曼滤波器来估计速度，毕竟现在已经有位置和时间戳了，可以用一个简单的一阶卡尔曼滤波器来估计速度，这样可以减少噪声的影响，同时也能更准确地反映云台的运动状态）
4. （注意是不是完全，因为我们需要在装甲板跳变前提前减速，所以对于三角波的尖峰，选择不去拟合）