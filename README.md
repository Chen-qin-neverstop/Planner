## Planner
***MPC(Model Predictive Control) 云台控制规划器，为了让目标云台轨迹和实际云台运动轨迹尽可能一致（注意是不是完全，因为我们需要在装甲板跳变前提前减速，所以对于三角波的尖峰，可以选择不去拟合）***

### 1. 流程概述
#### Overview
**Tracker → Shooter (计算目标yaw/pitch)
              ↓
          Planner (MPC优化)
              ↓
          下位机**
### 2. 具体实现
1. 订阅shooter/info发送的yaw和pitch作为输入，同时订阅/communicate/autoaim获取当前云台状态，同样包括yaw和pitch
2. 将当前云台状态(current_state)和目标状态(target_state)传入MPC规划器进行求解，一定时间通过数组来保存历史状态
3. tinympc内部通过admm算法求解优化问题(我目前只使用接口，对admm了解不多)
4. 发布planner/target话题，供下位机使用
    