# 交互工具 / Interaction Tools

## 目标 / Goal

这些文件把人或手机输入转换为已有目标接口，或提供独立设备控制。 / These files
translate human or phone input into existing target interfaces, or control an
independent device.

## 文件 / Files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `src/keyboard.cpp` | 读取 Linux input event 并发布统一 25 键状态 / read Linux input events and publish the unified 25-key state |
| `armbycontroller/teleop/keyboard.py` | 将固定 25 键协议边沿检测为关节选择、限位内增量和模式请求，不依赖 ROS/CAN / edge-detect the fixed 25-key protocol into joint selection, limit-safe increments, and mode requests without ROS/CAN |
| `armbycontroller/pose_controller.py` | 独立 `PoseStamped -> screw IK -> move_j/JointState` 工作流，支持 RViz dry-run / standalone pose-to-IK-to-position workflow with RViz dry-run |
| `armbycontroller/terminal_teleop.py` | 终端按键转换为相对目标位姿 / terminal keys to relative target poses |
| `armbycontroller/motion_link_bridge.py` | Motion Link WebSocket 手机姿态到 `PoseStamped`，含零点、超时和姿态范围限制 / phone orientation to `PoseStamped` with zeroing, timeout, and orientation bounds |
| `armbycontroller/hand_controller.py` | Revo2 手部开合、触觉启用与状态打印；独立于 arm 控制循环 / Revo2 open/close, touch setup, and status; independent of the arm loop |

## 安全责任 / Safety responsibility

交互工具生成目标，不实现机械臂力矩安全。`pose_controller.py` 负责工作空间、IK/FK
校验；`motion_link_bridge.py` 负责手机消息 freshness 与角度限制；最终硬件限制仍由
执行节点和驱动器承担。 / Interaction tools generate targets rather than arm
torque safety. `pose_controller.py` owns workspace and IK/FK checks;
`motion_link_bridge.py` owns phone-message freshness and orientation bounds;
hardware limits remain with the executing node and drives.
