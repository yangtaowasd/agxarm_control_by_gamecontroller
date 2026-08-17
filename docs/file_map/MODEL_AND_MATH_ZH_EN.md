# 模型与数学 / Model and Mathematics

## 目标 / Goal

这些文件提供与 ROS 无关的李群、URDF 运动学/动力学和外力观测数学。 / These
files provide ROS-independent Lie-group, URDF kinematics/dynamics, and
external-disturbance observation mathematics.

## 文件 / Files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `armbycontroller/modeling/__init__.py` | 独立共享建模 package 的边界 / boundary of the standalone shared-modeling package |
| `armbycontroller/modeling/lie.py` | SO(3)/SE(3) 指数与对数、空间误差、伴随、空间运动/力叉乘 / SO(3)/SE(3) exp/log, space error, adjoint, motion/force cross products |
| `armbycontroller/modeling/screw_model.py` | 解析 URDF/Xacro，构造 PoE chain，计算 FK、空间 Jacobian、质量矩阵、RNEA、动量观测项 / parse URDF/Xacro; PoE, FK, space Jacobian, mass matrix, RNEA, observer terms |
| `armbycontroller/modeling/momentum_observer.py` | 不使用加速度微分的广义动量残差积分器 / acceleration-free generalized-momentum residual integrator |
| `armbycontroller/__init__.py` | 顶层 Python package 标识 / top-level Python package marker |

## 依赖方向 / Dependency direction

```text
modeling/lie.py <- cartesian/spatial.py <- impedance/ | admittance/
       ^
       +-------- modeling/screw_model.py <- controller adapters
                              \---- modeling/momentum_observer.py
```

数学 module 不应反向依赖 ROS 节点、键盘协议或 AGX SDK。模型数组遵循关节顺序，
任务空间遵循 `[angular; linear]`。 / Mathematical modules must not depend back
on ROS nodes, keyboard protocol, or the AGX SDK. Model arrays use joint order;
task-space arrays use `[angular; linear]`.
