# 项目学习指南 / Project Study Guide

## 1. 项目目标 / Project goal

本项目通过 ROS 2、键盘输入和 AGX SDK 控制 Nero 与 Piper-L。当前分支
`feature/cartesian-impedance-step-by-step` 的目标，是从已经存在的关节 MIT
阻抗出发，逐层建立公式清楚、坐标一致、可测试的空间笛卡尔阻抗，并为
Piper-L 增加由动量观测外力驱动的笛卡尔导纳模式。

This project controls Nero and Piper-L through ROS 2, keyboard input, and the
AGX SDK. The goal of `feature/cartesian-impedance-step-by-step` is to start
from the existing joint MIT impedance and build a formula-explicit,
frame-consistent, testable Cartesian impedance controller in stages, plus a
Piper-L Cartesian-admittance mode driven by momentum-observer external torque.

当前阶段已把纯数学核心接入 `mit_tick()` 和 AGX MIT/CAN。默认
`impedance_backend:=cartesian`；设为 `joint` 可以保留旧关节 MIT 作为对照。
软件集成测试通过不代表已经验证真实机械臂的物理稳定性。

The pure formula core is now wired into `mit_tick()` and AGX MIT/CAN. The
default is `impedance_backend:=cartesian`; selecting `joint` retains the old
joint MIT backend for comparison. Passing software integration tests does not
establish physical stability on real hardware.

交互后端严格互锁：`I` 切换阻抗，`O` 切换导纳；进入一个模式会先退出另一个，
两者绝不同时运行。导纳目前只支持 Piper-L。 / Interaction backends are
strictly interlocked: `I` toggles impedance and `O` toggles admittance;
entering either first exits the other, and both can never run together.
Admittance currently supports Piper-L only.

## 2. 心智模型 / Mental model

关节阻抗把每个关节看成转动弹簧和阻尼器：

Joint impedance treats each joint as a rotational spring and damper:

```text
tau_q = Kq (q_d - q) + Dq (qdot_d - qdot)
```

笛卡尔阻抗把末端工具看成六维弹簧和阻尼器。先计算期望 wrench，再通过虚功
关系映射为关节力矩：

Cartesian impedance treats the tool as a six-dimensional spring and damper.
It first computes a desired wrench, then maps it to joint torque by virtual
work:

```text
F_c   = Kx e_x + Dx (xdot_d - xdot)
tau_x = Jg(q)^T F_c
```

核心关系不是 `J^-1`，而是 `J^T`。速度通过 `J` 从关节空间映射到任务空间，
wrench 通过其功率对偶 `J^T` 映射回关节空间：

The key relationship is not `J^-1`, but `J^T`. Velocity maps from joint space
to task space through `J`; its power-dual wrench maps back through `J^T`:

```text
xdot = Jg qdot
tau_x^T qdot = F_c^T xdot
```

## 3. 坐标与数据约定 / Frame and data conventions

- 所有任务空间量都表达在 `base_link`。 / All task-space quantities are
  expressed in `base_link`.
- 六维顺序固定为 `[rx, ry, rz, x, y, z]`，也就是 `[角; 线]`。 / The
  six-dimensional order is `[rx, ry, rz, x, y, z]`, or `[angular; linear]`.
- wrench 顺序对应 `[Mx, My, Mz, Fx, Fy, Fz]`。 / Wrench order is
  `[Mx, My, Mz, Fx, Fy, Fz]`.
- `q` 使用 rad，`qdot` 使用 rad/s，关节力矩使用 N·m。 / `q` uses rad,
  `qdot` uses rad/s, and joint torque uses N·m.
- 旋转误差使用 base-frame rotation vector，不使用 RPY/Euler 角直接相减。 /
  Orientation error is a base-frame rotation vector, not a direct RPY/Euler
  angle subtraction.

## 4. 从 PoE 雅可比到几何雅可比 / PoE-to-geometric Jacobian

`UrdfScrewModel.space_jacobian()` 返回 Modern Robotics 的空间 twist 雅可比：

`UrdfScrewModel.space_jacobian()` returns a Modern Robotics space-twist
Jacobian:

```text
V_s = [omega; v] = J_s qdot
```

其中 `v` 不是末端原点的直接线速度。若末端位置为 `p`：

Here `v` is not directly the velocity of the tool origin. For tool position
`p`:

```text
p_dot = v + omega × p = v - [p]x omega

Jg = [ J_omega                 ]
     [ J_v - [p]x J_omega      ]
```

若跳过这一步，平移速度、阻尼 wrench 和最终关节力矩的方向都会错误。

Skipping this conversion gives incorrect translational velocity, damping
wrench, and joint-torque direction.

## 5. 控制公式 / Control equations

### 5.1 位姿误差 / Pose error

```text
e_R = Log(R_d R^T)^vee
e_p = p_d - p
e_x = [e_R; e_p]
```

`R_d R^T` 使旋转误差表达在基坐标系，与 `Jg` 的角速度行一致。

`R_d R^T` expresses orientation error in the base frame, matching the angular
rows of `Jg`.

这里的 `Log(.)^vee` 是 SO(3) 对数映射：结果是轴角形式的三维旋转向量。笛卡尔
阻抗的平移误差仍直接使用工具原点位移 `p_d-p`，使平移刚度保持 N/m，并与工具
原点几何雅可比、物理 wrench 一致。这不是完整 `MatrixLog6` 误差，是有意保留的
阻抗语义。

Here `Log(.)^vee` is the SO(3) logarithm and returns a three-dimensional
axis-angle rotation vector. Cartesian impedance still uses the direct tool-
origin displacement `p_d-p`, preserving translational stiffness in N/m and
its pairing with the tool-origin geometric Jacobian and physical wrench. It
is intentionally not a full `MatrixLog6` error.

### 5.1.1 旋量 IK 的完整 SE(3) 误差 / Full SE(3) error for screw IK

```text
V_error^s = Log(T_d T^-1)^vee
delta_q   = J_s(q)^# V_error^s
```

旋量 IK 使用完整的基坐标系空间误差 twist，并直接配对 PoE 空间雅可比 `J_s`。
旋转和平移在 SE(3) 对数中耦合；这里不再使用“旋转向量 + 普通位置差”的混合
迭代误差。`#` 表示带奇异值自适应阻尼的广义逆。

Screw IK uses a full base-frame space-error twist paired directly with the
PoE space Jacobian `J_s`. Rotation and translation are coupled by the SE(3)
logarithm; the iterative error is no longer a mixture of a rotation vector
and ordinary position difference. `#` denotes the singularity-adaptive
damped inverse.

RPY 只作为 URDF 和手机传感器的输入格式。键盘小角度增量、URDF RPY 转换和手机
姿态映射都通过 SO(3) 指数映射构造旋转矩阵；ROS 消息 seam 仍必须使用四元数。
RPY 不参与 IK 或笛卡尔阻抗的姿态误差计算。 / RPY remains only an input
format for URDF and phone sensor data. Keyboard increments, URDF RPY
conversion, and phone orientation mapping construct rotations through SO(3)
exponentials; ROS message seams still require quaternions. RPY is not used to
compute IK or Cartesian-impedance orientation error.

### 5.2 速度误差与 wrench / Velocity error and wrench

```text
xdot     = Jg(q) qdot
e_xdot   = xdot_d - xdot
F_c      = Kx e_x + Dx e_xdot
```

`Kx`、`Dx` 必须是有限、对称、半正定的 `6×6` 矩阵。代码也接受长度为 6
的向量，并明确转换为对角矩阵。完整矩阵允许保留轴间耦合。

`Kx` and `Dx` must be finite, symmetric, positive-semidefinite `6×6`
matrices. The code also accepts six-vectors and explicitly converts them to
diagonal matrices. Full matrices preserve cross-axis coupling.

### 5.3 虚功映射 / Virtual-work mapping

```text
tau_task = Jg(q)^T F_c
```

该公式对冗余和非冗余机械臂都成立，不需要求 `J^-1`，也不会因为接近奇异位形
而直接反演发散。

This relation applies to redundant and non-redundant arms. It does not require
`J^-1`, so it does not directly introduce an inverse singularity.

### 5.4 URDF 动力学支撑 / URDF dynamics support

当前公式核心把零期望加速度代入逆动力学：

The formula core evaluates inverse dynamics with zero desired acceleration:

```text
tau_model = ID(q, qdot, 0)
          = C(q, qdot) qdot + g(q)

tau_cmd = tau_task + tau_null + tau_model
```

它使用实测 `q/qdot`，不把 IK 参考速度或轨迹加速度混入模型支撑。

It uses measured `q/qdot`; IK reference velocity and trajectory acceleration
are not mixed into model support.

`tau_cmd` 是当前采样的直接公式结果，控制器不读取上一周期力矩，也不实现
`delta_tau`/`delta_tau_max` 力矩变化率限制。MIT adapter 只实施逐关节绝对
力矩上限，默认 ±8 N·m，且这个总量已经包含模型支撑。

`tau_cmd` is the immediate equation result for the current sample. The
controller does not read the previous torque command and does not implement a
`delta_tau`/`delta_tau_max` torque-rate limiter. The MIT adapter applies only a
per-joint absolute limit, ±8 N·m by default, to the total that already includes
model support.

### 5.5 Nero 动力学一致零空间阻抗 / Nero dynamically consistent nullspace impedance

Nero 有 7 个关节而任务为 6 维，因此在满任务秩姿态仍有一个冗余自由度。仅有
`J_g^T F_c` 时，这个内部自由度没有回位弹簧。Nero 使用关节参考姿态构造一个
低增益关节弹簧/阻尼，并通过质量矩阵一致的力矩投影器只保留零空间分量：

Nero has seven joints for a six-dimensional task, leaving one redundant degree
of freedom at a full-task-rank pose. `J_g^T F_c` alone supplies no restoring
spring for that internal motion. Nero forms a low-gain joint spring/damper
around the joint reference and retains only its dynamically consistent
nullspace component:

```text
tau_0 = K_0 (q_0 - q) + D_0 (qdot_0 - qdot)

N_tau = I - J_g^T (J_g M^-1 J_g^T)^+ J_g M^-1
tau_null = N_tau tau_0
```

该投影满足 `J_g M^-1 tau_null = 0`（数值容差内），因此零空间恢复力矩局部不
产生任务空间加速度。上标 `+` 是 Moore–Penrose 伪逆，仅用于任务惯量投影，不是
用伪逆把 wrench 映射成任务力矩；任务项仍严格为 `J_g^T F_c`。Piper-L 为 6 轴，
此项关闭。

The projector satisfies `J_g M^-1 tau_null = 0` up to numerical tolerance, so
the restoring torque locally produces no task acceleration. Superscript `+`
is the Moore–Penrose pseudoinverse used only in the operational-inertia
projector; it does not replace the strict task mapping `J_g^T F_c`. The term is
disabled for six-axis Piper-L.

### 5.6 与关节阻抗的局部等价 / Local joint-impedance equivalence

在一个姿态附近，若 `delta_x ≈ Jg delta_q`：

Near one configuration, with `delta_x ≈ Jg delta_q`:

```text
Kq = Jg^T Kx Jg
Dq = Jg^T Dx Jg
```

这些是完整的 `n×n` 矩阵。只取对角线会丢失关节耦合，因此不能称为严格等价。

These are full `n×n` matrices. Keeping only the diagonal loses joint
coupling and is therefore not a strict equivalence.

若从已知 `Kq/Dq` 反算空间增益，只有 `Jg` 为非奇异 `6×6` 时才存在唯一结果：

Recovering task gains from known `Kq/Dq` has a unique result only when `Jg` is
nonsingular and `6×6`:

```text
Kx = Jg^-T Kq Jg^-1
Dx = Jg^-T Dq Jg^-1
```

Piper-L 在非奇异姿态可使用这个等价式；奇异姿态不可。Nero 7 轴存在零空间，
仅给定 `Kq/Dq` 时解不唯一。严格接口会拒绝这两类非唯一情况，而不是自动使用
伪逆并隐藏额外假设。实际控制律仍直接使用 `tau=Jg^T F`，不需要反算 Kx。

Piper-L can use this equivalence at a regular pose, but not at a singular one.
Nero's seventh axis creates a nullspace, so `Kq/Dq` alone does not define a
unique task gain. The strict interface rejects both non-unique cases rather
than silently introducing pseudoinverse assumptions. The actual control law
still uses `tau=Jg^T F` directly and does not need to recover Kx.

### 5.7 广义动量观测器 / Generalized momentum observer

```text
M(q) qddot + C(q,qdot) qdot + g(q) = tau_motor + tau_ext
p = M(q) qdot
Mdot = C + C^T
pdot = tau_motor + tau_ext + C^T qdot - g
beta = g - C^T qdot = g - dT/dq
r = K_o [p - p(0) - integral(tau_motor - beta + r) dt]
```

这里的 `beta` 是由 URDF 动力学计算的已知项，不是人工/标定 bias，不能从公式中
删除；当前固定标定 bias 为零。 / Here `beta` is a known term computed from
URDF dynamics, not an empirical/calibration bias, and must not be removed from
the equation; the current fixed calibration bias is zero.

`r` 是外部关节力矩的一阶低通估计，`K_o` 的单位为 `1/s`。正的 `r_i` 表示
环境对机械臂施加了关节 `i` 正方向的广义力矩。该公式不需要 `qddot`，也不反求
`M^-1`。 / `r` is a first-order low-pass estimate of external joint torque,
and `K_o` has units `1/s`. Positive `r_i` means the environment applies a
generalized torque in joint `i`'s positive direction. The equation needs
neither `qddot` nor `M^-1`.

实现使用空间惯量前向/后向递推以 O(n) 计算 `p`。`C^T qdot` 通过
`Mdot qdot-C qdot` 获得，其中 `Mdot qdot` 是 `p=M(q)qdot` 沿 `qdot` 的单次
方向导数，不构造完整 Coriolis 矩阵。 / The implementation computes `p` in
O(n) through spatial-inertia forward/backward recursion. It obtains
`C^T qdot` from `Mdot qdot-C qdot`; `Mdot qdot` is one directional derivative
of `p=M(q)qdot` along `qdot`, without constructing a full Coriolis matrix.

## 6. 架构 / Architecture

```text
Keyboard / Pose target
          |
          v
     Screw IK target                 measured q, qdot
          |                                  |
          +----------------------+-----------+
                                 |
                                 v
                Cartesian impedance formula core
             FK -> J_s -> J_g -> e/twist -> wrench
                                 |
                    tau_task = J_g^T wrench
                                 |
             Nero: M(q) nullspace torque tau_null
                                 |
       URDF ID(q, qdot, 0) + tau_task + tau_null
                                 |
                  absolute torque clip (no delta-tau)
                                 |
              MIT: kp=0, kd=0, p_des=q, v_des=0,
                         t_ff=tau_cmd
                                 |
                         pyAgxArm / CAN
                                 |
              one shared 100 Hz measured q/qdot/motor-torque sample
                                 |
                 /arm_dynamics_state (JointState)
                                 |
                 separate momentum-observer process
                                 |
          /arm_external_joint_torque (effort = r, N.m)
                                 |
                  DLS: tau_ext = J_g^T F_ext
                                 |
              M_a xdd + D_a xd + K_a x = F_ext
                                 |
             Exp(rotation-vector), p -> screw IK
                                 |
                       planned move_j
```

旧关节 MIT 路径仍可通过 `impedance_backend:=joint` 选择，用于同机对照。

The old joint MIT path remains selectable with `impedance_backend:=joint` for
comparison on the same arm.

## 7. 数据流 / Data flow

1. 输入实测 `q`、`qdot`。 / Receive measured `q`, `qdot`.
2. PoE FK 得到 `T=[R,p]`。 / PoE FK produces `T=[R,p]`.
3. 将 `J_s` 转为工具原点 `J_g`。 / Convert `J_s` to tool-origin `J_g`.
4. 计算 base-frame `e_x` 与 `xdot`。 / Compute base-frame `e_x` and
   `xdot`.
5. 计算完整矩阵 wrench。 / Evaluate the full-matrix wrench.
6. 使用 `J_g^T` 得到 `tau_task`。 / Map through `J_g^T` to `tau_task`.
7. Nero 使用 `M(q)` 投影低增益关节参考，得到 `tau_null`；Piper 为零。 /
   Nero projects its low-gain joint reference using `M(q)` to obtain
   `tau_null`; Piper uses zero.
8. 使用 URDF 逆动力学得到 `tau_model`。 / Evaluate `tau_model` from URDF
   inverse dynamics.
9. 合成为 `tau_cmd`。 / Compose `tau_cmd`.
10. 对总力矩实施逐关节绝对上限。 / Apply the per-joint absolute limit to the
   total torque.
11. 每个轴发送 `move_mit(kp=0,kd=0,t_ff=tau_cmd)`。 / Send each axis with
    `move_mit(kp=0,kd=0,t_ff=tau_cmd)`.
12. 控制器每个 100 Hz 周期读取一次 SDK 缓存的 `q/qdot/tau_motor`，并发布
    `/arm_dynamics_state`；该流在 planned、阻抗和导纳模式都存在。 / Once per
    100 Hz cycle, the controller reads cached SDK `q/qdot/tau_motor` and
    publishes `/arm_dynamics_state`; the stream exists in planned, impedance,
    and admittance modes.
13. 独立观测器进程只订阅该 topic；它不连接 CAN，也不再次读取机械臂。 / The
    separate observer process only subscribes to that topic; it neither
    connects to CAN nor reads the arm again.
14. 观测器按消息时间戳逐帧积分；区间 `[t_{k-1},t_k]` 使用上一周期的实测
    `tau_motor[k-1]`，ROS 调度延迟不参与 `dt`。 / The observer integrates every
    timestamped sample; interval `[t_{k-1},t_k]` uses the preceding cycle's
    measured `tau_motor[k-1]`, so ROS scheduling delay does not enter `dt`.
15. 观测器发布 `/arm_external_joint_torque`；`JointState.effort` 是 `r`，单位
    N·m。 / The observer publishes `/arm_external_joint_torque`; its
    `JointState.effort` is `r` in N·m.
16. Piper-L 导纳开启时，使用 `tau_ext=J_g^T F_ext` 的阻尼最小二乘解估计
    `F_ext`，积分虚拟质量-阻尼-弹簧方程，并以旋转向量指数映射生成连续 SE(3)
    目标。 / With Piper-L admittance active, a damped least-squares solution of
    `tau_ext=J_g^T F_ext` estimates `F_ext`; the virtual mass-damper-spring
    equation is integrated and a rotation-vector exponential generates a
    continuous SE(3) target.
17. 旋量 IK 将目标转换为关节位置，并由 planned `move_j` 后端发送；该路径不
    调用 MIT。 / Screw IK converts the target to joint position and the planned
    `move_j` backend sends it; this path does not use MIT.

## 8. 模块地图 / Module map

| 模块 / Module | 责任 / Responsibility |
| --- | --- |
| `armbycontroller/impedance/cartesian.py` | 纯阻抗公式、坐标验证和完整双向等价关系（仅逆关系唯一时） / Pure impedance formula, frame validation, and full bidirectional equivalence only when the inverse is unique |
| `armbycontroller/impedance/admittance.py` | 纯导纳虚拟动力学和 `tau_ext -> F_ext` 阻尼最小二乘 / Pure admittance virtual dynamics and damped least-squares `tau_ext -> F_ext` mapping |
| `armbycontroller/lie.py` | 共享 SO(3)/SE(3) 指数、对数、空间误差旋量、伴随矩阵和空间向量原语 / Shared SO(3)/SE(3) exponentials, logarithms, space-error twist, adjoint, and spatial-vector primitives |
| `armbycontroller/screw_model.py` | URDF PoE FK、空间雅可比、RNEA 逆动力学 / URDF PoE FK, space Jacobian, RNEA inverse dynamics |
| `armbycontroller/ik/screw.py` | 使用完整 SE(3) 空间误差和 PoE 空间雅可比的数值 IK / Numerical IK using a full SE(3) space error and PoE space Jacobian |
| `armbycontroller/ik/core.py` | IK 创建、目标增量和控制器共享工具；唯一工厂是 `create_screw_solver` / IK construction, target increments, and shared controller helpers; `create_screw_solver` is the sole factory |
| `armbycontroller/ros/keyboard_controller_node.py` | ROS 键盘状态机、I/O 互锁、SDK/CAN adapter、100 Hz 实测状态发布 / ROS keyboard state machine, I/O interlock, SDK/CAN adapter, and 100 Hz measured-state publication |
| `armbycontroller/momentum_observer.py` | 无 ROS/CAN 的纯广义动量观测器 / ROS/CAN-free generalized-momentum observer |
| `armbycontroller/ros/momentum_observer_node.py` | 只订阅 `/arm_dynamics_state` 的独立 ROS adapter；发布外力矩但不控制机械臂 / Separate ROS adapter that only subscribes to `/arm_dynamics_state`; publishes external torque without controlling the arm |
| `test/test_cartesian_impedance.py` | 坐标、符号、耦合、功率与模型支撑契约 / Frame, sign, coupling, power, and model-support contracts |
| `test/test_arm_control.py` | 现有关节控制、IK、动力学和硬件 adapter 回归 / Existing joint control, IK, dynamics, and hardware-adapter regression |
| `test/test_momentum_observer.py` | 空间动量、动能梯度、残差收敛、离散稳定性及禁止 SDK/CAN 访问 / Spatial momentum, kinetic gradient, residual convergence, discrete stability, and forbidden SDK/CAN-access contracts |
| `test/test_cartesian_admittance.py` | wrench 估计、虚拟平衡、旋转向量、边界和重置契约 / Wrench-estimation, virtual-equilibrium, rotation-vector, bound, and reset contracts |

模型调用方直接依赖 `UrdfScrewModel` 和 `project_gravity_vector`；项目不再提供
`UrdfGravityModel`、`nero_mount_gravity` 或 `create_tracik_solver` 浅兼容名称。
已删除的本包 `ScrewVelocityIk` 没有运行时调用方；当前位置目标 IK 只由
`ScrewIkSolver` 实现。 / Model callers depend directly on `UrdfScrewModel`
and `project_gravity_vector`; the project no longer exposes the shallow
`UrdfGravityModel`, `nero_mount_gravity`, or `create_tracik_solver`
compatibility names. The removed package-local `ScrewVelocityIk` had no
runtime callers; `ScrewIkSolver` is the current position-target IK
implementation.

## 9. 参数、单位和默认值 / Parameters, units, and defaults

| 量 / Quantity | 形状 / Shape | 单位 / Units | 当前默认 / Current default |
| --- | --- | --- | --- |
| `impedance_backend` | string | — | `cartesian` (`joint` 可对照 / for comparison) |
| `cartesian_impedance_rotation_stiffness` | scalar | N·m/rad | `0.4`，基座 X/Y 旋转 / base-frame X/Y rotation |
| `cartesian_impedance_base_z_rotation_stiffness` | scalar | N·m/rad | `4.0`，基座 Z 旋转 / base-frame Z rotation |
| `cartesian_impedance_translation_stiffness` | scalar | N/m | `10.0` |
| `cartesian_impedance_rotation_damping` | scalar | N·m·s/rad | `0.08` |
| `cartesian_impedance_translation_damping` | scalar | N·s/m | `0.8` |
| `cartesian_impedance_nullspace_stiffness` | 1 or n | N·m/rad | `0.4`，仅 Nero / Nero only |
| `cartesian_impedance_nullspace_damping` | 1 or n | N·m·s/rad | `0.1`，仅 Nero / Nero only |
| `cartesian_impedance_torque_limit` | 1 or n | N·m | `8.0` per joint |
| `mit_command_rate` | scalar | Hz | `100.0` |
| `dynamics_state_topic` | string | — | `/arm_dynamics_state`; `effort=tau_motor` |
| `momentum_observer_enabled` | bool | — | `true`，只启动/停止独立观测进程 / only starts/stops the separate observer process |
| `momentum_observer_rate` | scalar | Hz | `100.0`，预期输入频率；计算由每条输入消息触发 / expected input rate; every input message triggers an update |
| `momentum_observer_gain` | 1 or n | 1/s | `10.0` |
| `momentum_observer_max_period` | scalar | s | `0.05`；较大数据间隙时重置 / reset after a larger stream gap |
| `external_torque_topic` | string | — | `/arm_external_joint_torque`；`effort` 为 N·m / `effort` is N·m |
| `admittance_virtual_mass` | 6 | N·m·s²/rad, kg | `[0.12,0.12,0.12,1.5,1.5,1.5]` |
| `admittance_damping` | 6 | N·m·s/rad, N·s/m | `[0.8,0.8,0.8,8,8,8]` |
| `admittance_stiffness` | 6 | N·m/rad, N/m | `[0.8,0.8,0.8,8,8,8]` |
| `admittance_wrench_deadband` | 6 | N·m, N | `[0.03,0.03,0.03,0.15,0.15,0.15]` |
| `admittance_wrench_limit` | 6 | N·m, N | `[2,2,2,8,8,8]` |
| `admittance_offset_limit` | 6 | rad, m | `[0.35,0.35,0.35,0.10,0.10,0.10]` |
| `admittance_velocity_limit` | 6 | rad/s, m/s | `[0.5,0.5,0.5,0.15,0.15,0.15]` |
| `admittance_wrench_filter_hz` | scalar | Hz | `5.0` |
| `admittance_wrench_dls_damping` | scalar | — | `0.05` |
| `admittance_wrench_timeout` | scalar | s | `0.10`; stale wrench becomes zero |
| `desired_twist` | `6` | rad/s, m/s | 由连续参考的 `Jg(q_ref) qdot_ref` 生成 / generated as `Jg(q_ref) qdot_ref` |
| `gravity` | `3` | m/s² | 由已有 URDF model 配置 / Existing URDF-model configuration |

启用 URDF 重力/逆动力学补偿时，补偿项严格为经过 `mit_gravity_scale` 和绝对
力矩限幅的模型输出，不再叠加 `mit_feedforward` 或任何固定标定 bias。
`mit_feedforward` 只在没有 URDF 补偿的关节对照 backend 中作为显式手动力矩。
/ With URDF gravity/inverse-dynamics compensation enabled, the compensation
term is strictly the scaled and absolute-bounded model output; neither
`mit_feedforward` nor any fixed calibration bias is added. `mit_feedforward`
remains an explicit manual torque only for the joint comparison backend when
URDF compensation is absent.

`mit_kp/mit_kd` 只属于 `joint` 对照 backend。Cartesian backend 无条件向固件
发送 `kp=kd=0`，防止关节 PD 与 `J^T F` 重复计算。

`mit_kp/mit_kd` belong only to the `joint` comparison backend. The Cartesian
backend always sends `kp=kd=0`, preventing native joint PD from double-counting
`J^T F`.

旋转刚度向量按基座坐标系组成
`[K_rx, K_ry, K_rz]=[K_rotation, K_rotation, K_base_z]`。独立提高 `K_base_z`
可增强主要由 J1 产生的基座 Z 旋转回正，而不同时提高腕部 X/Y 旋转刚度。它仍是
任务空间刚度，不是 J1 的直接关节 `Kp`；实际 J1 等效刚度仍由
`Kq=Jg^T Kx Jg` 和当前姿态决定。

The base-frame rotational stiffness vector is
`[K_rx, K_ry, K_rz]=[K_rotation, K_rotation, K_base_z]`. Raising `K_base_z`
strengthens the base-Z return direction produced mainly by J1 without also
raising wrist X/Y rotational stiffness. It remains a task-space stiffness,
not a direct J1 joint `Kp`; the actual equivalent J1 stiffness still depends
on the current pose through `Kq=Jg^T Kx Jg`.

## 10. 安全边界 / Safety boundaries

- 数学函数不做饱和；MIT adapter 对包含重力补偿的总力矩实施默认 ±8 N·m 绝对
  上限，不实施力矩变化率限制。 / The math function is unsaturated; the MIT
  adapter applies a default ±8 N·m absolute limit to total torque including
  gravity compensation, with no torque-rate limiter.
- `Kx/Dx` 必须对称半正定，以避免明显的主动负刚度/负阻尼配置。 / `Kx/Dx`
  must be symmetric positive semidefinite, rejecting obvious active negative
  stiffness/damping.
- Nero 零空间增益必须非负，且零空间力矩也计入同一个 ±8 N·m 总力矩上限。 /
  Nero nullspace gains must be nonnegative, and nullspace torque shares the
  same ±8 N·m total-torque envelope.
- `move_mit()` 对各轴依次调用，CAN 层不提供整批原子提交或逐帧 ACK。 /
  `move_mit()` is called sequentially per axis; CAN provides neither atomic
  batch commit nor per-frame acknowledgement here.
- 软件计算上限不是驱动器电流硬限，也不是力传感器测量。 / A software
  command limit is neither a drive-current hard limit nor a force-sensor
  measurement.
- 动量观测器是被动监视器；它不修改 `tau_cmd`，也不触发急停。 / The momentum
  observer is a passive monitor; it neither changes `tau_cmd` nor triggers an
  emergency stop.
- 观测器进程禁止访问 SDK/CAN；输入只能来自控制器复用的 100 Hz
  `/arm_dynamics_state`。 / The observer process must not access the SDK/CAN;
  its only input is the controller's reused 100 Hz `/arm_dynamics_state`
  stream.
- `I/O` 是互锁切换，不是叠加：导纳运行在 planned `move_j`，阻抗运行在 MIT；
  同时按下两个键会保持原模式。 / `I/O` are interlocked switches, not layers:
  admittance runs on planned `move_j`, impedance runs on MIT, and pressing
  both keys together leaves the current mode unchanged.
- 导纳路径不受 ±8 N·m MIT 命令限幅保护；驱动器内部 planned-position 力矩由
  固件决定。软件仅限制 wrench、虚拟速度/位移和单周期 IK 关节步长。 /
  Admittance is not protected by the ±8 N·m MIT command clip; drive torque in
  planned-position mode is determined by firmware. Software only bounds the
  wrench, virtual velocity/offset, and per-cycle IK joint step.
- 离散实现要求 `momentum_observer_gain * momentum_observer_max_period < 2`；
  不稳定组合在启动时拒绝。 / The discrete implementation requires
  `momentum_observer_gain * momentum_observer_max_period < 2`; unstable
  combinations are rejected at startup.

## 11. 已知风险 / Known risks

- SO(3)/SE(3) 对数映射在接近 180° 旋转时数值条件变差。 / SO(3)/SE(3)
  logarithms become ill-conditioned near 180-degree rotation.
- `J^T` 不需要求逆，但奇异位形仍会失去某些可控 wrench 方向。 / `J^T`
  avoids inversion, but singular configurations still lose controllable
  wrench directions.
- 零空间投影依赖 URDF 质量矩阵；错误的质量/惯量会降低动态解耦精度。在任务秩
  下降时零空间维数会增加，低增益和绝对力矩上限仍然必要。 / The nullspace
  projector depends on the URDF mass matrix; incorrect mass/inertia reduces
  dynamic decoupling accuracy. Task-rank loss increases nullspace dimension,
  so low gains and the absolute torque bound remain necessary.
- URDF 不包含摩擦、线缆力、齿隙、未知负载和驱动延迟。 / URDF omits
  friction, cable forces, backlash, unknown payload, and drive delay.
- 因此观测残差包含接触、摩擦、齿隙、负载误差、力矩跟踪误差和编码器噪声；未
  验证前不能把它解释为纯接触力矩或安全碰撞阈值。 / The residual therefore
  combines contact, friction, backlash, payload error, torque-tracking error,
  and encoder noise; before validation it is neither pure contact torque nor
  a safe collision threshold.
- `/arm_dynamics_state.effort` 是 SDK 电机状态中的力矩估计，不是六维力/力矩
  传感器测量，也不是硬件同步采样。 / `/arm_dynamics_state.effort` is the
  SDK motor-state torque estimate, not a six-axis force/torque-sensor reading
  or a hardware-synchronous sample.
- SDK 在 `connect()` 后由后台线程持续接收 CAN；控制器读取的是解析器当前缓存。
  `get_joint_angles()` 与逐关节 `get_motor_states()` 是不同反馈消息，因而同一循环
  内也不是硬件原子快照。观测器不会再次调用它们。 / After `connect()`, the SDK
  receives CAN in a background thread and the controller reads its current
  parser cache. `get_joint_angles()` and per-joint `get_motor_states()` are
  different feedback messages, so one loop still does not form a
  hardware-atomic snapshot. The observer never calls them again.
- 100 Hz Python/ROS 循环不是硬实时。 / A 100 Hz Python/ROS loop is not
  hard real time.
- 由关节残差反解 Cartesian wrench 在奇异位形附近病态；DLS 只能正则化，不能
  恢复不可观方向。 / Joint-residual-to-Cartesian-wrench inversion is
  ill-conditioned near singularities; DLS regularizes it but cannot recover
  unobservable directions.
- 导纳依赖动量残差，因而摩擦/模型误差会形成假外力；`0.10 s` 超时只将旧
  wrench 清零，随后虚拟弹簧回到入口锚点。 / Admittance inherits friction and
  model error as false external wrench; the `0.10 s` timeout only zeros stale
  wrench, after which the virtual spring returns toward its entry anchor.
- 目标位姿不连续会直接造成力矩跳变；本设计明确不使用力矩变化率限制掩盖这种
  输入，目标生成器必须自己保持连续。 / A discontinuous pose reference causes
  an immediate torque step; this design deliberately does not hide it with a
  torque-rate limiter, so the reference generator itself must remain
  continuous.
- Piper-L 的 IK 和动力学都读取
  `piper_l_with_gripper_description.xacro`，夹爪质量进入动力学；任务点仍是
  `link6`，不能把它当作指尖接触点。 / Piper-L IK and dynamics both read
  `piper_l_with_gripper_description.xacro`, so gripper mass enters dynamics;
  the task point remains `link6`, not the fingertip contact point.
- Nero 的 IK、质量矩阵和逆动力学默认读取
  `nero_with_left_revo2_description.xacro`，固定 Revo2 质量进入模型；任务点为
  `link7`。 / Nero IK, mass matrix, and inverse dynamics load
  `nero_with_left_revo2_description.xacro` by default, including the fixed
  Revo2 mass; the task point is `link7`.
- 任务空间阻抗和 IK 目标生成必须解耦，慢 IK 不能阻塞力矩刷新。 / Task
  impedance and IK target generation must be decoupled so slow IK cannot block
  torque refresh.

## 12. 构建、测试和运行 / Build, test, and run

### 构建 / Build

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
python3 -m pip install "modern_robotics>=1.1.1"
colcon build --packages-select armbycontroller --symlink-install
source install/setup.bash
```

### 公式测试 / Formula tests

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_cartesian_impedance.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_momentum_observer.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_cartesian_admittance.py
```

### 全部测试 / Full tests

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select armbycontroller
colcon test-result --verbose
```

### Nero 公式/模型仿真 / Nero formula/model simulation

下面的测试加载真实 `nero_with_left_revo2_description.xacro`，验证 7 轴质量
矩阵投影产生非零零空间力矩，并检查 `J_g M^-1 tau_null≈0`。它是确定性模型
测试，不是含摩擦、时延和 CAN 的独立物理仿真。

The following test loads the real `nero_with_left_revo2_description.xacro`,
checks that the seven-axis mass-matrix projector produces nonzero nullspace
torque, and verifies `J_g M^-1 tau_null≈0`. It is a deterministic model test,
not an independent physical simulation with friction, delay, and CAN:

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_cartesian_impedance.py \
  -k nero_revo2_nullspace
```

### Nero 实机接线 / Nero hardware wiring

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto nero_mount:=horizontal \
  execute_motion:=true impedance_backend:=cartesian \
  impedance_enabled:=false \
  cartesian_impedance_nullspace_stiffness:=0.4 \
  cartesian_impedance_nullspace_damping:=0.1 \
  cartesian_impedance_torque_limit:=8.0 \
  move_home_on_start:=false reset_emergency_stop_on_start:=true
```

`nero_mount` 必须与实际安装一致：平置用 `horizontal`，项目定义的 -90° 侧装
用 `side`。启动后保持机械臂被支撑，按 `I` 同时捕获当前末端位姿和 7 轴零空间
姿态，再进入 Cartesian MIT。 / `nero_mount` must match the installation:
use `horizontal` for a horizontal base and `side` for the project's -90° side
mount. With the arm physically supported, press `I` to capture both the tool
pose and seven-axis nullspace posture before entering Cartesian MIT.

### Piper-L 实机接线 / Piper-L hardware wiring

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto execute_motion:=true \
  impedance_backend:=cartesian impedance_enabled:=false \
  cartesian_impedance_base_z_rotation_stiffness:=4.0 \
  move_home_on_start:=false reset_emergency_stop_on_start:=true
```

启动后按 `I` 才进入 Cartesian MIT。进入时捕获当前姿态，使首周期任务误差为零；
按 `P` 后用旋量 IK 更新参考。首次实机只能在支撑机械臂、急停可触达和空旷环境下
进行。 / Press `I` after startup to enter Cartesian MIT. Entry captures the
current pose so the first task error is zero; press `P` to update references
through screw IK. First hardware trials require physical arm support, an
accessible emergency stop, and a clear workspace.

Piper-L 导纳使用同一启动命令。保持 `impedance_enabled:=false`，启动后按 `O`
捕获当前位姿并进入导纳；再次按 `O` 退出。若阻抗已开启，`O` 会先退出阻抗；若
导纳已开启，`I` 会先退出导纳。导纳中 `P`、手动 jog 和 home 被锁住。 /
Piper-L admittance uses the same launch command. Keep
`impedance_enabled:=false`, then press `O` to capture the current pose and
enter admittance; press `O` again to leave. If impedance is active, `O` exits
it first; if admittance is active, `I` exits it first. `P`, manual jog, and
home are locked while admittance is active.

实机按 `O` 前必须已有新鲜的 `/arm_external_joint_torque`（默认不超过
`0.10 s`）；否则控制器拒绝进入并提示检查该 topic。 / Before pressing `O` on
hardware, `/arm_external_joint_torque` must be fresh (no older than `0.10 s`
by default); otherwise entry is rejected with a topic diagnostic.

两个实机命令都会默认启动 100 Hz 被动动量观测器。进入 MIT 后查看： / Both
hardware commands start the passive 100 Hz momentum observer by default.
After entering MIT, inspect:

```bash
ros2 topic echo /arm_external_joint_torque
```

若只想运行控制器，启动参数追加 `momentum_observer_enabled:=false`。 / Add
`momentum_observer_enabled:=false` to run only the controller.

## 13. 诊断 / Diagnostics

公式层应依次检查：

The formula layer should be diagnosed in this order:

1. `T` 是否为合法 SE(3)。 / Is `T` valid SE(3)?
2. `J_s` 是否为 `6×n` 且顺序为 `[角; 线]`。 / Is `J_s` `6×n` and ordered
   `[angular; linear]`?
3. `J_g` 的线速度是否与 FK 有限差分一致。 / Does `J_g` linear velocity
   match an FK finite difference?
4. `e_R/e_p` 的正负方向是否使 wrench 指向目标。 / Do `e_R/e_p` signs make
   the wrench point toward the target?
5. IK 是否使用 `Log(T_d T^-1)^vee` 与同一基坐标系的 `J_s`。 / Does IK pair
   `Log(T_d T^-1)^vee` with `J_s` in the same base frame?
6. 是否满足 `tau^T qdot = F^T xdot`。 / Does
   `tau^T qdot = F^T xdot` hold?
7. `ID(q,qdot,0)` 是否只产生科氏/离心和重力支撑。 / Does
   `ID(q,qdot,0)` contain only Coriolis/centrifugal and gravity support?
8. Nero 是否满足 `J_g M^-1 tau_null≈0`。 / Does Nero satisfy
   `J_g M^-1 tau_null≈0`?
9. 日志中的 `task`、`null`、`model` 三项相加后是否等于限幅前 `total`。 /
   Do logged `task`, `null`, and `model` sum to the pre-clip `total`?
10. `/arm_dynamics_state` 是否约为 100 Hz，且 `position/velocity/effort`
    分别来自 SDK 的关节角、关节速度和电机力矩。 / Is
    `/arm_dynamics_state` near 100 Hz, with `position/velocity/effort` sourced
    from SDK joint angle, joint velocity, and motor torque?
11. 静止无接触时，残差是否稳定但可能存在摩擦/模型偏置；接触时符号是否符合关节
    正方向。 / At rest without contact, is the residual stable despite possible
    friction/model bias, and does contact follow the positive joint sign?
12. 按 `O` 后是否 `admittance=true, impedance=false`；随后按 `I` 是否先退出
    planned 导纳再进入 MIT 阻抗。 / After `O`, is admittance true and
    impedance false; after `I`, does planned admittance exit before MIT
    impedance enters?

## 14. 分阶段学习与实现路线 / Staged learning and implementation path

1. **公式核心 / Formula core**：已完成；纯函数、坐标和等价关系测试。 / Done;
   pure function, frame, and equivalence tests.
2. **MIT/CAN adapter**：已完成软件接线；`kp=kd=0`，总力矩进入 `t_ff`，仅绝对
   上限。 / Software wiring done; `kp=kd=0`, total torque through `t_ff`,
   absolute limit only.
3. **Nero 7 轴零空间 / Nero seven-axis nullspace**：已完成质量矩阵一致的力矩
   投影、真实 Revo2 模型测试和 7 轴 MIT 周期测试。 / Dynamically consistent
   torque projection, real-Revo2 model test, and complete seven-axis MIT-cycle
   test are done.
4. **仿真 adapter / Simulation adapter**：仍待完成；用独立 plant 验证符号、能量和收敛，
   不允许 controller 与 plant 使用完全相同模型掩盖误差。 / Validate signs,
   energy, and convergence with an independent plant; avoid a perfect-model
   inverse crime.
5. **目标状态机 / Reference state machine**：进入时捕获零误差 pose；连续关节
   参考生成 pose/twist。 / Capture zero-error pose on entry; generate pose and
   twist from a continuous joint reference.
6. **低增益实机 / Low-gain hardware**：先静止保持，再毫米级 IK，保存完整日志。
   / First stationary hold, then millimetre IK steps with complete logs.
7. **动量观测 / Momentum observation**：已完成只读 100 Hz topic seam、独立进程、
   O(n) 动量递推和残差发布；尚未完成摩擦辨识或碰撞阈值验证。 / The read-only
   100 Hz topic seam, separate process, O(n) momentum recursion, and residual
   publication are done; friction identification and collision-threshold
   validation remain undone.
8. **Piper-L 导纳 / Piper-L admittance**：已完成 `O` 键、I/O 互锁、DLS wrench、
   虚拟动力学、旋量 IK 和 planned adapter；实机手感与残差质量仍需低速验证。 /
   The `O` key, I/O interlock, DLS wrench, virtual dynamics, screw IK, and
   planned adapter are implemented; hardware feel and residual quality still
   require low-speed validation.

## 15. 练习 / Exercises

1. 推导为什么 `J_s` 的线速度行必须减去 `[p]x J_omega`。 / Derive why the
   linear rows of `J_s` require subtracting `[p]x J_omega`.
2. 用随机 `J、F、qdot` 验证虚功等式。 / Verify virtual work using random
   `J`, `F`, and `qdot`.
3. 构造非对角 `Kx`，观察 `J^T Kx J` 的关节耦合项。 / Construct a
   non-diagonal `Kx` and inspect coupling in `J^T Kx J`.
4. 在满秩 `6×6` J 上完成 `Kx -> Kq -> Kx` 往返，并解释为什么 7 轴解不唯一。
   / Round-trip `Kx -> Kq -> Kx` through a full-rank `6×6` J and explain why
   a seven-axis inverse is non-unique.
5. 比较 `ID(q,0,0)` 与 `ID(q,qdot,0)`，区分纯重力和完整模型支撑。 / Compare
   `ID(q,0,0)` with `ID(q,qdot,0)` to separate gravity from full model support.
6. 用 FK 有限差分验证 Piper-L 某姿态下 `J_g qdot`。 / Validate `J_g qdot`
   at a Piper-L pose using an FK finite difference.
7. 对 Nero 的 `6×7` Jacobian 做 SVD，取最后一个右奇异向量作为姿态偏差，验证
   零空间恢复力矩非零且 `J_g M^-1 tau_null≈0`。 / Use the final right singular
   vector of Nero's `6×7` Jacobian as a posture error and verify nonzero
   restoring torque with `J_g M^-1 tau_null≈0`.
8. 证明 `pdot=tau+tau_ext+C^T qdot-g`，并用一自由度恒定外力矩仿真验证 `r`
   收敛。 / Derive `pdot=tau+tau_ext+C^T qdot-g` and verify residual
   convergence with a one-DOF constant-external-torque simulation.

## 16. 术语表 / Glossary

| 中文 | English | 含义 / Meaning |
| --- | --- | --- |
| 关节阻抗 | Joint impedance | Joint-space spring-damper torque law |
| 笛卡尔阻抗 | Cartesian impedance | Tool-space wrench spring-damper law |
| 笛卡尔导纳 | Cartesian admittance | External wrench drives a virtual mass-damper-spring pose |
| 互锁 | Interlock | Entering impedance exits admittance and vice versa |
| 空间雅可比 | Space Jacobian | PoE space-twist Jacobian `[omega; v]` |
| 几何雅可比 | Geometric Jacobian | Tool-origin velocity Jacobian `[omega; p_dot]` |
| wrench | Wrench | Moment-force vector `[M; F]` |
| 虚功 | Virtual work | Power-dual relation `tau=J^T F` |
| 模型支撑 | Model support | `C(q,qdot)qdot+g(q)` from URDF inverse dynamics |
| 冗余自由度 | Redundant degree of freedom | Joint motion not required by the six-dimensional task |
| 零空间阻抗 | Nullspace impedance | Projected joint spring/damper for redundant posture |
| 动力学一致投影 | Dynamically consistent projection | Torque projection satisfying `J M^-1 tau_null=0` |
| 半正定 | Positive semidefinite | Matrix with nonnegative quadratic energy |
| 旋量 IK | Screw IK | IK based on PoE screws and SE(3) logarithms |
| 旋转向量 | Rotation vector | `Log(R_d R^T)^vee` 轴角姿态误差，不是 RPY 差值 / Axis-angle orientation error, not an RPY difference |
| 空间误差旋量 | Space-error twist | `Log(T_d T^-1)^vee`，表达在基坐标系并与 `J_s` 配对 / Base-frame SE(3) error paired with `J_s` |
| 广义动量 | Generalized momentum | `p=M(q)qdot` |
| 动量观测残差 | Momentum-observer residual | 外部关节力矩的一阶低通估计；也包含未建模扰动 / First-order estimate of external joint torque that also contains unmodelled disturbances |
| 阻尼最小二乘 | Damped least squares | Regularized solve of `tau_ext=J^T F_ext` near rank loss |
| MIT 命令 | MIT command | Native motor command with `p_des/v_des/kp/kd/t_ff` |

## 17. 主要资料 / Primary sources

- Kevin M. Lynch and Frank C. Park, *Modern Robotics: Mechanics, Planning,
  and Control*, Cambridge University Press, 2017. Companion material:
  https://modernrobotics.northwestern.edu/
- Oussama Khatib, “A Unified Approach for Motion and Force Control of Robot
  Manipulators: The Operational Space Formulation,” IEEE Journal of Robotics
  and Automation, 1987. DOI: 10.1109/JRA.1987.1087068.
- Neville Hogan, “Impedance Control: An Approach to Manipulation,” ASME Journal
  of Dynamic Systems, Measurement, and Control, 1985.
- Christian Ott, Alin Albu-Schäffer, Andreas Kugi, and Gerd Hirzinger,
  “Cartesian Impedance Control of Flexible Joint Robots: A Passivity-Based
  Approach,” IEEE Transactions on Robotics, 2008,
  https://doi.org/10.1109/TRO.2008.915438.
- Alessandro De Luca and Raffaella Mattone, “Sensorless Robot Collision
  Detection and Hybrid Force/Motion Control,” IEEE ICRA, 2005, pp. 999-1004.
  DOI: 10.1109/ROBOT.2005.1570247. 广义动量残差公式来源。 / Primary source
  for the generalized-momentum residual.
- Matthias Mayr and Julian M. Salt-Ducaju, “A C++ Implementation of a
  Cartesian Impedance Controller for Robotic Manipulators,” JOSS 9(93), 5194,
  2024, https://doi.org/10.21105/joss.05194. 本项目采用其 `J^T` 任务力矩
  结构作为对照，但有意不采用其力矩变化率限制。 / This project uses its
  `J^T` task-torque structure as a reference while deliberately excluding its
  torque-rate limiter.
- 参考实现 / Reference implementation:
  https://github.com/matthias-mayr/Cartesian-Impedance-Controller
- 本仓库 `armbycontroller/screw_model.py`：PoE、空间雅可比和 RNEA 的实际实现。
  / This repository's `armbycontroller/screw_model.py`: the implemented PoE,
  space Jacobian, and RNEA model.
- 本仓库 `armbycontroller/ros/keyboard_controller_node.py`：当前 ROS 状态机、
  AGX MIT/planned adapter 与互锁基线。 / This repository's
  `armbycontroller/ros/keyboard_controller_node.py`: the current ROS state
  machine, AGX MIT/planned adapters, and interlock baseline.
- AgileX Robotics, Piper API：后台读取线程、`MessageAbstract.timestamp`、
  `get_joint_angles()` 与高频 `get_motor_states()`：
  https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md
- AgileX Robotics, Nero API：相同状态接口（7 轴）：
  https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/nero_api.md
