# 项目学习指南 / Project Study Guide

## 1. 项目目标 / Project goal

本项目通过 ROS 2、键盘输入和 AGX SDK 控制 Nero 与 Piper-L。当前分支
`feature/cartesian-impedance-step-by-step` 的目标，是从已经存在的关节 MIT
阻抗出发，逐层建立公式清楚、坐标一致、可测试的空间笛卡尔阻抗，并为
Nero/Piper-L 增加由动量观测外力驱动、分为零力和阻力两种的笛卡尔导纳模式，
以及按任务方向互补分解的阻抗/导纳混合模式。

This project controls Nero and Piper-L through ROS 2, keyboard input, and the
AGX SDK. The goal of `feature/cartesian-impedance-step-by-step` is to start
from the existing joint MIT impedance and build a formula-explicit,
frame-consistent, testable Cartesian impedance controller in stages, plus
zero-force and resistive Cartesian-admittance modes for Nero and Piper-L,
driven by momentum-observer external torque, and a direction-selective,
complementary impedance/admittance hybrid mode.

当前阶段已把纯数学核心接入 `mit_tick()` 和 AGX MIT/CAN。默认
`impedance_backend:=cartesian`；设为 `joint` 可以保留旧关节 MIT 作为对照。
导纳下游采用已验证 `nero_admittance_mit` 的受限旋量 Jacobian 速度 IK 与实测状态重锚定 MIT
结构，同时保留本项目的两种虚拟导纳律、观测器和互锁。软件集成测试通过不代表
已经验证当前整合版本在真实机械臂上的物理稳定性。

The pure formula core is now wired into `mit_tick()` and AGX MIT/CAN. The
default is `impedance_backend:=cartesian`; selecting `joint` retains the old
joint MIT backend for comparison. Admittance uses the hardware-validated
`nero_admittance_mit` bounded screw-Jacobian velocity IK and measured-state-reanchored MIT
structure while retaining this project's two virtual laws, observer, and
interlock. Passing software integration tests does not establish physical
stability of the integrated controller on real hardware.

交互后端严格互锁：`I` 切换阻抗，`O` 切换导纳，`H` 切换混合；跨模式切换严格经过
`当前模式 -> 普通 planned-position 模式 -> 目标模式`，普通模式恢复失败时拒绝
进入目标模式，三个交互模式绝不同时运行。两种机械臂均支持导纳和混合，但机器人
相关参数仍分别位于各自 YAML。 / Interaction backends are strictly
interlocked: `I` toggles impedance, `O` toggles admittance, and `H` toggles
hybrid control. A cross-mode transition strictly follows `current mode ->
normal planned-position mode -> target mode`; failure to restore normal mode
rejects the target mode, and the three interaction modes never run together.
Both arms support admittance and hybrid control, with independent
robot-specific YAML tuning.

真实硬件启动统一采用两阶段连接。第一阶段以 SDK `default` profile 创建探测
实例，连接后取得并保存完整 firmware 字典，随后必定断开；第二阶段根据保存的
`software_version` 选择 Nero/Piper-L 对应驱动 profile。探测实例断开后等待默认
`0.5 s`，才创建全新的正式控制实例。
Nero 和 Piper-L 探测阶段都会短暂请求使能，读取固件后立即请求失能；不会发送
运动或固件写入命令。 /
Real-hardware startup uses one two-stage connection for both arms. Stage one
creates a probe with
the SDK `default` profile, connects, saves the complete firmware dictionary,
and always disconnects. After a default `0.5 s` post-disconnect delay, stage
two selects the Nero/Piper-L driver profile from the saved `software_version`
and creates a distinct formal control instance.
Both Nero and Piper-L probes briefly request enable and immediately request
disable after the firmware read; they send no motion or firmware-write command.

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

Nero 默认的“零力”是柔顺零力，不是自由积分器。它用弱保持、速度阻尼与有限
粘/滑阻力抑制动量观测偏置和摩擦；Piper-L 默认的 `resistive` 使用更明确的
阻尼与回中弹簧： / Nero's default zero-force feel is soft zero force, not a
free integrator. Weak holding, velocity damping, and bounded stick/slip
resistance suppress momentum-observer bias and friction; Piper-L's default
`resistive` mode uses explicit damping and a restoring spring:

```text
soft zero: M xdd + D0 xd + Kh x + Fstick/slip = Fext
resistive: M xdd + Dr xd + Kr x               = Fext
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
tau_raw   = ID(q, qdot, 0)
          = C(q, qdot) qdot + g(q)
tau_model = s_model .* tau_raw

tau_posture = K_p (q_ref - q) + D_p (qdot_ref - qdot)
tau_cmd = tau_task + tau_null + tau_posture + tau_model
```

`s_model` 是 `[0,1]` 内的标量或逐关节向量，由
`cartesian_impedance_model_scale` 配置。它使用实测 `q/qdot`，不把 IK 参考
速度或轨迹加速度混入模型支撑。

`s_model` is a scalar or per-joint vector in `[0,1]`, configured by
`cartesian_impedance_model_scale`. The model uses measured `q/qdot`; IK
reference velocity and trajectory acceleration are not mixed into support.

`tau_cmd` 是当前采样的直接公式结果，控制器不读取上一周期力矩，也不实现
`delta_tau`/`delta_tau_max` 力矩变化率限制。MIT adapter 只实施逐关节绝对
力矩上限，默认 ±8 N·m，且这个总量已经包含模型支撑。

`tau_cmd` is the immediate equation result for the current sample. The
controller does not read the previous torque command and does not implement a
`delta_tau`/`delta_tau_max` torque-rate limiter. The MIT adapter applies only a
per-joint absolute limit, ±8 N·m by default, to the total that already includes
model support and optional joint-posture torque.

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

`M(q)` 使用一次复合刚体算法（CRBA）树回扫计算，不再通过每个单位关节加速度
重复调用 RNEA。该实现与原 `modern_robotics.MassMatrix` 在 Nero/Piper-L 及附件
模型上保持数值等价。保持关节参考不变时，controller adapter 复用参考位姿和参考
Jacobian；实测状态的 FK、Jacobian、质量矩阵和逆动力学仍逐周期更新。 /
`M(q)` is computed by one composite-rigid-body algorithm (CRBA) tree sweep
instead of repeated RNEA calls for unit joint accelerations. It remains
numerically equivalent to the former `modern_robotics.MassMatrix` path for
Nero, Piper-L, and accessory models. While the joint reference is unchanged,
the controller adapter reuses its reference pose and Jacobian; measured-state
FK, Jacobian, mass matrix, and inverse dynamics still update every cycle.

该投影满足 `J_g M^-1 tau_null = 0`（数值容差内），因此零空间恢复力矩局部不
产生任务空间加速度。上标 `+` 是 Moore–Penrose 伪逆，仅用于任务惯量投影，不是
用伪逆把 wrench 映射成任务力矩；任务项仍严格为 `J_g^T F_c`。Piper-L 为 6 轴，
此项关闭。

The projector satisfies `J_g M^-1 tau_null = 0` up to numerical tolerance, so
the restoring torque locally produces no task acceleration. Superscript `+`
is the Moore–Penrose pseudoinverse used only in the operational-inertia
projector; it does not replace the strict task mapping `J_g^T F_c`. The term is
disabled for six-axis Piper-L.

### 5.6 Nero 关节选择性混合姿态阻抗 / Nero joint-selective hybrid posture impedance

纯笛卡尔阻抗只保证末端位姿恢复，不保证每个关节都有独立弹簧。2026-08-17 的
Nero 实机记录中，J4 的运动学零空间分量约为 `2e-8`，因此增加投影后的
`cartesian_impedance_nullspace_stiffness[3]` 几乎不会产生 J4 力矩。Nero 在
controller adapter 中额外计算一个未投影的低增益姿态项。2026-08-17 10:27 的
实机记录与人工顺应性检查随后显示 J2/J3 略软、其他关节略硬，其中 J4 更明显：

Pure Cartesian impedance restores tool pose but does not guarantee an
independent spring at every joint. In the 2026-08-17 Nero hardware trace, the
J4 component of the kinematic nullspace was about `2e-8`, so increasing the
projected `cartesian_impedance_nullspace_stiffness[3]` produces essentially no
J4 torque. The Nero controller adapter therefore adds a low-gain, unprojected
posture term. The 2026-08-17 10:27 hardware trace and manual compliance check
then showed J2/J3 slightly softer than desired while the other joints were
slightly firmer, especially J4:

```text
tau_posture = K_p (q_ref - q) + D_p (qdot_ref - qdot)
```

当前 J2/J3/J4 非零：`K_p=[0,0.5,0.5,0.6,0,0,0] N·m/rad`、
`D_p=[0,0.08,0.08,0.12,0,0,0] N·m·s/rad`。同时 Nero 各向同性旋转刚度从
`2.0` 小步降至 `1.9 N·m/rad`，平移刚度从 `75` 降至 `70 N/m`。该项故意不乘
`N_tau`，所以能直接平衡 J2/J3/J4 的实机手感，但也会影响末端任务；这是“笛卡尔
阻抗 + 关节选择性姿态阻抗”的混合控制，不是严格纯笛卡尔阻抗。它仍在 100 Hz
软件外环计算、计入同一个 ±8 N·m 总力矩限幅，并通过 `t_ff` 下发；固件 MIT
`kp=kd=0` 不变。Piper-L 配置不启用该项。

J2/J3/J4 are nonzero by default:
`K_p=[0,0.5,0.5,0.6,0,0,0] N·m/rad` and
`D_p=[0,0.08,0.08,0.12,0,0,0] N·m·s/rad`. Nero's isotropic rotational
stiffness is also reduced slightly from `2.0` to `1.9 N·m/rad`, and
translation stiffness from `75` to `70 N/m`. The posture term intentionally
bypasses `N_tau`, directly balancing J2/J3/J4 hardware feel while also
affecting the tool task. This is hybrid Cartesian-plus-joint-selective-posture
impedance, not strict pure Cartesian impedance. It remains a 100 Hz software
outer-loop term, shares the same ±8 N·m total-torque clip, and is sent through
`t_ff`; firmware MIT `kp=kd=0` is unchanged. Piper-L does not enable it.

### 5.7 与关节阻抗的局部等价 / Local joint-impedance equivalence

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

### 5.8 广义动量观测器 / Generalized momentum observer

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

### 5.9 导纳受限旋量速度 IK 与 MIT / Bounded screw velocity IK and MIT

导纳状态输出基坐标系 twist `v_a=[omega;v]`。受限旋量速度 IK 从同一 PoE 模型
取得 `J_s(q)`，按第 4 节转换为工具原点 `J_g(q)`，再求以下带约束的加权阻尼
最小二乘解： / The admittance state outputs base-frame twist
`v_a=[omega;v]`. Bounded Screw Velocity IK obtains `J_s(q)` from the shared
PoE model, converts it to tool-origin `J_g(q)` as in Section 4, and solves the
following constrained weighted damped-least-squares problem:

```text
dq_ref = argmin ||W (Jg dq - v_a)||^2 + lambda^2 ||dq||^2
subject to |dq_i| <= dq_limit_i
           q_lower_i + margin <= q_i + dq_i dt <= q_upper_i - margin

q_ref = q_measured + dq_ref dt
tau_est = kp (q_ref-q) + kd (dq_ref-qdot) + tau_ff
```

`q_ref` 每周期从实测位置重锚定，不积分上一周期关节参考。共享模型补偿 module
产生缩放的 URDF 重力请求；共享 MIT 安全包络再调整 `tau_ff`，使 `tau_est` 位于
逐关节上限。若单靠 PD 已经无法在允许的 `tau_ff` 内满足上限，该周期拒绝并退出
到普通 hold。 / `q_ref` is
reanchored to measured position every cycle and never integrates the previous
joint reference. The shared Model Compensation module produces scaled URDF
gravity, then the shared MIT Safety Envelope adjusts `tau_ff` so `tau_est`
stays within each joint limit. If PD feedback alone
cannot be counteracted within the allowed `tau_ff`, the cycle is rejected and
the controller exits to a normal hold.

### 5.10 Twist 层互补混合控制 / Complementary Twist-level hybrid control

混合模式仍遵守本项目的六维顺序 `[rx,ry,rz,x,y,z]=[角;线]`。配置
`hybrid_admittance_axes` 生成对角选择矩阵 `S_a`，互补阻抗选择矩阵严格取
`S_i=I-S_a`，因此每个任务方向只有一个外环拥有。默认值 `z` 对应
`S_a=diag(0,0,0,0,0,1)`：基座 Z 平移走导纳，其余五维围绕按 `H` 时捕获的
末端位姿走阻抗。 / Hybrid mode retains the project ordering
`[rx,ry,rz,x,y,z]=[angular;linear]`. `hybrid_admittance_axes` constructs the
diagonal selection matrix `S_a`, and the complementary impedance selection is
strictly `S_i=I-S_a`, so one outer loop owns each task direction. The default
`z` gives `S_a=diag(0,0,0,0,0,1)`: base-frame Z translation is admittance,
while the remaining five dimensions use impedance around the pose captured
when `H` is pressed.

```text
W_a     = S_a (W_ext - W_des)
V_a     = S_a Admittance(W_a)
W_i     = S_i [K e_X + D (0 - V_measured)]
dq_ref  = bounded_screw_velocity_IK(V_a)
q_ref   = q_measured + dq_ref dt
tau_ff  = Jg^T W_i + tau_null + C(q,dq)dq + g(q)
tau_est = kp (q_ref-q) + kd (dq_ref-dq) + tau_ff
```

`hybrid_desired_wrench` 的顺序为 `[Mx,My,Mz,Fx,Fy,Fz]`，只有 `S_a` 选中的
分量进入导纳。实现由一个 `HybridCartesianController` 独占输出：选择、导纳
Twist、互补阻抗 wrench、受限旋量速度 IK、模型补偿和 MIT 包络在同一周期合成，
不会同时启动原来的纯阻抗与纯导纳 controller。混合模式复用统一交互安全对象的
参考/实测速度边界，并使用导纳低增益 MIT 参数；包含阻抗、零空间、模型和 PD 的估算总力矩实施逐关节
最大 `8 N.m` 上限。当前选择轴固定表达在 `base_link`；曲面法向动态投影尚未实现。
/
`hybrid_desired_wrench` is ordered `[Mx,My,Mz,Fx,Fy,Fz]`, and only components
selected by `S_a` enter admittance. One `HybridCartesianController` owns the
output: selection, admittance Twist, complementary impedance wrench, bounded
screw-velocity IK, model compensation, and the MIT envelope are combined in
one cycle. The existing pure impedance and pure admittance controllers are not
run simultaneously. Hybrid mode reuses the unified interaction reference and
measured-speed limits plus admittance low-gain MIT tuning, and applies a
per-joint maximum `8 N.m`
estimated-total-torque envelope including impedance, nullspace, model, and PD
terms. Selection axes are currently fixed in `base_link`; dynamic
surface-normal projection is not yet implemented.

## 6. 架构 / Architecture

```text
Keyboard / IK reference       measured q/qdot/tau       observer wrench
          |                            |                       |
          +----------------------------+-----------------------+
                                       |
                                ControlInput
                                       |
       +--------------+--------------+----------------+----------------+
       |              |              |                |                |
 joint_impedance  cartesian_imp.  cartesian_adm.  hybrid_cartesian    |
       |              |              |           Sa Vadm + Si Wimp    |
       |          J^T W + model   admittance Twist  + screw IK        |
       |              |          + screw IK + MIT  + one MIT output   |
       +--------------+--------------+----------------+----------------+
                                      |
                                 ControlResult
                                      |
                       +---------------------------+-------------------+
                       |                                               |
               ROS/AGX hardware adapter                       control_sample
                       |                                               |
                  pyAgxArm / CAN                     /arm_control_sample (JSON)
                                                                       |
                                                          experiment recorder
                                                                       |
                                           manifest + samples + events + summary
```

`ControlEngine` 只接受标准化 `ControlInput`，并返回 `ControlResult`；当前四个
交互 adapter 都生成 MIT 命令。adapter 不访问 ROS、磁盘或 CAN；ROS 节点保留
模式互锁、状态采集、安全检查和硬件发送。旧关节 MIT 路径仍可通过
`impedance_backend:=joint` 选择，用于同机对照。

`ControlEngine` consumes only normalized `ControlInput` and returns a
`ControlResult`; all four current interaction adapters produce MIT commands.
The controller adapters do not access
ROS, disk, or CAN; the ROS node retains interlocks, acquisition, safety checks,
and hardware transmission. The old joint MIT path remains selectable with
`impedance_backend:=joint` for comparison on the same arm.

四个交互 adapter 共用六个控制律之外的 deep module：笛卡尔任务几何、模型补偿、
交互安全边界、MIT 安全包络、控制周期 guard 和交互模式生命周期。K/D/M、零空间、
摩擦/保持及参考生成仍分别留在 `impedance/`、`admittance/` 与 `hybrid/`，因此共享
安全语义不会混合各控制模式的手感。 / The four interaction adapters share
six deep modules outside their control laws: Cartesian Task Geometry, Model
Compensation, Interaction Safety Limits, MIT Safety Envelope, Control Cycle
Guard, and Interaction Mode Lifecycle. K/D/M laws, nullspace,
friction/holding, and reference generation remain local to `impedance/`,
`admittance/`, and `hybrid/`, so shared safety semantics do not merge their
control feel.

工作区的 `nero_admittance_mit` 是下游结构的实机验证基线。本仓库现已采用相同的
`导纳速度 -> 受限旋量 Jacobian IK -> 实测 q 重锚定 -> 低增益 MIT`，但保留自己的
`zero_force`/`resistive` 动力学、统一观测器、I/O/H 互锁和 8 N·m 安全上限。 /
The workspace's `nero_admittance_mit` is the hardware-validated downstream
baseline. This repository now uses the same `admittance twist -> bounded
screw-Jacobian IK -> measured-q reanchoring -> low-gain MIT` chain while retaining
its own `zero_force`/`resistive` dynamics, shared observer, I/O/H interlock, and
8 N·m safety ceiling.

## 7. 数据流 / Data flow

进入以下控制周期前，硬件 adapter 先执行：`DEFAULT 探测连接 -> 保存设备信息 ->
断开 -> 等待 0.5 s -> 按检测 profile 正式连接`。Nero 的边界为
`1.11/1.12/1.20`；Piper-L 的
边界为 `S-V1.8-3/8/9`，与当前 pyAgxArm profile 定义一致。无法取得或解析
`software_version` 时启动失败，不会用猜测 profile 建立控制连接。 / Before the
following cyclic flow, the hardware adapter runs `DEFAULT probe connection ->
save device data -> disconnect -> wait 0.5 s -> formal connection with the
detected profile`. Nero boundaries are `1.11/1.12/1.20`; Piper-L boundaries are
`S-V1.8-3/8/9`, matching the current pyAgxArm profile definitions. Startup
fails if `software_version` cannot be obtained or parsed; no guessed control
profile is used.

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
8. Nero 计算未投影的 J2/J3/J4 `tau_posture`；Piper 为零。 / Nero evaluates
   the unprojected J2/J3/J4 `tau_posture`; Piper uses zero.
9. 共享模型补偿 module 按 adapter 选择 `g(q)`、`C(q,qdot)qdot+g(q)` 或完整
   逆动力学，得到 `tau_model`。 / The shared Model Compensation module selects
   gravity, bias, or full inverse dynamics for each adapter to obtain
   `tau_model`.
10. 合成为 `tau_cmd`。 / Compose `tau_cmd`.
11. 共享 MIT 安全包络组合 feedback/model/task/auxiliary，并对估算总力矩实施
    逐关节绝对上限。 / The shared MIT Safety Envelope combines feedback,
    model, task, and auxiliary torque and applies the per-joint absolute limit
    to estimated total torque.
12. 每个轴发送 `move_mit(kp=0,kd=0,t_ff=tau_cmd)`。 / Send each axis with
    `move_mit(kp=0,kd=0,t_ff=tau_cmd)`.
13. 控制器每个 100 Hz 周期读取一次 SDK 缓存的 `q/qdot/tau_motor`，并发布
    `/arm_dynamics_state`；该流在 planned、阻抗、导纳和混合模式都存在。 / Once per
    100 Hz cycle, the controller reads cached SDK `q/qdot/tau_motor` and
    publishes `/arm_dynamics_state`; the stream exists in planned, impedance,
    admittance, and hybrid modes.
14. 独立观测器进程只订阅该 topic；它不连接 CAN，也不再次读取机械臂。 / The
    separate observer process only subscribes to that topic; it neither
    connects to CAN nor reads the arm again.
15. 观测器按消息时间戳逐帧积分；区间 `[t_{k-1},t_k]` 使用上一周期的实测
    `tau_motor[k-1]`，ROS 调度延迟不参与 `dt`。 / The observer integrates every
    timestamped sample; interval `[t_{k-1},t_k]` uses the preceding cycle's
    measured `tau_motor[k-1]`, so ROS scheduling delay does not enter `dt`.
15. 观测器发布 `/arm_external_joint_torque`；`JointState.effort` 是 `r`，单位
    N·m。 / The observer publishes `/arm_external_joint_torque`; its
    `JointState.effort` is `r` in N·m.
16. 任一机械臂导纳或混合开启时，共享笛卡尔任务几何使用
    `tau_ext=J_g^T F_ext` 的阻尼最小二乘解估计 `F_ext`。`zero_force` 积分
    `M*xdd+D0*xd+Kh*x+Fstick/slip=F_ext`，`resistive` 积分
    `M*xdd+Dr*xd+Kr*x=F_ext`；两者输出连续导纳 twist，并保留有界 SE(3) 状态
    用于诊断和回中动力学。 /
    With admittance or hybrid control active on either arm, a
    damped-least-squares solution of
    `tau_ext=J_g^T F_ext` estimates `F_ext`. `zero_force` integrates
    `M*xdd+D0*xd+Kh*x+Fstick/slip=F_ext`, while `resistive` integrates
    `M*xdd+Dr*xd+Kr*x=F_ext`; both output a continuous admittance twist and
    retain bounded SE(3) state for diagnostics and restoring dynamics.
17. 纯导纳把全部 Twist 送入受限旋量速度 IK；混合先用 `S_a` 保留选中方向的
    导纳 Twist，同时用 `S_i=I-S_a` 在其余方向生成阻抗 wrench。两者均从 PoE
    `J_s` 构造 `J_g` 并用加权 DLS 求 `dq_ref`，同时实施关节速度和预测位置边界；每周期令
    `q_ref=q_measured+dq_ref*dt`，再以低增益 `kp/kd`、模型重力前馈和估算总力矩
    上限发送 `move_mit`。 / Pure admittance sends the full Twist to Bounded
    Screw Velocity IK; hybrid control first retains selected Twist directions
    with `S_a` and creates impedance wrench on the complement
    `S_i=I-S_a`. Both build `J_g` from PoE `J_s` and solve weighted DLS for
    `dq_ref` while enforcing
    joint-velocity and predictive-position bounds. Each cycle sets
    `q_ref=q_measured+dq_ref*dt`, then sends `move_mit` with low `kp/kd`, model
    gravity feedforward, and an estimated-total-torque limit.
18. 每个实际执行的 controller 周期将同一 `ControlInput` 与 `ControlResult` 合并
    为 schema v1 `Control Sample`，发布到 `/arm_control_sample`；缺失反馈使用
    validity flag，不写 NaN。 / Every executed controller cycle combines the
    same `ControlInput` and `ControlResult` into a schema-v1 Control Sample on
    `/arm_control_sample`; missing feedback uses validity flags, never NaN.
19. controller 使能、退出和急停作为离散 `Control Event` 发布到
    `/arm_control_event`。 / Controller enable, exit, and emergency stop are
    published as discrete Control Events on `/arm_control_event`.
20. 可选 recorder 进程订阅两个 JSON topic，流式写入 `samples.jsonl` 和
    `events.jsonl`；启动/收尾原子写 `manifest.json` 与 `summary.json`。 /
    The optional recorder process subscribes to both JSON topics, streams
    `samples.jsonl` and `events.jsonl`, and atomically writes `manifest.json`
    and `summary.json` at start/close.

## 8. 模块地图 / Module map

| 模块 / Module | 责任 / Responsibility |
| --- | --- |
| `armbycontroller/cartesian/spatial.py` | 阻抗/导纳唯一共用的笛卡尔任务几何：`[角;线]`、SE(3)、tool-origin Jacobian 和虚功双向映射；不含 K/D/M / Sole Cartesian Task Geometry shared by impedance/admittance: ordering, SE(3), tool-origin Jacobian, and bidirectional virtual-work mappings, without K/D/M |
| `armbycontroller/impedance/cartesian.py` | 纯阻抗公式、误差、增益等价和 Nero 动力学一致零空间 / Pure impedance law, error, gain equivalence, and Nero dynamically consistent nullspace |
| `armbycontroller/impedance/controllers.py` | 关节 MIT 与笛卡尔阻抗 controller adapter，以及 J2/J3/J4 姿态项；模型补偿和 MIT 包络委托给共享 control module / Joint MIT and Cartesian-impedance controller adapters plus J2/J3/J4 posture terms; model compensation and MIT envelope are delegated to shared control modules |
| `armbycontroller/admittance/core.py` | 两种导纳内部共用的输入整形、二阶积分、SE(3) 目标和安全边界 / Input conditioning, second-order integration, SE(3) target, and safety bounds shared internally by admittance modes |
| `armbycontroller/admittance/zero_force.py` | Nero 优先的弱保持、阻尼、粘/滑抗漂移柔顺零力 / Nero-first anti-drift soft zero force with weak holding, damping, and stick/slip resistance |
| `armbycontroller/admittance/resistive.py` | 正阻尼和正回中刚度的阻力导纳 / Resistive admittance with positive damping and restoring stiffness |
| `armbycontroller/admittance/controller.py` | 导纳 twist 到受限旋量速度 IK、实测位置重锚定和低增益 MIT 的 controller adapter / Controller adapter from admittance twist to bounded screw velocity IK, measured-position reanchoring, and low-gain MIT |
| `armbycontroller/hybrid/selection.py` | 按项目 `[rx,ry,rz,x,y,z]` 顺序解析固定基座轴并生成任务选择 mask / Parses fixed base-frame axes in project order and creates task-selection masks |
| `armbycontroller/hybrid/controller.py` | 单一所有者的互补混合 adapter：选中轴导纳 Twist、其余轴阻抗 wrench、受限旋量 IK、模型补偿和一个 MIT 包络 / One-owner complementary hybrid adapter: selected-axis admittance Twist, remaining-axis impedance wrench, bounded screw IK, model compensation, and one MIT envelope |
| `armbycontroller/control/core.py` | 统一 `ControlInput -> ControlResult` interface、MIT/planned 命令类型、`ControlEngine` 与 schema v1 sample / Unified controller interface, command types, engine, and schema-v1 sample |
| `armbycontroller/control/model_compensation.py` | 阻抗、导纳与混合共享的重力、偏置和完整逆动力学模型补偿 / Gravity, bias, and full inverse-dynamics Model Compensation shared by impedance, admittance, and hybrid control |
| `armbycontroller/control/mit.py` | 统一 MIT Safety Envelope、总力矩可行性和力矩分解诊断 / Shared MIT Safety Envelope, total-torque feasibility, and torque-decomposition diagnostics |
| `armbycontroller/control/safety.py` | `InteractionSafetyLimits` 统一四个 MIT adapter 的力矩、参考/实测速度和关节边界，并提供反馈完整性、周期及持续/硬速度 guard / `InteractionSafetyLimits` unifies torque, reference/measured speed, and joint boundaries for all four MIT adapters and provides feedback, period, and sustained/hard-speed guards |
| `armbycontroller/control/interaction.py` | `normal/impedance/admittance/hybrid` 互锁和强制经过普通模式的迁移路径 / Normal/impedance/admittance/hybrid interlock and normal-mediated transition paths |
| `armbycontroller/experiment/core.py` | `ExperimentRun` 生命周期、汇总指标、sink interface、Memory/JSONL adapter / Experiment lifecycle, metrics, sink interface, and Memory/JSONL adapters |
| `armbycontroller/hardware/connection.py` | Nero/Piper-L 共用的 DEFAULT 探测、版本映射、断开和 profile 化正式重连 / Shared DEFAULT probe, version mapping, disconnect, and profile-specific formal reconnect for Nero/Piper-L |
| `armbycontroller/modeling/lie.py` | 共享 SO(3)/SE(3) 指数、对数、空间误差旋量、伴随矩阵和空间向量原语 / Shared SO(3)/SE(3) exponentials, logarithms, space-error twist, adjoint, and spatial-vector primitives |
| `armbycontroller/modeling/screw_model.py` | URDF PoE FK、空间雅可比、RNEA 逆动力学和一次树回扫 CRBA 质量矩阵 / URDF PoE FK, space Jacobian, RNEA inverse dynamics, and one-sweep CRBA mass matrix |
| `armbycontroller/ik/screw.py` | 完整 SE(3) 姿态 IK 与导纳用受限旋量 Jacobian 速度 IK，共享 PoE 模型 / Full-SE(3) pose IK and bounded screw-Jacobian velocity IK for admittance over one PoE model |
| `armbycontroller/ik/core.py` | IK 创建、目标增量和控制器共享工具；唯一工厂是 `create_screw_solver` / IK construction, target increments, and shared controller helpers; `create_screw_solver` is the sole factory |
| `armbycontroller/ros/keyboard_controller_node.py` | ROS 键盘状态机、I/O/H 互锁、SDK/CAN adapter、100 Hz 实测状态发布 / ROS keyboard state machine, I/O/H interlock, SDK/CAN adapter, and 100 Hz measured-state publication |
| `armbycontroller/modeling/momentum_observer.py` | 无 ROS/CAN 的纯广义动量观测器 / ROS/CAN-free generalized-momentum observer |
| `armbycontroller/ros/momentum_observer_node.py` | 只订阅 `/arm_dynamics_state` 的独立 ROS adapter；发布外力矩但不控制机械臂 / Separate ROS adapter that only subscribes to `/arm_dynamics_state`; publishes external torque without controlling the arm |
| `armbycontroller/ros/experiment_recorder_node.py` | 可选独立记录进程；订阅 sample/event、提供 recording service、不访问 CAN / Optional recorder process; subscribes to samples/events, exposes recording service, and never accesses CAN |
| `config/common.yaml` | 两种机械臂共用的周期、默认 backend 和固件探测时序 / Rates, default backend, and firmware-probe timing shared by both arms |
| `config/nero.yaml` | Nero 独立固件、裸臂、侧装、速度估计、7 轴零空间、J2/J3/J4 混合姿态、两种导纳和观测器参数 / Nero-only firmware, bare-arm side mount, velocity estimation, seven-axis nullspace, J2/J3/J4 hybrid posture, both admittance modes, and observer parameters |
| `config/piper_l.yaml` | Piper-L 独立固件、夹爪、6 轴增益/比例/限制、两种导纳和观测器参数 / Piper-L-only firmware, gripper, six-axis tuning and limits, both admittance modes, and observer parameters |
| `test/test_cartesian_common.py` | 共享 Jacobian、SE(3) 和虚功映射契约 / Shared Jacobian, SE(3), and virtual-work contracts |
| `test/test_cartesian_impedance.py` | 阻抗符号、耦合、零空间与模型支撑契约 / Impedance sign, coupling, nullspace, and model-support contracts |
| `test/test_arm_control.py` | 现有关节控制、IK、动力学和硬件 adapter 回归 / Existing joint control, IK, dynamics, and hardware-adapter regression |
| `test/test_momentum_observer.py` | 空间动量、动能梯度、残差收敛、离散稳定性及禁止 SDK/CAN 访问 / Spatial momentum, kinetic gradient, residual convergence, discrete stability, and forbidden SDK/CAN-access contracts |
| `test/test_cartesian_admittance.py` | 抗漂移柔顺零力、阻力平衡、旋转向量、边界和重置契约 / Anti-drift soft-zero-force, resistive-equilibrium, rotation-vector, bound, and reset contracts |
| `test/test_control_interface.py` | 四种交互模式的共用 interface、命令、sample schema 和互锁契约 / Shared interface, command, sample-schema, and interlock contracts for four interaction modes |
| `test/test_hybrid_control.py` | 任务顺序、互补选择、导纳目标力投影和阻抗/导纳输出解耦契约 / Task ordering, complementary selection, desired-wrench projection, and impedance/admittance output-decoupling contracts |
| `test/test_experiment.py` | manifest、JSONL、事件顺序和汇总指标契约 / Manifest, JSONL, event-order, and summary-metric contracts |
| `test/test_hardware_connection.py` | 两次连接顺序、探测数据保存、版本到 profile 映射和失败断开契约 / Two-connection ordering, saved probe data, version-to-profile mapping, and failure-disconnect contracts |

用户要求排除 `ros/`、`ik/`、`impedance/` 后的详细文件分类，见
`docs/file_map/README_ZH_EN.md` 及其五个分类子文档。 / For the requested
detailed classification outside `ros/`, `ik/`, and `impedance/`, see
`docs/file_map/README_ZH_EN.md` and its five category documents.

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
| `firmware` | string | — | `auto`；实机检测结果优先，显式值仅作配置检查与干跑 profile / detected hardware wins; explicit value is a configuration check and dry-run profile |
| `firmware_probe_timeout` | scalar | s | `5.0`；第一阶段取得固件数据的总时限 / total stage-one firmware-data deadline |
| `firmware_probe_poll_period` | scalar | s | `0.1`；无数据时的查询间隔 / retry interval while data is absent |
| `firmware_reconnect_delay` | scalar | s | `0.5`；探测连接断开与正式连接创建之间的等待 / delay between probe disconnect and formal-connection creation |
| `impedance_backend` | string | — | `cartesian` (`joint` 可对照 / for comparison) |
| `interaction_torque_limit` | 1 or n | N·m | `8.0` per joint；关节阻抗、笛卡尔阻抗、导纳和混合共用的估算总力矩上限 / estimated-total-torque limit shared by joint/Cartesian impedance, admittance, and hybrid |
| `interaction_reference_joint_velocity_limit` | 1 or n | rad/s | `1.0` per joint；阻抗轨迹和导纳/混合旋量速度 IK 共用 / shared by impedance trajectories and admittance/hybrid screw-velocity IK |
| `interaction_measured_joint_velocity_stop_limit` | 1 or n | rad/s | Nero=`2.0`; Piper-L=`1.5` per joint；持续速度停止线 / sustained measured-speed stop threshold |
| `interaction_measured_joint_velocity_hard_limit` | 1 or n | rad/s | `2.5` per joint；任一 `I/O/H` 模式单周期立即急停 / immediate one-cycle stop in every I/O/H mode |
| `interaction_measured_velocity_violation_cycles` | scalar | cycles | `3`；持续速度超限去抖 / sustained-speed debounce |
| `interaction_joint_limit_margin` | scalar | rad | `0.03`；所有 MIT 周期的实测位置容差，也是导纳/混合预测恢复带 / measured-position tolerance for all MIT cycles and predictive recovery band for admittance/hybrid |
| `cartesian_impedance_rotation_stiffness` | scalar | N·m/rad | Nero=`1.9`；Piper-L=`0.4`，基座 X/Y 旋转 / base-frame X/Y rotation |
| `cartesian_impedance_base_z_rotation_stiffness` | scalar | N·m/rad | Nero=`1.9`（无补强 / isotropic）；Piper-L=`4.0`（基座 Z 补强 / reinforced） |
| `cartesian_impedance_translation_stiffness` | scalar | N/m | Nero=`70.0`; Piper-L=`10.0` |
| `cartesian_impedance_rotation_damping` | scalar | N·m·s/rad | Nero=`0.24`; Piper-L=`0.08` |
| `cartesian_impedance_translation_damping` | scalar | N·s/m | Nero=`1.4`; Piper-L=`0.8` |
| `cartesian_impedance_nullspace_stiffness` | 1 or n | N·m/rad | `0.4`，仅 Nero / Nero only |
| `cartesian_impedance_nullspace_damping` | 1 or n | N·m·s/rad | `0.1`，仅 Nero / Nero only |
| `cartesian_impedance_joint_posture_stiffness` | 1 or n | N·m/rad | Nero=`[0,0.5,0.5,0.6,0,0,0]`; Piper-L=`0` |
| `cartesian_impedance_joint_posture_damping` | 1 or n | N·m·s/rad | Nero=`[0,0.08,0.08,0.12,0,0,0]`; Piper-L=`0` |
| `cartesian_impedance_model_scale` | 1 or n | — | `1.0`，范围 `[0,1]`，逐关节缩放 `Cqdot+g` / per-joint scale for `Cqdot+g` |
| `nero_mount` | string | — | YAML 为 `side`；也可用 `horizontal` / YAML uses `side`; `horizontal` is valid |
| `tool_configuration` | string | — | Nero 文件=`none` 裸臂 / bare arm；Piper-L 文件=`gripper` |
| `nero_velocity_estimation_enabled` | bool | — | `true`，仅 v111/v112 / v111/v112 only |
| `velocity_filter_time_constant` | scalar | s | `0.03`，一阶低通差分 / first-order filtered finite difference |
| `mit_command_rate` | scalar | Hz | `100.0` |
| `dynamics_state_topic` | string | — | `/arm_dynamics_state`; `effort=tau_motor` |
| `momentum_observer_enabled` | bool | — | `true`，只启动/停止独立观测进程 / only starts/stops the separate observer process |
| `momentum_observer_rate` | scalar | Hz | `100.0`，预期输入频率；计算由每条输入消息触发 / expected input rate; every input message triggers an update |
| `momentum_observer_gain` | 1 or n | 1/s | `10.0` |
| `momentum_observer_max_period` | scalar | s | `0.05`；较大数据间隙时重置 / reset after a larger stream gap |
| `external_torque_topic` | string | — | `/arm_external_joint_torque`；`effort` 为 N·m / `effort` is N·m |
| `admittance_mode` | string | — | Nero=`zero_force`; Piper-L=`resistive` |
| `admittance_virtual_mass` | 6 | N·m·s²/rad, kg | Nero=`[0.2,0.2,0.2,2,2,2]`; Piper-L=`[0.12,0.12,0.12,1.5,1.5,1.5]` |
| `admittance_zero_force_damping` | 6 | N·m·s/rad, N·s/m | Nero=`[2,2,2,18,18,18]`; Piper-L=`[0.8,0.8,0.8,8,8,8]` |
| `admittance_zero_force_holding_stiffness` | 6 | N·m/rad, N/m | Nero=`[0.25,0.25,0.25,5,5,5]`; Piper-L=`[0.1,0.1,0.1,2,2,2]` |
| `admittance_zero_force_friction` | 6 | N·m, N | Nero=`[0.03,0.03,0.03,0.35,0.35,0.35]`; Piper-L=`[0.02,0.02,0.02,0.25,0.25,0.25]` |
| `admittance_zero_force_stiction_velocity` | 6 | rad/s, m/s | Both=`[0.015,0.015,0.015,0.005,0.005,0.005]` |
| `admittance_resistive_damping` | 6 | N·m·s/rad, N·s/m | Nero=`[2.5,2.5,2.5,25,25,25]`; Piper-L=`[0.8,0.8,0.8,8,8,8]` |
| `admittance_resistive_stiffness` | 6 | N·m/rad, N/m | Nero=`[0.8,0.8,0.8,12,12,12]`; Piper-L=`[0.8,0.8,0.8,8,8,8]` |
| `admittance_wrench_deadband` | 6 | N·m, N | Nero=`[0.05,0.05,0.05,0.25,0.25,0.25]`; Piper-L=`[0.03,0.03,0.03,0.15,0.15,0.15]` |
| `admittance_wrench_limit` | 6 | N·m, N | Nero=`[1.5,1.5,1.5,6,6,6]`; Piper-L=`[2,2,2,8,8,8]` |
| `admittance_offset_limit` | 6 | rad, m | Nero=`[0.25,0.25,0.25,0.08,0.08,0.08]`; Piper-L=`[0.35,0.35,0.35,0.10,0.10,0.10]` |
| `admittance_velocity_limit` | 6 | rad/s, m/s | Nero=`[0.12,0.12,0.12,0.05,0.05,0.05]`; Piper-L=`[0.5,0.5,0.5,0.15,0.15,0.15]` |
| `admittance_wrench_filter_hz` | scalar | Hz | `5.0` |
| `admittance_wrench_dls_damping` | scalar | — | `0.05` |
| `admittance_wrench_timeout` | scalar | s | `0.10`; stale wrench becomes zero |
| `admittance_mit_kp` | n | N·m/rad | Nero=`[0.32,0.24,0.32,0.24,0.32,0.28,0.28]`; Piper-L=`[0.3,0.5,0.5,0.5,1.0,0.3]` |
| `admittance_mit_kd` | n | N·m·s/rad | Nero=`0.08`; Piper-L=`0.01` per joint |
| `admittance_mit_model_scale` | scalar | — | `1.0`, range `[0,1]`; gravity feedforward scale |
| `admittance_task_weights` | 6 | — | `[0.4,0.4,0.4,1,1,1]` in `[angular;linear]` order |
| `admittance_velocity_dls_damping` | scalar | — | `0.02` |
| `hybrid_admittance_axes` | string | — | `z`; base-frame names from `rx,ry,rz,x,y,z`, comma/space separated |
| `hybrid_desired_wrench` | 6 | N·m, N | `[0,0,0,0,0,0]` in `[Mx,My,Mz,Fx,Fy,Fz]` order; only selected admittance axes apply |
| `desired_twist` | `6` | rad/s, m/s | 由连续参考的 `Jg(q_ref) qdot_ref` 生成 / generated as `Jg(q_ref) qdot_ref` |
| `gravity` | `3` | m/s² | 由已有 URDF model 配置 / Existing URDF-model configuration |
| `control_sample_topic` | string | — | `/arm_control_sample`; schema-v1 JSON `std_msgs/String` |
| `control_event_topic` | string | — | `/arm_control_event`; JSON `std_msgs/String` |
| `experiment_recording_enabled` | bool | — | `false`; 是否启动独立 recorder / whether to launch the separate recorder |
| `experiment_output_directory` | path | — | `~/.ros/armbycontroller/experiments` |
| `experiment_name` | string | — | `manual_control` |
| `experiment_flush_every` | scalar | samples/events | `1`; 每条刷新，安全优先 / flush every record, prioritizing recoverability |

启用 URDF 重力/逆动力学补偿时，补偿项严格为经过 `mit_gravity_scale` 和绝对
力矩限幅的模型输出，不再叠加 `mit_feedforward` 或任何固定标定 bias。
`mit_feedforward` 只在没有 URDF 补偿的关节对照 backend 中作为显式手动力矩。
/ With URDF gravity/inverse-dynamics compensation enabled, the compensation
term is strictly the scaled and absolute-bounded model output; neither
`mit_feedforward` nor any fixed calibration bias is added. `mit_feedforward`
remains an explicit manual torque only for the joint comparison backend when
URDF compensation is absent.

`mit_kp/mit_kd` 只属于 `joint` 对照 backend。Cartesian 阻抗 backend 无条件向
固件发送 `kp=kd=0`，防止关节 PD 与 `J^T F` 重复计算。导纳与混合独立使用低增益
`admittance_mit_kp/kd` 跟踪每周期重锚定的短期参考。

`mit_kp/mit_kd` belong only to the `joint` comparison backend. Cartesian
impedance always sends `kp=kd=0`, preventing native joint PD from
double-counting `J^T F`. Admittance and hybrid control independently use low
`admittance_mit_kp/kd` to track its one-cycle, measured-state-reanchored
reference.

上述控制整定值先读取 `config/common.yaml`，再按 `robot_model` 读取
`config/nero.yaml` 或 `config/piper_l.yaml`。固件探测时序来自 common；同名
launch 参数显式传值时优先于两层 YAML。`can_interface`、`execute_motion`、话题
和进程开关仍由 launch 管理。
/ These values load from `config/common.yaml` and then the `nero.yaml` or
`piper_l.yaml` selected by `robot_model`. Firmware-probe timing comes from
common. Explicit same-name launch values take precedence over both YAML
layers. `can_interface`, `execute_motion`, topics, and process switches remain
launch-managed.

旋转刚度向量按基座坐标系组成
`[K_rx, K_ry, K_rz]=[K_rotation, K_rotation, K_base_z]`。Piper-L 独立提高
`K_base_z`，以增强基座 Z 旋转回正，而不同时提高腕部 X/Y 旋转刚度。
Nero 不使用这个 Piper-L 专用补强，配置为 `[1.9, 1.9, 1.9]`。这些都是
任务空间刚度，不是直接关节 `Kp`；实际等效关节刚度仍由
`Kq=Jg^T Kx Jg` 和当前姿态决定。

The base-frame rotational stiffness vector is
`[K_rx, K_ry, K_rz]=[K_rotation, K_rotation, K_base_z]`. Piper-L raises
`K_base_z` independently to strengthen base-Z return without also raising
wrist X/Y rotational stiffness. Nero does not use this Piper-L-specific
reinforcement and is configured as `[1.9, 1.9, 1.9]`. These remain task-space
stiffnesses rather than direct joint `Kp` values; equivalent joint stiffness
still depends on the current pose through `Kq=Jg^T Kx Jg`. Nero's separately
listed J2/J3/J4 posture gains are the deliberate exception and act directly
in joint space before the total torque clip.

## 10. 安全边界 / Safety boundaries

- Nero 和 Piper-L 固件探测都在 `get_firmware()` 前发送一次临时 `enable()`，并
  在断开前发送 `disable()`；它们不切换模式、不运动、不写固件。正式控制实例只
  在探测实例断开并等待 `firmware_reconnect_delay` 后创建。 / Both Nero and
  Piper-L firmware probes send one temporary `enable()` before `get_firmware()`
  and send `disable()` before disconnecting; they do not change modes, move, or
  write firmware. The formal control instance is created only after the probe
  disconnects and `firmware_reconnect_delay` elapses.
- 固件数据缺失、`software_version` 无法解析或检测 profile 不受当前 SDK 支持时，
  实机启动直接失败。 / Hardware startup fails closed when firmware data is
  absent, `software_version` cannot be parsed, or the detected profile is not
  supported by the installed SDK.
- 关节阻抗、笛卡尔阻抗、导纳和混合都从同一个
  `interaction_torque_limit` 读取逐关节估算总力矩上限，默认 ±8 N·m，且包含
  feedback、模型、任务和辅助项；任何模式都不能配置超过 `8 N·m`。这些路径
  都不实施力矩变化率限制。 / Joint impedance, Cartesian impedance,
  admittance, and hybrid control all read their per-joint estimated-total-
  torque envelope from `interaction_torque_limit`, default ±8 N·m including
  feedback, model, task, and auxiliary terms. No mode accepts a value above
  `8 N·m`, and none implements a torque-rate limiter.
- `Kx/Dx` 必须对称半正定，以避免明显的主动负刚度/负阻尼配置。 / `Kx/Dx`
  must be symmetric positive semidefinite, rejecting obvious active negative
  stiffness/damping.
- Nero 零空间增益必须非负，且零空间力矩也计入同一个 ±8 N·m 总力矩上限。 /
  Nero nullspace gains must be nonnegative, and nullspace torque shares the
  same ±8 N·m total-torque envelope.
- 关节选择性姿态增益必须非负；该项未经零空间投影，会改变末端任务，但仍与任务、
  零空间和模型力矩一起进入同一个 ±8 N·m 总限幅。 / Joint-selective posture
  gains must be nonnegative. This unprojected term changes the tool task but
  shares the same ±8 N·m total clip with task, nullspace, and model torque.
- `cartesian_impedance_model_scale` 只能减小模型项，不能超过 `1.0`；它不是自动
  标定。修改 J4 前必须支撑机械臂并在多个静态姿态比较实测保持力矩。 /
  `cartesian_impedance_model_scale` can only reduce model support and cannot
  exceed `1.0`; it is not automatic calibration. Support the arm and compare
  measured holding torque at several static poses before changing J4.
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
- `I/O/H` 是互锁切换，不是三个 controller 叠加：纯导纳、纯阻抗和混合各自运行
  一份 MIT 输出；
  跨模式必须先成功回到普通 planned-position 并保持当前位置，再进入目标模式；
  同时请求其中两个或三个模式会保持原模式。 / `I/O/H` are interlocked
  switches, not three stacked controllers: pure admittance, pure impedance,
  and hybrid control each own one MIT output. Every cross-mode transition must
  restore normal planned-position control and hold the current position before
  entering its target; requesting two or three modes together leaves the
  current mode unchanged.
- 所有 `I/O/H` 模式的参考关节速度统一受
  `interaction_reference_joint_velocity_limit=1.0 rad/s` 限制，并在 ROS 硬件
  adapter 先经过同一个实测速度保护器。Nero 任一关节超过 `2.0 rad/s` 连续三个
  周期、Piper-L 超过 `1.5 rad/s` 连续三个周期，或任一机械臂单周期超过
  `2.5 rad/s`，都会触发电子急停。导纳与混合还保留各自的 wrench、虚拟
  Twist/位移和预测位置限制。 / Every `I/O/H` mode shares
  `interaction_reference_joint_velocity_limit=1.0 rad/s` and first passes the
  same measured-speed guard in the ROS hardware adapter. An electronic stop is
  triggered after three consecutive cycles above `2.0 rad/s` on Nero or
  `1.5 rad/s` on Piper-L, or immediately above `2.5 rad/s` on either arm.
  Admittance and hybrid control additionally retain their wrench, virtual
  Twist/offset, and predictive-position bounds.
- 导纳与混合对 URDF 边界外最多 `interaction_joint_limit_margin=0.03 rad` 只开放受控
  恢复：旋量速度 IK 禁止继续向外，只允许静止或向内运动；超过该恢复带仍由
  Control Cycle Guard 拒绝。普通 planned-position 使用独立的 SDK 限位与
  `move_j` 路径，不依赖该恢复规则。 / Admittance and hybrid control permit only
  a controlled recovery within `interaction_joint_limit_margin=0.03 rad`
  outside a URDF limit: screw velocity IK blocks outward motion and allows only
  rest or inward motion, while the Control Cycle Guard rejects larger
  violations. Normal
  planned-position control has its own SDK limits and `move_j` path and does
  not depend on this admittance recovery rule.
- Nero 的 `zero_force` 是柔顺零力，不是数学零抵抗：弱 `Kh`、正 `D0`、输入
  deadband 与有限 `Fstick/slip` 共同阻止静态残差积成漂移。只有超过阈值的接触
  才开始移动。 / Nero's `zero_force` is soft zero force, not mathematically
  zero resistance: weak `Kh`, positive `D0`, input deadband, and bounded
  `Fstick/slip` prevent static residual from integrating into drift. Motion
  begins only after contact exceeds the configured threshold.
- 离散实现要求 `momentum_observer_gain * momentum_observer_max_period < 2`；
  不稳定组合在启动时拒绝。 / The discrete implementation requires
  `momentum_observer_gain * momentum_observer_max_period < 2`; unstable
  combinations are rejected at startup.
- controller adapter 禁止直接访问 ROS、磁盘或 CAN；所有输出仍经过 ROS/AGX
  adapter 的模式与硬件检查。 / Controller adapters must not access ROS, disk,
  or CAN directly; every output still crosses the ROS/AGX adapter's mode and
  hardware checks.
- experiment recorder 默认不启动、从不连接 CAN；记录失败不能生成机械臂命令。
  JSONL 每条记录独立成行，异常退出时已刷新行仍可恢复。 / The experiment
  recorder is disabled by default and never connects to CAN; recording failure
  cannot produce arm commands. Each JSONL record is independent, so flushed
  lines survive an abnormal exit.

## 11. 已知风险 / Known risks

- `firmware_probe_timeout` 过短或 CAN 负载过高会导致安全的启动超时；增加时限只
  延长探测等待，不改变正式驱动 profile。 / A short
  `firmware_probe_timeout` or heavy CAN load can cause a safe startup timeout;
  increasing it only extends discovery and does not alter the selected formal
  driver profile.
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
- Nero J2/J3/J4 姿态力矩未经零空间投影，因此增大其增益会直接改变末端顺应性并
  可能与 `J_g^T F_c` 竞争；它不能被解释为不影响任务的冗余姿态控制。 / Nero's
  J2/J3/J4 posture torque bypasses the nullspace projector, so larger gains
  directly change tool compliance and can compete with `J_g^T F_c`; it is not
  task-invariant redundant-posture control.
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
- Nero v111/v112 的 SDK 速度字段固定为零；控制器现在以位置有限差分和 `0.03 s`
  一阶低通估算速度。它仍受编码器量化、缓存不同步和 Python 调度抖动影响。 /
  The Nero v111/v112 SDK velocity field is fixed at zero; the controller now
  estimates velocity from position differences with a `0.03 s` first-order
  low-pass filter. Encoder quantization, cache skew, and Python scheduling
  jitter still affect it.
- SDK 在 `connect()` 后由后台线程持续接收 CAN；控制器读取的是解析器当前缓存。
  `get_joint_angles()` 与逐关节 `get_motor_states()` 是不同反馈消息，因而同一循环
  内也不是硬件原子快照。观测器不会再次调用它们。 / After `connect()`, the SDK
  receives CAN in a background thread and the controller reads its current
  parser cache. `get_joint_angles()` and per-joint `get_motor_states()` are
  different feedback messages, so one loop still does not form a
  hardware-atomic snapshot. The observer never calls them again.
- 100 Hz Python/ROS 循环不是硬实时。CRBA 与参考运动学缓存把 1218 帧 Nero
  实机 trace 的离线纯控制计算从约 `9.72 ms` 降到约 `2.4 ms/周期`，但实际周期
  仍包含 SDK/CAN 读写和 ROS 调度，必须由新实机记录验证。 / A 100 Hz
  Python/ROS loop is not hard real time. CRBA and reference-kinematics caching
  reduced offline pure-control replay of the 1,218-sample Nero hardware trace
  from about `9.72 ms` to about `2.4 ms/cycle`; actual periods still include
  SDK/CAN I/O and ROS scheduling and require a fresh hardware recording.
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
- Nero 文件的 `tool_configuration=none` 读取裸臂
  `nero_description.urdf`，不包含手或夹爪质量；也可在物理安装确实变化后显式
  选择 `gripper`、`left_revo2` 或 `right_revo2`。错误的工具选择会直接形成错误
  重力力矩，任务点仍为 `link7`。 / Nero `tool_configuration=none` loads the
  bare `nero_description.urdf`, with no hand or gripper mass. Select `gripper`,
  `left_revo2`, or `right_revo2` only after the physical installation changes.
  A wrong tool directly creates wrong gravity torque; the task point remains
  `link7`.
- 任务空间阻抗和 IK 目标生成必须解耦，慢 IK 不能阻塞力矩刷新。 / Task
  impedance and IK target generation must be decoupled so slow IK cannot block
  torque refresh.
- `/arm_control_sample` 在控制进程内进行 JSON 序列化；磁盘写入在独立进程，但
  Python 序列化和 ROS publish 仍会增加非硬实时循环的负载。 / Control samples
  are JSON-serialized in the control process. Disk writes are separate, but
  Python serialization and ROS publication still add load to the non-real-time
  loop.
- `summary.json` 的关节位置 RMSE 使用每周期 `reference.position-state.position`，
  是控制参考跟踪指标，不等于 Cartesian 误差、接触质量或稳定性证明。 / The
  joint-position RMSE in `summary.json` uses
  `reference.position-state.position`; it is a reference-tracking metric, not
  Cartesian error, contact quality, or proof of stability.
- JSON topic 使用 schema version 保护格式演进，但当前没有 custom ROS message
  的编译期字段检查；新版本 consumer 必须显式处理 `schema_version`。 / JSON
  topics carry a schema version but lack compile-time field checks from custom
  ROS messages; future consumers must handle `schema_version` explicitly.

## 12. 构建、测试和运行 / Build, test, and run

### 构建 / Build

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
python3 -m pip install "modern_robotics>=1.1.1"
colcon build --packages-select armbycontroller --symlink-install
source install/setup.bash
```

### 配置文件 / Configuration file

launch 先加载 `config/common.yaml`，再根据 `robot_model` 只加载
`config/nero.yaml` 或 `config/piper_l.yaml`。common 只含共用周期、默认 backend
和固件探测时序；固件配置检查、工具、重力/模型比例、逐关节增益与限制、轨迹和
观测器参数和两种导纳的整套参数全部按机器人分开；Nero 速度估计、零空间和
J2/J3/J4 混合姿态增益仍只在 Nero 文件。显式 launch 参数仍有最高优先级。
`common_config:=/path/common.yaml`
可替换共用层，
`controller_config:=/path/robot.yaml` 可替换机器人层。 / Launch first loads
`config/common.yaml`, then only `config/nero.yaml` or `config/piper_l.yaml`
according to `robot_model`. Common holds only shared rates/default backend and
firmware-probe timing; firmware configuration checks, tool, gravity/model
scales, joint gains/limits, trajectories, observer tuning, and both admittance
modes are robot-specific. Velocity estimation, nullspace tuning, and
J2/J3/J4 hybrid-posture gains exist only in Nero.
Explicit launch arguments have highest priority. `common_config` and
`controller_config` can replace the respective layers.

Nero 文件对应当前侧装、无手机械臂，显式设置 `tool_configuration=none`；
Piper-L 文件独立设置 `tool_configuration=gripper`。实机启动时，两种机械臂都会先
用 `default` profile 探测并保存数据、断开，再以检测 profile 正式重连；例如 Nero
`1.11 -> v111`、Piper-L `S-V1.8-8 -> v188`。`firmware=auto` 不再使用静态版本
猜测。 / Nero explicitly uses `tool_configuration=none`; Piper-L independently
uses `tool_configuration=gripper`. On hardware, both arms first probe with the
`default` profile, save the returned data, disconnect, and formally reconnect
with the detected profile; examples are Nero `1.11 -> v111` and Piper-L
`S-V1.8-8 -> v188`. `firmware=auto` no longer guesses a static version.

`cartesian_impedance_model_scale` 对 Nero 可写 7 个值，顺序为 J1…J7，J4 是第
4 个。当前保持 `[1.0]`。旧日志中的侧装 Revo2 模型给 J4 约 `3.55 N·m`；改为
裸臂模型后，同一姿态的预测约为 `1.98 N·m`，去除了约 `1.57 N·m` 的错误末端
负载贡献。剩余比例仍需机械支撑下的多姿态静态力矩标定。 / For Nero,
`cartesian_impedance_model_scale` may contain seven values in J1…J7 order; J4
is the fourth and remains `[1.0]`. The old side-mounted Revo2 model predicted
about `3.55 N·m` at J4; the bare model predicts about `1.98 N·m` at the same
pose, removing about `1.57 N·m` of incorrect tool contribution. Calibrate any
remaining scale at multiple supported static poses.

### 公式测试 / Formula tests

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_cartesian_impedance.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_momentum_observer.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_cartesian_admittance.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_control_interface.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest -q test/test_experiment.py
```

### 全部测试 / Full tests

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select armbycontroller
colcon test-result --verbose
```

### 实验记录 / Experiment recording

默认不启动 recorder。显式启用会为每次 launch 创建新的 run directory；不会覆盖
已有实验。 / The recorder is not launched by default. Explicit enablement
creates a fresh run directory for every launch and never overwrites an existing
experiment.

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l \
  experiment_recording_enabled:=true \
  experiment_name:=joint_vs_cartesian \
  experiment_output_directory:=~/.ros/armbycontroller/experiments
```

输出 / Outputs:

```text
<run-id>/manifest.json
<run-id>/samples.jsonl
<run-id>/events.jsonl
<run-id>/summary.json
```

单独启动 recorder 时，通过 `/arm_experiment_recorder/recording` 的
`std_srvs/srv/SetBool` 开始/结束。`false` 收尾时才写 `summary.json`；急停事件会
写入 `events.jsonl`，但不会由 recorder 自己触发急停。 / When the recorder is
started independently, use `/arm_experiment_recorder/recording`
(`std_srvs/srv/SetBool`) to start/stop. `summary.json` is written when `false`
closes the run. Emergency-stop events are recorded in `events.jsonl`; the
recorder never initiates an emergency stop.

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
  can_interface:=can0 execute_motion:=true \
  nero_mount:=horizontal \
  move_home_on_start:=false reset_emergency_stop_on_start:=true
```

使用 `scripts/start_nero.sh` 或 `scripts/start_piper_l.sh` 选择本地键盘时，设备
输入可写为编号 `3`、事件名 `event3` 或完整路径 `/dev/input/event3`；启动脚本
会统一解析为完整设备路径。命令行简写 `device:=3` 也采用同一规则。 / When
selecting a local keyboard through `scripts/start_nero.sh` or
`scripts/start_piper_l.sh`, the device may be entered as event number `3`, event
name `event3`, or full path `/dev/input/event3`; the startup script normalizes
all forms to the full device path. The `device:=3` command-line shorthand uses
the same rule.

YAML 默认保持 `nero_mount=side`。上面的指令只为本次平置实机启动显式传
`nero_mount:=horizontal`，不修改默认。启动后保持机械臂被支撑，按 `I` 同时
捕获当前末端位姿、7 轴零空间姿态和 J2/J3/J4 姿态参考，再进入 Cartesian MIT。
Nero 的旋转刚度为 `[1.9,1.9,1.9]`，不使用 Piper-L 的基座 Z 补强；J2/J3/J4
另有未经零空间投影的 `[0.5,0.5,0.6] N·m/rad` 外环姿态弹簧。 / YAML defaults
to `nero_mount=side`. The command above passes `nero_mount:=horizontal` only
for this horizontal-base run and does not change the default. With the arm
physically supported,
press `I` to capture tool pose, nullspace posture, and the J2/J3/J4 posture
reference before Cartesian MIT. Nero uses `[1.9,1.9,1.9]` rotational stiffness
without Piper-L's base-Z reinforcement and adds unprojected outer-loop
`[0.5,0.5,0.6] N·m/rad` posture springs on J2/J3/J4.

Nero 默认 `admittance_mode=zero_force`。保持机械臂受支撑，确认
`/arm_external_joint_torque` 新鲜后按 `O` 捕获锚定位姿并进入抗漂移柔顺零力；
默认平移 deadband 与虚拟摩擦分别为 `0.25 N`、`0.35 N`，且有 `5 N/m` 弱保持，
因此约小于 `0.6 N` 的静态模型残差不会开始移动；
下游从同一 PoE 模型取得旋量 Jacobian、构造工具几何 Jacobian，再由受限加权
DLS 生成 `dq_ref`，并用验证包同值低增益 MIT 跟踪；共享 Model Compensation
提供重力项，每周期参考从实测 `q` 重锚定，估算总力矩限制为 ±8 N·m；参考关节速度
限制为 `1.0 rad/s`，实测任一关节超过 `2.0 rad/s` 连续三个周期或单周期超过
`2.5 rad/s` 会触发电子急停。
若要验证带阻力且松手回中的版本，在启动命令末尾追加
`admittance_mode:=resistive`。 / Nero defaults to
`admittance_mode=zero_force`. With the arm supported and a fresh
`/arm_external_joint_torque`, press `O` to capture the anchor and enter
anti-drift soft zero force. The default translational deadband and virtual
friction are `0.25 N` and `0.35 N`, with weak `5 N/m` holding; a static model
residual below roughly `0.6 N` therefore does not initiate motion. Downstream,
the bounded screw-Jacobian velocity IK obtains the screw Jacobian from the
same PoE model, constructs the tool geometric Jacobian, and uses weighted DLS
to produce `dq_ref`. It is tracked with the verified package's low MIT gains;
shared Model Compensation supplies gravity, the reference is reanchored to
measured `q` every cycle, and estimated total torque is limited to ±8 N·m.
Reference joint speed is limited to `1.0 rad/s`; measured speed above
`2.0 rad/s` for three consecutive cycles or above `2.5 rad/s` for one cycle
triggers the electronic stop. Append
`admittance_mode:=resistive` to test the strongly returning variant.

混合实机试验使用同一启动命令，默认无需修改 YAML：确认观测 wrench 新鲜、支撑
机械臂后按 `H`，当前末端位姿成为阻抗锚点，基座 Z 方向走导纳，其余五维走阻抗；
再次按 `H` 先退出到普通保持。可在启动命令显式追加
`hybrid_admittance_axes:=x,y` 选择多个固定基座方向。`I`、`O`、`H` 任意互切都
先经过普通模式，混合期间 `P`、home 和手动 jog 锁定。 / Use the same launch
command for a hybrid hardware trial without changing YAML. After confirming a
fresh observer wrench and physically supporting the arm, press `H`: the
current tool pose becomes the impedance anchor, base-frame Z uses admittance,
and the other five dimensions use impedance. Press `H` again to exit through
normal hold. Append `hybrid_admittance_axes:=x,y` to select multiple fixed
base-frame directions. Every switch among `I`, `O`, and `H` passes through
normal mode; `P`, home, and manual jog are locked while hybrid is active.

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

Piper-L 默认使用带阻尼和回中刚度的 `resistive` 导纳；追加
`admittance_mode:=zero_force` 可选择零力版本。保持 `impedance_enabled:=false`，
启动后按 `O` 捕获当前位姿并进入导纳；再次按 `O` 退出。若阻抗已开启，`O` 会先
退出阻抗；若导纳已开启，`I` 会先退出导纳。导纳中 `P`、手动 jog 和 home 被
锁住。 / Piper-L defaults to damped and restoring `resistive` admittance;
append `admittance_mode:=zero_force` to select the zero-force variant. Keep
`impedance_enabled:=false`, then press `O` to capture the current pose and
enter admittance; press `O` again to leave. Press `H` for the default
base-frame-Z admittance plus five-axis impedance hybrid. Every switch among
`I`, `O`, and `H` exits the current controller to normal hold before entering
the target. `P`, manual jog, and home are locked while admittance or hybrid is
active.

实机按 `O` 或 `H` 前必须已有新鲜的 `/arm_external_joint_torque`（默认不超过
`0.10 s`）；否则控制器拒绝进入并提示检查该 topic。 / Before pressing `O` or
`H` on hardware, `/arm_external_joint_torque` must be fresh (no older than
`0.10 s` by default); otherwise entry is rejected with a topic diagnostic.

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

实机启动只有出现 `<robot> ready` 才表示 `arm_ready=true`；若 CAN、使能、模式或
反馈初始化失败，节点明确打印 `motion unavailable`，此时键盘运动命令被禁用。 /
On hardware, only a `<robot> ready` line means `arm_ready=true`. A CAN,
enable, mode, or feedback initialization failure prints `motion unavailable`
and disables keyboard motion commands.

`reset_emergency_stop_on_start:=true` 表示每次正式连接后都显式发送一次电子急停
复位，再执行电机使能和模式确认；不能依赖连接瞬间尚未刷新的缓存状态来决定是否
发送 reset。 / `reset_emergency_stop_on_start:=true` explicitly sends an
electronic-stop reset after every formal connection, before motor enable and
mode confirmation. The decision does not rely on possibly stale cached status
at connection time.

1. `T` 是否为合法 SE(3)。 / Is `T` valid SE(3)?
2. `J_s` 是否为 `6×n` 且顺序为 `[角; 线]`。 / Is `J_s` `6×n` and ordered
   `[angular; linear]`?
3. `J_g` 的线速度是否与 FK 有限差分一致。 / Does `J_g` linear velocity
   match an FK finite difference?
4. `e_R/e_p` 的正负方向是否使 wrench 指向目标。 / Do `e_R/e_p` signs make
   the wrench point toward the target?
5. IK 是否使用 `Log(T_d T^-1)^vee` 与同一基坐标系的 `J_s`。 / Does IK pair
   `Log(T_d T^-1)^vee` with `J_s` in the same base frame?
6. 若实测关节位于 URDF 限位外但仍在 `0.03 rad` 恢复带内，旋量速度 IK 是否对
   外向命令输出零、只允许向内速度；更大越界是否被周期保护器拒绝。 / When a
   measured joint is outside a URDF limit but within the `0.03 rad` recovery
   band, does screw velocity IK block outward commands and permit only inward
   velocity, while the cycle guard rejects larger violations?
7. 是否满足 `tau^T qdot = F^T xdot`。 / Does
   `tau^T qdot = F^T xdot` hold?
8. `ID(q,qdot,0)` 是否只产生科氏/离心和重力支撑。 / Does
   `ID(q,qdot,0)` contain only Coriolis/centrifugal and gravity support?
9. Nero 是否满足 `J_g M^-1 tau_null≈0`。 / Does Nero satisfy
   `J_g M^-1 tau_null≈0`?
10. 四个 MIT controller adapter 的 `torque_feedback`、`torque_model_requested`、
   `torque_task_requested` 和 `torque_auxiliary_requested` 是否组成
   `torque_total_requested`；`torque_feedforward_sent` 与反馈相加是否等于
   `torque_total_estimated`；饱和时是否给出 `torque_saturation_reason`。 / Across
   all four MIT controller adapters, do feedback, requested model/task/auxiliary
   torque compose `torque_total_requested`; does sent feedforward plus feedback
   equal `torque_total_estimated`; and is a saturation reason present when
   clipping occurs?
11. `/arm_dynamics_state` 是否约为 100 Hz，实验 `summary.period.mean` 是否接近
    `0.01 s`；Nero v111/v112 的 `velocity` 是否为位置差分估计，其余 profile
    是否来自 SDK。 / Is
    `/arm_dynamics_state` near 100 Hz, with finite-difference `velocity` for
    Nero v111/v112 and SDK velocity for other profiles, and is experiment
    `summary.period.mean` near `0.01 s`?
12. 静止无接触时，残差是否稳定但可能存在摩擦/模型偏置；接触时符号是否符合关节
    正方向。 / At rest without contact, is the residual stable despite possible
    friction/model bias, and does contact follow the positive joint sign?
13. Interaction Mode Lifecycle 是否始终保持普通、阻抗、导纳、混合四者互斥；
    按 `H` 后 sample 是否显示 `hybrid`、默认 mask 是否为
    `[0,0,0,0,0,1]`；随后按 `I` 是否先提交普通 planned-position hold，再进入
    MIT 阻抗。 / Does the Interaction Mode Lifecycle keep normal, impedance,
    admittance, and hybrid mutually exclusive; after `H`, does the sample show
    `hybrid` with default mask `[0,0,0,0,0,1]`; and after `I`, is normal
    planned-position hold committed before MIT impedance enters?
14. `/arm_control_sample` 的 `interaction_mode` 是否明确为
    `admittance_zero_force` 或 `admittance_resistive`；前者松手后是否停止在新
    偏置，后者是否回到锚点。 / Does `/arm_control_sample` identify
    `admittance_zero_force` or `admittance_resistive`; does the former settle
    at its new offset while the latter returns to the anchor?
15. `ros2 topic echo /arm_control_sample` 是否显示所选 controller、相同周期的
    state/reference/command 和 `schema_version: 1`。 / Does
    `/arm_control_sample` show the selected controller, same-cycle
    state/reference/command, and `schema_version: 1`?
16. recorder status 是否给出唯一 run directory；正常收尾后四个文件是否存在，
    `summary.sample_count` 是否等于 `samples.jsonl` 行数。 / Does recorder
    status expose a unique run directory, do all four files exist after a
    normal close, and does `summary.sample_count` equal the number of JSONL
    sample lines?

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
8. **Nero/Piper-L 导纳 / Nero/Piper-L admittance**：已完成独立
   `cartesian/` 共用任务几何、完全分开的 `impedance/`/`admittance/` adapter、
   Nero 抗漂移柔顺零力、`resistive`、机器人独立参数、`O` 键、I/O/H 互锁、7/6 轴
   DLS wrench、受限旋量 Jacobian 速度 IK、实测位置重锚定和低增益 MIT adapter；整合后的
   实机阈值与手感仍需低速验证。 /
   Shared `cartesian/` task geometry, separated `impedance/` and `admittance/`
   adapters, Nero anti-drift soft zero force, `resistive` dynamics,
   robot-specific tuning, the `O` key, I/O/H interlock, seven/six-axis DLS
   wrench mapping, bounded screw-Jacobian velocity IK, measured-position
   reanchoring, and low-gain MIT adapter are implemented; hardware thresholds
   and feel of the integrated controller still require low-speed validation.
9. **Twist 混合控制 / Twist hybrid control**：已完成新 `hybrid/` 模块、`H` 键、
   固定基座轴 `S_a` 与互补 `S_i`、单一 controller 输出、受限旋量速度 IK、共享
   速度/力矩保护和四态互锁；曲面法向动态投影与整合版本实机验证仍待完成。 /
   The new `hybrid/` module, `H` key, fixed-base-axis `S_a` and complementary
   `S_i`, one-owner controller output, bounded screw-velocity IK, shared
   speed/torque guards, and four-state interlock are implemented; dynamic
   surface-normal projection and hardware validation of this integration
   remain pending.
10. **统一 controller seam / Unified controller seam**：已完成四个 adapter 的
   `ControlInput -> ControlResult`、统一命令类型、JSON sample 和共用测试面。 /
   The four adapters now share `ControlInput -> ControlResult`, normalized
   command types, JSON samples, and one test surface.
11. **实验运行 / Experiment runs**：已完成 manifest、JSONL sample/event、原子
    summary、Memory/JSONL sink 和可选 ROS recorder；自动轨迹脚本和跨 run
    统计比较仍待完成。 / Manifest, JSONL samples/events, atomic summary,
    Memory/JSONL sinks, and the optional ROS recorder are implemented;
    scripted trajectories and cross-run statistical comparison remain.

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
9. 实现一个只返回 `PositionCommand` 的 hold controller adapter，并通过
   `ControlEngine` 与 Memory sink 对它做十周期实验。 / Implement a hold
   controller adapter returning only `PositionCommand`, then run ten cycles
   through `ControlEngine` and the Memory sink.
10. 对同一路径分别运行 `joint_impedance` 与 `cartesian_impedance`，比较两个
    `summary.json` 的周期分布、RMSE 与最大估计力矩，并说明为何不能仅凭 RMSE
    宣称稳定。 / Run the same path with joint and Cartesian impedance, compare
    period, RMSE, and maximum estimated torque across summaries, and explain
    why RMSE alone does not establish stability.
11. 令 `S_a=diag(0,0,0,0,0,1)`，验证 `S_i=I-S_a`、`S_a S_i=0`，并解释为何
    本项目中第六维而非第三维才是平移 Z。 / Set
    `S_a=diag(0,0,0,0,0,1)`, verify `S_i=I-S_a` and `S_a S_i=0`, and explain
    why translational Z is the sixth rather than third component in this
    project.

## 16. 术语表 / Glossary

| 中文 | English | 含义 / Meaning |
| --- | --- | --- |
| 关节阻抗 | Joint impedance | Joint-space spring-damper torque law |
| 笛卡尔阻抗 | Cartesian impedance | Tool-space wrench spring-damper law |
| 笛卡尔导纳 | Cartesian admittance | External wrench drives a virtual mass-damper-spring pose |
| 互补混合控制 | Complementary hybrid control | Selected task axes produce admittance Twist while the complementary axes produce impedance wrench inside one controller |
| 选择矩阵 | Selection matrix | Diagonal task projector `S_a`, with complementary `S_i=I-S_a` |
| 柔顺零力 | Soft zero force | Near-zero desired wrench with weak holding, damping, and stick/slip anti-drift resistance |
| 阻力导纳 | Resistive admittance | Positive damping and stiffness resist motion and restore the anchor |
| 互锁 | Interlock | Normal/impedance/admittance/hybrid are mutually exclusive and every cross-mode switch passes through normal |
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
| 受限旋量速度 IK | Bounded screw velocity IK | 从 PoE 空间 Jacobian 构造工具几何 Jacobian，并以加权 DLS、关节速度和预测位置边界求解 `dq_ref` / Computes `dq_ref` from a PoE space Jacobian through the tool geometric Jacobian, weighted DLS, joint-speed limits, and predictive-position bounds |
| 模型补偿 | Model Compensation | 在共享 interface 中选择重力、偏置或完整逆动力学 / Shared interface selecting gravity, bias, or full inverse dynamics |
| 交互安全边界 | Interaction Safety Limits | 四个 MIT adapter 共用且一次校验的力矩、参考/实测速度和关节位置边界 / One validated source of torque, reference/measured-speed, and joint-position bounds shared by all four MIT adapters |
| MIT 安全包络 | MIT Safety Envelope | 检查反馈力矩可行性，并在估算总力矩限制内分配前馈 / Checks feedback-torque feasibility and bounds feedforward within the estimated total-torque limit |
| 控制周期保护器 | Control Cycle Guard | 每周期检查反馈完整性、周期、实测位置和速度 / Per-cycle checks for feedback completeness, period, measured position, and velocity |
| 持续速度保护器 | Sustained Velocity Guard | 将可去抖的实测速度停止阈值与单周期硬停止阈值分开 / Separates a debounced measured-speed stop threshold from an immediate hard-stop threshold |
| 交互模式生命周期 | Interaction Mode Lifecycle | 保持普通/阻抗/导纳/混合互斥，并强制跨模式经过普通态 / Keeps normal/impedance/admittance/hybrid mutually exclusive and forces cross-mode transitions through normal |
| 旋转向量 | Rotation vector | `Log(R_d R^T)^vee` 轴角姿态误差，不是 RPY 差值 / Axis-angle orientation error, not an RPY difference |
| 空间误差旋量 | Space-error twist | `Log(T_d T^-1)^vee`，表达在基坐标系并与 `J_s` 配对 / Base-frame SE(3) error paired with `J_s` |
| 广义动量 | Generalized momentum | `p=M(q)qdot` |
| 动量观测残差 | Momentum-observer residual | 外部关节力矩的一阶低通估计；也包含未建模扰动 / First-order estimate of external joint torque that also contains unmodelled disturbances |
| 阻尼最小二乘 | Damped least squares | Regularized solve of `tau_ext=J^T F_ext` near rank loss |
| MIT 命令 | MIT command | Native motor command with `p_des/v_des/kp/kd/t_ff` |
| 控制输入 | Control Input | 一个周期的 state、reference、wrench、timestamp 与 period / Same-cycle state, reference, wrench, timestamp, and period |
| 控制结果 | Control Result | MIT/planned 命令及同周期诊断 signals / MIT/planned command plus same-cycle diagnostic signals |
| controller adapter | Controller adapter | 实现 `ControlInput -> ControlResult` 且不访问 ROS/CAN 的算法 / Algorithm implementing the common seam without ROS/CAN access |
| 控制样本 | Control Sample | 可记录的 schema-v1 input/result JSON / Recordable schema-v1 input/result JSON |
| 实验运行 | Experiment Run | 一个 manifest、顺序 sample/event 流和结束 summary / One manifest, ordered sample/event streams, and closing summary |

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
- 本仓库 `armbycontroller/modeling/screw_model.py`：PoE、空间雅可比和 RNEA 的实际实现。
  / This repository's `armbycontroller/modeling/screw_model.py`: the implemented PoE,
  space Jacobian, and RNEA model.
- 本仓库 `armbycontroller/ros/keyboard_controller_node.py`：当前 ROS 状态机、
  AGX MIT/planned adapter 与互锁基线。 / This repository's
  `armbycontroller/ros/keyboard_controller_node.py`: the current ROS state
  machine, AGX MIT/planned adapters, and interlock baseline.
- 本仓库 `armbycontroller/control/core.py`、`cartesian/spatial.py`、
  `impedance/controllers.py` 与 `admittance/controller.py`：统一 controller
  interface、共用任务几何、命令 schema 和解耦 adapter。 / This repository's
  controller interface, shared task geometry, command schema, and decoupled
  adapters.
- 本仓库 `armbycontroller/experiment/core.py`：实验 manifest、证据流、sink seam
  和汇总指标的实际定义。 / This repository's implemented experiment manifest,
  evidence streams, sink seam, and summary metrics.
- AgileX Robotics, Piper API：后台读取线程、`MessageAbstract.timestamp`、
  `get_joint_angles()` 与高频 `get_motor_states()`：
  https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md
- AgileX Robotics, Nero API：相同状态接口（7 轴）：
  https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/nero_api.md
