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

ROS 2 包名与仓库目录统一为
`agxarm_control_by_gamecontroller`；`armbycontroller` 仅作为内部 Python
import namespace。因此 `colcon --packages-select`、`ros2 launch` 和
`ros2 run` 均使用前者，Python import 使用后者。 / The ROS 2 package
and repository directory share the name `agxarm_control_by_gamecontroller`;
`armbycontroller` is retained only as the internal Python import namespace.
Therefore, `colcon --packages-select`, `ros2 launch`, and `ros2 run` use the
former, while Python imports use the latter.

项目自带 Nero、Piper-L 和 Revo2 的完整 URDF/Xacro 与 mesh，并在
`armbycontroller` 内实现 PoE、雅可比、CRBA 和 RNEA。运行时只解析本包安装目录
或源码树中的 `agx_arm_urdf`，不依赖 `nero_screw_dynamics`。 / The project
ships the complete Nero, Piper-L, and Revo2 URDF/Xacro and mesh assets and
implements PoE, Jacobian, CRBA, and RNEA inside `armbycontroller`. Runtime
model resolution uses only this package's installed `agx_arm_urdf` or source
tree and does not depend on `nero_screw_dynamics`.

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

真实硬件启动统一采用两连接并存流程。第一条以 SDK `default` profile 创建探测
实例，执行 `connect()` 后立即发送一次 `enable()`，再取得并保存完整 firmware
字典；根据 `software_version` 创建对应 Nero/Piper-L profile 的第二条正式连接。
两次 `connect()` 之间不调用 `disable()`、不调用 `disconnect()`、也不等待；探测
实例会与正式实例一起保留到节点退出。探测实例不发送模式、运动或固件写入命令。 /
Real-hardware startup keeps two connections alive. The first uses the SDK
`default` profile, calls `connect()`, immediately sends one `enable()`, and
saves the complete firmware dictionary. A second formal connection is then
created with the Nero/Piper-L profile selected from `software_version`.
There is no `disable()`, `disconnect()`, or delay between the two `connect()`
calls; both instances remain alive until node shutdown. The probe sends no
mode, motion, or firmware-write command.

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
F_c   = Kx e_x + Dx (xdot_d - xdot) + F_I
tau_x = Jg(q)^T F_c
```

Nero 默认启用一个刻意很弱的六维衰减位置积分，用于慢慢消除摩擦、死区和模型
偏差留下的回正误差；它不是恒力控制器。空间顺序为 `[Rx,Ry,Rz,X,Y,Z]`，默认
增益为 `[0.1,0.1,0.1,0.2,0.2,0.2]`，旋转/平移单位分别为
`N·m/(rad·s)` 与 `N/(m·s)`。积分状态直接用附加 wrench 表示，并带泄漏、死区、
近目标门控、推/松手滞回和范数上限： / Nero enables a deliberately weak
six-dimensional leaky position integral by default to slowly remove residual
return error from friction, deadband, and model mismatch; it is not a
constant-force controller. Spatial order is `[Rx,Ry,Rz,X,Y,Z]`, with default
gain `[0.1,0.1,0.1,0.2,0.2,0.2]` in rotational `N·m/(rad·s)` and translational
`N/(m·s)` units. The state is stored directly as an additive wrench and has
leak, deadband, near-target gating, push/release hysteresis, and norm limits:

```text
e_eff = deadband(e_x)
F_I[k] = limit(exp(-lambda dt) F_I[k-1] + Ki e_eff dt)
```

只有动量观测 wrench 新鲜且已从 `1.0 N/0.2 N·m` 推动门限降到
`0.5 N/0.1 N·m` 松手门限以下、位姿误差位于近目标窗口、笛卡尔 wrench 和关节
总力矩都未饱和时才累积。笛卡尔目标变化和模式复位会清零；普通暂停按
`0.05 s^-1` 泄漏，饱和暂停按 `0.1 s^-1`（10 s 时间常数）衰减。 /
Accumulation occurs only with a fresh momentum-observer wrench that has
fallen from the `1.0 N/0.2 N.m` push thresholds below the
`0.5 N/0.1 N.m` release thresholds, with pose error inside the near-target
window and neither Cartesian wrench nor total joint torque saturated.
Cartesian-target changes and mode resets clear the state; ordinary pauses
leak at `0.05 s^-1`, while saturation pauses decay at `0.1 s^-1` (a 10 s time
constant).

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

Nero、Piper-L 和 Revo2 的模型资源位于仓库根目录 `agx_arm_urdf/`，并由
`CMakeLists.txt` 复制到本包 share 目录。默认解析不访问其他工作区包；显式
`urdf_path` 仍具有最高优先级。 / Nero, Piper-L, and Revo2 model assets live
under the repository-root `agx_arm_urdf/` and are copied into this package's
share directory by `CMakeLists.txt`. Default resolution does not access
another workspace package; an explicit `urdf_path` still has highest priority.

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

`tau_cmd` 是当前采样的直接公式结果。共享 MIT envelope 在公式之后同时实施逐
关节绝对力矩上限和估算总力矩变化率限制：默认分别为 ±8 N·m 与
`20 N·m/s`，两者都包含模型、任务、零空间和姿态支撑。进入模式时，变化率历史
从实测电机力矩初始化。

`tau_cmd` is the immediate equation result for the current sample. After that
formula, the shared MIT envelope applies both a per-joint absolute limit and an
estimated-total-torque slew limit, defaulting to ±8 N·m and `20 N·m/s`. Both
include model, task, nullspace, and posture support. Rate history is initialized
from measured motor torque when the mode is entered.

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

### 5.8.1 Stribeck 摩擦与 Smith 预估 / Stribeck friction and Smith prediction

经典静态 Stribeck 模型严格使用： / The classical static Stribeck model is:

```text
tau_f(v) = [tau_c + (tau_s-tau_c) exp(-(|v|/v_s)^alpha)] sign(v) + b v
tau_s >= tau_c >= 0,  v_s > 0,  alpha > 0,  b >= 0
```

`tau_f` 与运动方向同号；在动力学方程中应从驱动力矩中减去，作为补偿时则同号
加入。实现明确取 `sign(0)=0`，因此零速输出为零；该速度静态映射不虚构预滑或
集合值静摩擦。 / `tau_f` has the sign of motion: subtract it as resistance
in the dynamics, or add it as compensation. The implementation explicitly
uses `sign(0)=0`, so its zero-speed output is zero; this velocity-static map
does not invent presliding or set-valued stiction.

离散 Smith 预估器把标称无延迟状态空间模型与纯 `N` 样本延迟分开： / The
discrete Smith predictor separates a nominal delay-free state-space model
from a pure `N`-sample delay:

```text
x[k+1] = A x[k] + B u[k]
y0[k]  = C x[k] + D u[k]
yd[k]  = y0[k-N]
y_hat[k] = y_measured[k] + y0[k] - yd[k]
```

控制器应反馈 `y_hat`。模型完全匹配时，`y_measured=yd`，故 `y_hat=y0`；模型
失配则通过 `y_measured-yd` 修正预测。实现顺序为先用 `x[k],u[k]` 计算输出，再
推进到 `x[k+1]`，不会错一拍。 / A controller should feed back `y_hat`.
With a perfect model, `y_measured=yd` and therefore `y_hat=y0`; model mismatch
enters through `y_measured-yd`. Output is evaluated from `x[k],u[k]` before
the state advances to `x[k+1]`, avoiding an off-by-one sample.

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

#### 5.9.1 平置重力平滑调度 / Smooth horizontal gravity scheduling

Nero 平置安装可对 J2、J4 独立使用正负角经验标定。令过渡半宽为 `Delta`，对每个
被调度关节计算： / A horizontal-mounted Nero can independently apply signed
empirical calibration to J2 and J4. With transition half-width `Delta`, each
scheduled joint uses:

```text
x = clip((q + Delta) / (2 Delta), 0, 1)
w = x^2 (3 - 2x)
k(q) = (1-w) k_negative + w k_positive
b(q) = (1-w) b_negative + w b_positive
tau_g_scheduled = k(q) tau_g_URDF + b(q)
```

默认 `Delta=2 deg`、`k_negative=k_positive=1`、`b_negative=b_positive=0`，
所以未标定时与原 URDF 完全一致。J2、J4不是 2×2 四象限耦合表；每轴只调度自己
的重力分量。完整逆动力学按
`tau_ID_scheduled=tau_ID-tau_g_URDF+tau_g_scheduled` 替换重力，因此惯性和
科氏项不变。控制器和独立动量观测器包装同一个算法，避免把经验修正误报为外力。
/ Defaults are `Delta=2 deg`, unit scales, and zero biases, leaving an
uncalibrated system identical to raw URDF gravity. J2 and J4 are independent,
not a coupled 2-by-2 quadrant table. Full inverse dynamics replaces only
gravity via `tau_ID_scheduled=tau_ID-tau_g_URDF+tau_g_scheduled`, preserving
inertia and Coriolis terms. The controller and separate momentum observer use
the same algorithm so empirical correction is not misreported as contact.

### 5.10 Twist 层互补混合控制 / Complementary Twist-level hybrid control

混合模式仍遵守 `[rx,ry,rz,x,y,z]=[角;线]`。令 `R_bc` 为柔顺参考系到基座的
旋转，`G=diag(R_bc,R_bc)`，局部轴 mask 为 `S_local`，则正交柔顺投影和互补阻抗
投影为 `S_a=G S_local G^T`、`S_i=I-S_a`。参考系可选固定 `base`、进入模式时
捕获的 `tool` 或由旋转向量定义的 `custom`。运行中重配置时，旧虚拟速度投影到
新 `S_a`，旧阻抗误差投影到新刚性子空间，新增刚性方向在实测位姿重锚定。
/ Hybrid mode retains `[rx,ry,rz,x,y,z]=[angular;linear]`. With compliance-frame
rotation `R_bc`, `G=diag(R_bc,R_bc)`, and local mask `S_local`, the orthogonal
compliance projector is `S_a=G S_local G^T` and `S_i=I-S_a`. The frame is
fixed `base`, mode-entry `tool`, or rotation-vector-defined `custom`.
Reconfiguration projects old virtual velocity into the new `S_a`, projects
old impedance error into the new rigid subspace, and captures measured pose
for newly rigid directions.

```text
W_a     = S_a (W_ext - W_des)
V_a     = S_a Admittance(W_a)
W_i     = S_i [K e_X + D (0 - V_measured)]
dq_ref  = bounded_screw_velocity_IK(V_a)
q_ref   = q_measured + dq_ref dt
tau_ff  = Jg^T W_i + tau_null + C(q,dq)dq + g(q)
tau_est = kp (q_ref-q) + kd (dq_ref-dq) + tau_ff
```

速度 IK 以 `sigma_min(Jg)` 分级：从 `0.05` 降到 `0.01` 时连续增加 DLS 阻尼并
把 Twist 缩放到零；非零运动在 `sigma_min<=0.01` 时拒绝。阻抗弹簧/阻尼 wrench
在 `Jg^T` 前分别实施 `4 N.m` 旋转力矩范数与 `10 N` 平移力范数上限。 / Velocity
IK grades singularity by `sigma_min(Jg)`: from `0.05` to `0.01`, DLS damping
increases while Twist scales continuously to zero; nonzero motion is rejected
at `sigma_min<=0.01`. Before `Jg^T`, impedance wrench is norm-limited to
`4 N.m` rotational torque and `10 N` translational force.

`hybrid_desired_wrench` 的顺序为 `[Mx,My,Mz,Fx,Fy,Fz]`，只有 `S_a` 选中的
分量进入导纳。实现由一个 `HybridCartesianController` 独占输出：选择、导纳
Twist、互补阻抗 wrench、受限旋量速度 IK、模型补偿和 MIT 包络在同一周期合成，
不会同时启动原来的纯阻抗与纯导纳 controller。混合模式复用统一交互安全对象的
参考/实测速度边界，并使用导纳低增益 MIT 参数；包含阻抗、零空间、模型和 PD 的估算总力矩实施逐关节
最大 `8 N.m` 上限。自定义参考系可对齐已知曲面法向；自动法向估计尚未实现。
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
terms. A custom frame can align one axis with a known surface normal; automatic
surface-normal estimation remains outside this controller.

## 6. 架构 / Architecture

实际 ROS 节点 topic/service 连接图、主进程 module 架构图和单周期时序图见
[`docs/ROS_NODE_AND_ARCHITECTURE_ZH_EN.md`](docs/ROS_NODE_AND_ARCHITECTURE_ZH_EN.md)。 /
See [`docs/ROS_NODE_AND_ARCHITECTURE_ZH_EN.md`](docs/ROS_NODE_AND_ARCHITECTURE_ZH_EN.md)
for the actual ROS topic/service graph, main-process module architecture, and
one-cycle sequence diagram. 下文的 `<ns>` 表示机器人 DDS 命名空间：Nero 为
`nero`，Piper-L 为 `piper_l`。 / `<ns>` below denotes the per-robot DDS
namespace: `nero` for Nero and `piper_l` for Piper-L.

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
                  pyAgxArm / CAN                /<ns>/arm_control_sample (JSON)
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

### 6.1 Python/C++ 性能边界 / Python/C++ performance boundary

Linux 键盘设备读取、X11 事件处理和 ROS 键状态发布已经由
`src/keyboard.cpp` 实现；这是当前明确的 C++ 低延迟 I/O adapter。主控制器继续
使用 Python，因为 AGX 传输 adapter 是 `pyAgxArm`，而任务空间矩阵运算已经由
NumPy/LAPACK 在编译代码中执行。把节点编排或七维小矩阵封装成 C++/Python FFI
不会自动提高周期效率，还会产生两套公式和序列化语义。 / Linux keyboard-device
reads, X11 event handling, and ROS key-state publication are already
implemented by `src/keyboard.cpp`; it is the current explicit low-latency C++
I/O adapter. The main controller remains Python because its AGX transport
adapter is `pyAgxArm`, while task-space matrix operations already execute in
compiled NumPy/LAPACK code. Wrapping node orchestration or tiny seven-axis
matrices behind a C++/Python FFI would not automatically improve cycle time
and would create duplicate equation and serialization semantics.

2026-08-20 在本机对实际 Nero/Piper-L URDF 进行 300 次热运行微基准；下表是单次
调用 wall time，仅作为迁移决策证据，不是硬实时保证。100 Hz 周期预算为 10 ms。 /
On 2026-08-20, 300 warmed iterations were measured locally against the actual
Nero/Piper-L URDFs. The table reports per-call wall time as migration evidence,
not a hard-real-time guarantee. The 100 Hz cycle budget is 10 ms.

| 热路径 / Hot path | Nero mean / P95 | Piper-L mean / P95 |
| --- | ---: | ---: |
| tool-origin geometric Jacobian | 0.723 / 1.152 ms | 0.562 / 0.625 ms |
| inverse dynamics | 0.818 / 0.882 ms | 0.713 / 0.742 ms |
| momentum-observer model terms | 1.713 / 1.878 ms | 1.485 / 1.560 ms |
| bounded screw-velocity IK | 0.716 / 0.915 ms | 0.620 / 0.685 ms |

因此当前不增加第二个 C++ 控制实现。只有实机记录显示持续 deadline miss，且 profiler
把瓶颈定位到可独立验证的纯数学 kernel 时，才考虑 C++；迁移必须保留 Python
公式作为交叉验证 oracle，直到逐样本数值一致。 / Therefore no second C++
controller implementation is added now. Consider C++ only when hardware logs
show sustained deadline misses and a profiler isolates a pure mathematical
kernel. Keep the Python formula as a cross-validation oracle until numerical
agreement is demonstrated sample by sample.

## 7. 数据流 / Data flow

进入以下控制周期前，硬件 adapter 先执行：`DEFAULT 探测连接 -> enable -> 保存设备
信息 -> 保持连接 -> 按检测 profile 建立第二条正式连接`。Nero 的边界为
`1.11/1.12/1.20`；Piper-L 的
边界为 `S-V1.8-3/8/9`，与当前 pyAgxArm profile 定义一致。无法取得或解析
`software_version` 时启动失败，不会用猜测 profile 建立控制连接。 / Before the
following cyclic flow, the hardware adapter runs `DEFAULT probe connection ->
enable -> save device data -> keep connected -> second formal connection with
the detected profile`. Nero boundaries are `1.11/1.12/1.20`; Piper-L boundaries are
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
    逐关节绝对上限和变化率上限。 / The shared MIT Safety Envelope combines
    feedback, model, task, and auxiliary torque and applies per-joint absolute
    and slew limits to estimated total torque.
12. 每个轴发送 `move_mit(kp=0,kd=0,t_ff=tau_cmd)`。 / Send each axis with
    `move_mit(kp=0,kd=0,t_ff=tau_cmd)`.
13. 控制器每个 100 Hz 周期读取一次 SDK 缓存的 `q/qdot/tau_motor`，并发布
    `/<ns>/arm_dynamics_state`；该流在 planned、阻抗、导纳和混合模式都存在。 / Once per
    100 Hz cycle, the controller reads cached SDK `q/qdot/tau_motor` and
    publishes `/<ns>/arm_dynamics_state`; the stream exists in planned, impedance,
    admittance, and hybrid modes.
14. 独立观测器进程只订阅该 topic；它不连接 CAN，也不再次读取机械臂。 / The
    separate observer process only subscribes to that topic; it neither
    connects to CAN nor reads the arm again.
15. 观测器按消息时间戳逐帧积分；区间 `[t_{k-1},t_k]` 使用上一周期的实测
    `tau_motor[k-1]`，ROS 调度延迟不参与 `dt`。 / The observer integrates every
    timestamped sample; interval `[t_{k-1},t_k]` uses the preceding cycle's
    measured `tau_motor[k-1]`, so ROS scheduling delay does not enter `dt`.
15. 观测器发布 `/<ns>/arm_external_joint_torque`；`JointState.effort` 是 `r`，单位
    N·m。 / The observer publishes `/<ns>/arm_external_joint_torque`; its
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
    为 schema v1 `Control Sample`，发布到 `/<ns>/arm_control_sample`，且永不写 NaN。
    活跃阻抗所需反馈超时后不再继续生成样本或 MIT 力矩，而是切回普通模式。 /
    Every executed controller cycle combines the same `ControlInput` and
    `ControlResult` into a schema-v1 Control Sample on
    `/<ns>/arm_control_sample`, never writing NaN. When feedback required by
    active impedance times out, it stops producing MIT torque/samples and
    returns to normal mode.
19. controller 使能、退出和急停作为离散 `Control Event` 发布到
    `/<ns>/arm_control_event`。 / Controller enable, exit, and emergency stop are
    published as discrete Control Events on `/<ns>/arm_control_event`.
20. 传输无关的 `InteractionModeInterface` 将外部 `normal/impedance/admittance`
    请求规范化为幂等 set-mode 操作，并强制跨模式先提交 `normal`；ROS adapter
    通过三个 `std_srvs/Trigger` 服务接入，并把 schema-v1 状态发布到
    `/<ns>/arm/interaction_state`；其中 `mode_services` 是经 namespace/remap 解析后的
    完整服务名。公开能力不包含 `hybrid`。 / The
    transport-neutral `InteractionModeInterface` normalizes external
    `normal/impedance/admittance` requests into idempotent set-mode operations
    and commits `normal` before every cross-mode transition. Thin ROS adapters
    expose three `std_srvs/Trigger` services and publish schema-v1 state on
    `/<ns>/arm/interaction_state`; `mode_services` contains resolved
    fully-qualified/remapped names, and public capabilities exclude `hybrid`.
21. 可选 recorder 进程订阅两个 JSON topic，流式写入 `samples.jsonl` 和
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
| `armbycontroller/admittance/laws.py` | 两种紧密相关的导纳律：Nero 优先的抗漂移柔顺零力，以及带正阻尼和回中刚度的阻力导纳 / Two closely related admittance laws: Nero-first anti-drift soft zero force and resistive admittance with positive damping and restoring stiffness |
| `armbycontroller/admittance/controller.py` | 导纳 twist 到受限旋量速度 IK、实测位置重锚定和低增益 MIT 的 controller adapter / Controller adapter from admittance twist to bounded screw velocity IK, measured-position reanchoring, and low-gain MIT |
| `armbycontroller/hybrid/selection.py` | 解析局部柔顺轴、`base/tool/custom` 参考系并生成基座坐标正交任务投影 / Parses local compliance axes and `base/tool/custom` frames into base-frame orthogonal task projectors |
| `armbycontroller/hybrid/controller.py` | 单一所有者的互补混合 adapter：选中轴导纳 Twist、其余轴阻抗 wrench、受限旋量 IK、模型补偿和一个 MIT 包络 / One-owner complementary hybrid adapter: selected-axis admittance Twist, remaining-axis impedance wrench, bounded screw IK, model compensation, and one MIT envelope |
| `armbycontroller/api/interaction.py` | 面向 UI/前端且与传输无关的幂等普通/阻抗/导纳 set-mode 合同、结果 schema 和状态 schema；不开放混合命令 / Transport-neutral idempotent normal/impedance/admittance set-mode contract, result schema, and state schema for UI/frontends; no hybrid command is public |
| `armbycontroller/control/core.py` | 统一 `ControlInput -> ControlResult` interface、MIT/planned 命令类型、`ControlEngine` 与 schema v1 sample / Unified controller interface, command types, engine, and schema-v1 sample |
| `armbycontroller/control/model_compensation.py` | 阻抗、导纳与混合共享的重力、偏置和完整逆动力学模型补偿 / Gravity, bias, and full inverse-dynamics Model Compensation shared by impedance, admittance, and hybrid control |
| `armbycontroller/control/mit.py` | 统一 MIT Safety Envelope、总力矩/变化率可行性和力矩分解诊断 / Shared MIT Safety Envelope, total-torque/rate feasibility, and torque-decomposition diagnostics |
| `armbycontroller/control/safety.py` | `InteractionSafetyLimits` 统一四个 MIT adapter 的力矩幅值/变化率、参考/实测速度和关节边界，并提供反馈完整性、周期及持续/硬速度 guard / `InteractionSafetyLimits` unifies torque magnitude/rate, reference/measured speed, and joint boundaries for all four MIT adapters and provides feedback, period, and sustained/hard-speed guards |
| `armbycontroller/control/interaction.py` | `normal/impedance/admittance/hybrid` 互锁和强制经过普通模式的迁移路径 / Normal/impedance/admittance/hybrid interlock and normal-mediated transition paths |
| `armbycontroller/control/trajectory.py` | 无 ROS/CAN 的限加加速度、限加速度和限速度关节参考轨迹 / ROS/CAN-free jerk-, acceleration-, and velocity-limited joint-reference trajectory |
| `armbycontroller/control/smith_predictor.py` | 严格实现 `y_hat=y+y0-yd` 的离散状态空间 Smith 预估器；尚未接入控制周期 / Discrete state-space Smith predictor implementing `y_hat=y+y0-yd`; not yet wired into the control cycle |
| `armbycontroller/friction/stribeck.py` | 逐关节经典静态 Stribeck 摩擦公式、标量广播和物理参数校验 / Per-joint classical static Stribeck equation, scalar broadcasting, and physical-parameter validation |
| `armbycontroller/experiment/core.py` | `ExperimentRun` 生命周期、汇总指标、sink interface、Memory/JSONL adapter / Experiment lifecycle, metrics, sink interface, and Memory/JSONL adapters |
| `armbycontroller/experiment/static_friction.py` | 关节测试顺序、反馈门控力矩阶梯、窗口二次拟合加速度、加速度候选/位移确认、反馈中位数、静摩擦估计和原子 YAML 存储 / Hardware-free joint ordering, feedback-gated torque steps, windowed quadratic acceleration, acceleration-candidate/displacement confirmation, feedback median, stiction estimation, and atomic YAML storage |
| `armbycontroller/hardware/connection.py` | Nero/Piper-L 共用的 DEFAULT 探测、版本映射和不中断使能的第二条 profile 连接 / Shared DEFAULT probe, version mapping, and profile-specific second connection without interrupting enable |
| `armbycontroller/hardware/feedback.py` | 无副作用的 SDK 关节反馈归一化、完整性提取和 Nero 低通差分速度估计 / Side-effect-free SDK joint-feedback normalization, completeness extraction, and filtered Nero velocity estimation |
| `armbycontroller/modeling/lie.py` | 共享 SO(3)/SE(3) 指数、对数、空间误差旋量、伴随矩阵和空间向量原语 / Shared SO(3)/SE(3) exponentials, logarithms, space-error twist, adjoint, and spatial-vector primitives |
| `armbycontroller/modeling/gravity_schedule.py` | Nero 平置 J2/J4 正负区 `scale+bias` 平滑调度，以及只替换重力分量的动力学模型包装 / Smooth signed J2/J4 `scale+bias` scheduling for horizontal Nero and a dynamics-model decorator that replaces gravity only |
| `armbycontroller/modeling/screw_model.py` | URDF PoE FK、空间雅可比、RNEA 逆动力学和一次树回扫 CRBA 质量矩阵 / URDF PoE FK, space Jacobian, RNEA inverse dynamics, and one-sweep CRBA mass matrix |
| `agx_arm_urdf/` | 本包自带的 Nero、Piper-L、Revo2 URDF/Xacro 与可视化 mesh；安装后位于本包 share 目录 / Package-owned Nero, Piper-L, and Revo2 URDF/Xacro and visualization meshes, installed into this package's share directory |
| `armbycontroller/ik/screw.py` | 完整 SE(3) 姿态 IK 与导纳用受限旋量 Jacobian 速度 IK，共享 PoE 模型 / Full-SE(3) pose IK and bounded screw-Jacobian velocity IK for admittance over one PoE model |
| `armbycontroller/ik/core.py` | IK 创建、目标增量和控制器共享工具；唯一工厂是 `create_screw_solver` / IK construction, target increments, and shared controller helpers; `create_screw_solver` is the sole factory |
| `armbycontroller/teleop/keyboard.py` | 与传输无关的固定 25 键协议、边沿检测和限位内 Keyboard Control Intent / Transport-independent fixed 25-key protocol, edge detection, and limit-safe Keyboard Control Intent |
| `armbycontroller/ros/parameters.py` | 主节点统一的 Controller Parameter Surface、机器人 profile、逐关节默认增益和标量/数组展开校验；不连接硬件 / Main-node Controller Parameter Surface, robot profiles, per-joint default gains, and scalar/array expansion validation; never connects hardware |
| `armbycontroller/ros/settings.py` | 在硬件连接前读取并校验机器人、启动、IK、安全和 ROS 接口参数，生成冻结的 Controller Settings Snapshot / Reads and validates robot, startup, IK, safety, and ROS-interface parameters before hardware connection, producing a frozen Controller Settings Snapshot |
| `armbycontroller/ros/telemetry.py` | 独占动力学、控制样本、控制事件和锁存交互状态 publisher 及其消息 schema / Sole owner of dynamics, control-sample, control-event, and latched interaction-state publishers and schemas |
| `armbycontroller/ros/main.py` | 27 行纯 ROS 可执行入口，只负责 `init/spin/shutdown` 生命周期 / 27-line ROS executable entry owning only the `init/spin/shutdown` lifecycle |
| `armbycontroller/ros/node.py` | 611 行 composition root：参数派生对象、运行状态和 ROS topic/service/timer 接线 / 611-line composition root for parameter-derived objects, runtime state, and ROS topic/service/timer wiring |
| `armbycontroller/ros/controller_runtime.py` | controller adapter 注册、标准 Control Input 构造和 telemetry 委托 / Controller-adapter registration, normalized Control Input construction, and telemetry delegation |
| `armbycontroller/ros/interaction_runtime.py` | UI service adapter、普通态中转互锁和阻抗/导纳/混合所有权切换 / UI service adapters, normal-mediated interlock, and impedance/admittance/hybrid ownership transitions |
| `armbycontroller/ros/hardware_session.py` | 两阶段连接、启动安全、反馈归一化、SDK 命令、急停和退出断开 / Two-stage connection, startup safety, normalized feedback, SDK commands, emergency stop, and shutdown disconnect |
| `armbycontroller/ros/control_cycle.py` | 键盘周期调度、四类 MIT tick、笛卡尔目标与单周期唯一命令所有者 / Keyboard-cycle dispatch, four MIT ticks, Cartesian targets, and one command owner per cycle |
| `armbycontroller/observers/momentum.py` | 无 ROS/CAN 的纯广义动量观测器 / ROS/CAN-free generalized-momentum observer |
| `armbycontroller/ros/momentum_observer_node.py` | 只订阅 `/<ns>/arm_dynamics_state` 的独立 ROS adapter；发布外力矩但不控制机械臂 / Separate ROS adapter that only subscribes to `/<ns>/arm_dynamics_state`; publishes external torque without controlling the arm |
| `armbycontroller/ros/experiment_recorder_node.py` | 可选独立记录进程；订阅 sample/event、提供 recording service、不访问 CAN / Optional recorder process; subscribes to samples/events, exposes recording service, and never accesses CAN |
| `config/common.yaml` | 两种机械臂共用的周期、默认 backend 和固件探测时序 / Rates, default backend, and firmware-probe timing shared by both arms |
| `config/nero.yaml` | Nero 独立固件、裸臂、侧装、速度估计、7 轴零空间、J2/J3/J4 混合姿态、两种导纳和观测器参数 / Nero-only firmware, bare-arm side mount, velocity estimation, seven-axis nullspace, J2/J3/J4 hybrid posture, both admittance modes, and observer parameters |
| `config/piper_l.yaml` | Piper-L 独立固件、夹爪、6 轴增益/比例/限制、两种导纳和观测器参数 / Piper-L-only firmware, gripper, six-axis tuning and limits, both admittance modes, and observer parameters |
| `agx_arm_urdf/revo2/{urdf,meshes}` | 为未来工具配置保留的完整 Revo2 模型资产；不得仅因当前配置未引用某个 mesh 而裁剪 / Complete Revo2 model assets retained for future tool configurations; do not prune a mesh solely because the current configuration does not reference it |
| `test/test_cartesian_common.py` | 共享 Jacobian、SE(3) 和虚功映射契约 / Shared Jacobian, SE(3), and virtual-work contracts |
| `test/test_cartesian_impedance.py` | 阻抗符号、耦合、零空间与模型支撑契约 / Impedance sign, coupling, nullspace, and model-support contracts |
| `test/test_arm_control.py` | 现有关节控制、IK、动力学和硬件 adapter 回归 / Existing joint control, IK, dynamics, and hardware-adapter regression |
| `test/test_momentum_observer.py` | 空间动量、动能梯度、残差收敛、离散稳定性及禁止 SDK/CAN 访问 / Spatial momentum, kinetic gradient, residual convergence, discrete stability, and forbidden SDK/CAN-access contracts |
| `test/test_friction_and_smith.py` | Stribeck 公式、零速约定、Smith 延迟时序、完美模型消延迟和失配修正契约 / Stribeck equation, zero-speed convention, Smith delay timing, perfect-model cancellation, and mismatch-correction contracts |
| `test/test_cartesian_admittance.py` | 抗漂移柔顺零力、阻力平衡、旋转向量、边界和重置契约 / Anti-drift soft-zero-force, resistive-equilibrium, rotation-vector, bound, and reset contracts |
| `test/test_control_interface.py` | 四种交互模式的共用 interface、命令、sample schema 和互锁契约 / Shared interface, command, sample-schema, and interlock contracts for four interaction modes |
| `test/test_hybrid_control.py` | 任务顺序、互补选择、导纳目标力投影和阻抗/导纳输出解耦契约 / Task ordering, complementary selection, desired-wrench projection, and impedance/admittance output-decoupling contracts |
| `test/test_experiment.py` | manifest、JSONL、事件顺序、静摩擦加速度起动检测和汇总指标契约 / Manifest, JSONL, event-order, static-friction acceleration-breakaway, and summary-metric contracts |
| `test/test_hardware_connection.py` | 两次连接顺序、探测连接保活、数据保存和版本到 profile 映射契约 / Two-connection ordering, retained probe, saved data, and version-to-profile mapping contracts |
| `test/test_keyboard_disconnect.py` | FIFO 驱动的真实 evdev 断连回归：按键清零并持续发布释放状态 / FIFO-driven real evdev disconnect regression for latched key release |
| `scripts/test_nero_static_friction.py` | 手动运行的 Nero 单关节准静态双向纯力矩测试；以窗口加速度候选和 `0.01°` 位移确认起动，速度仅作硬安全保护 / Manually run Nero single-joint quasi-static bidirectional pure-torque test using a windowed-acceleration candidate plus `0.01 deg` displacement confirmation; speed is safety-only |

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
| `arm_namespace` (launch) | string | — | `__robot__`，解析为 `nero` 或 `piper_l`；封装脚本显式传入同名值 / resolves to `nero` or `piper_l`; wrappers pass the same value explicitly |
| `keyboard_topic` | string | — | 相对名 `arm_keyboard_state`，解析为 `/<ns>/arm_keyboard_state` / relative name resolved below `/<ns>` |
| `firmware` | string | — | `auto`；实机检测结果优先，显式值仅作配置检查与干跑 profile / detected hardware wins; explicit value is a configuration check and dry-run profile |
| `firmware_probe_timeout` | scalar | s | `5.0`；第一阶段取得固件数据的总时限 / total stage-one firmware-data deadline |
| `firmware_probe_poll_period` | scalar | s | `0.1`；无数据时的查询间隔 / retry interval while data is absent |
| `firmware_reconnect_delay` | scalar | s | `0.5`；兼容保留，当前两连接并存流程不等待 / retained for compatibility; current overlapping-connection flow does not wait |
| `impedance_backend` | string | — | `cartesian` (`joint` 可对照 / for comparison) |
| `interaction_torque_limit` | 1 or n | N·m | `8.0` per joint；关节阻抗、笛卡尔阻抗、导纳和混合共用的估算总力矩上限 / estimated-total-torque limit shared by joint/Cartesian impedance, admittance, and hybrid |
| `interaction_torque_rate_limit` | 1 or n | N·m/s | `20.0` per joint；在 100 Hz 下每周期最多变化 0.2 N·m / estimated-total-torque slew limit shared by all four MIT controllers; at 100 Hz, at most 0.2 N·m per cycle |
| `interaction_reference_joint_velocity_limit` | 1 or n | rad/s | `1.0` per joint；阻抗轨迹和导纳/混合旋量速度 IK 共用 / shared by impedance trajectories and admittance/hybrid screw-velocity IK |
| `interaction_measured_joint_velocity_stop_limit` | 1 or n | rad/s | Nero=`2.5`; Piper-L=`1.5` per joint；持续速度停止线 / sustained measured-speed stop threshold |
| `interaction_measured_joint_velocity_hard_limit` | 1 or n | rad/s | Nero=`2.8`; Piper-L=`2.5` per joint；任一 `I/O/H` 模式单周期立即急停 / immediate one-cycle stop in every I/O/H mode |
| `interaction_measured_velocity_violation_cycles` | scalar | cycles | `3`；持续速度超限去抖 / sustained-speed debounce |
| `interaction_joint_limit_margin` | scalar | rad | `0.03`；所有 MIT 周期的实测位置容差，也是导纳/混合预测恢复带 / measured-position tolerance for all MIT cycles and predictive recovery band for admittance/hybrid |
| `feedback_timeout` | scalar | s | `3.0`；启动关节反馈以及 `impedance_enabled:=true` 所需完整实时 `q/dq/torque` 反馈的等待时限；超时保留普通模式 / startup deadline for joint feedback and the complete live `q/dq/torque` bundle required by `impedance_enabled:=true`; timeout preserves normal mode |
| `interaction_feedback_timeout` | scalar | s | `0.10`；SDK 关节角和每轴电机状态源时间戳停止推进后的阻抗退出时限 / impedance exit deadline when any SDK source timestamp stops advancing |
| `interaction_feedback_handover_max_displacement` | scalar | rad | `0.03`；反馈丢失交接时 `abs(qdot) * sample_age` 的逐关节上限，超限则急停 / per-joint `abs(qdot) * sample_age` bound for feedback-loss handover; exceeding it E-stops |
| `cartesian_impedance_rotation_stiffness` | scalar | N·m/rad | Nero=`1.9`；Piper-L=`0.4`，基座 X/Y 旋转 / base-frame X/Y rotation |
| `cartesian_impedance_base_z_rotation_stiffness` | scalar | N·m/rad | Nero=`1.9`（无补强 / isotropic）；Piper-L=`4.0`（基座 Z 补强 / reinforced） |
| `cartesian_impedance_translation_stiffness` | scalar | N/m | Nero=`70.0`; Piper-L=`10.0` |
| `cartesian_impedance_rotation_damping` | scalar | N·m·s/rad | Nero=`0.24`; Piper-L=`0.08` |
| `cartesian_impedance_translation_damping` | scalar | N·s/m | Nero=`1.4`; Piper-L=`0.8` |
| `cartesian_impedance_max_force` | scalar | N | `10.0`；`Jg^T` 前的平移力向量范数上限 / translational-force norm limit before `Jg^T` |
| `cartesian_impedance_max_torque` | scalar | N·m | `4.0`；`Jg^T` 前的旋转力矩向量范数上限 / rotational-torque norm limit before `Jg^T` |
| `cartesian_impedance_position_integral_gain` | 6 | N·m/(rad·s), N/(m·s) | Nero=`[0.1,0.1,0.1,0.2,0.2,0.2]`，弱六维默认；`[0,0,0,2,2,2]` 仅为显式高增益平移试验 / weak six-axis Nero default; translation-only value is an explicit high-gain trial |
| `cartesian_impedance_position_integral_deadband` | 6 | rad, m | Nero=`[0,0,0,0.001,0.001,0.001]` / Nero translational deadband |
| `cartesian_impedance_position_integral_max_rotation_error` | scalar | rad | `0.05`；超过则暂停累积 / pause above this error |
| `cartesian_impedance_position_integral_max_translation_error` | scalar | m | `0.02`；超过则暂停累积 / pause above this error |
| `cartesian_impedance_position_integral_max_force` | scalar | N | `0.75`；积分平移 wrench 范数上限 / integral translational-wrench norm cap |
| `cartesian_impedance_position_integral_max_torque` | scalar | N·m | `0.2`；积分旋转 wrench 范数上限 / integral rotational-wrench norm cap |
| `cartesian_impedance_position_integral_leak_rate` | scalar | 1/s | `0.05`；历史以 `exp(-lambda dt)` 衰减 / exponential history decay |
| `cartesian_impedance_position_integral_saturation_leak_rate` | scalar | 1/s | `0.1`；饱和时 10 s 时间常数 / 10 s saturation-decay time constant |
| `cartesian_impedance_position_integral_external_force_gate` | scalar | N | `1.0`；进入推动状态 / enter pushed state |
| `cartesian_impedance_position_integral_external_force_release` | scalar | N | `0.5`；低于后退出推动状态 / leave pushed state below this value |
| `cartesian_impedance_position_integral_external_torque_gate` | scalar | N·m | `0.2`；进入推动状态 / enter pushed state |
| `cartesian_impedance_position_integral_external_torque_release` | scalar | N·m | `0.1`；低于后退出推动状态 / leave pushed state below this value |
| `cartesian_impedance_position_integral_requires_external_wrench` | bool | — | `true`；无新鲜观测时不累积 / no accumulation without a fresh observation |
| `cartesian_impedance_nullspace_stiffness` | 1 or n | N·m/rad | `0.4`，仅 Nero / Nero only |
| `cartesian_impedance_nullspace_damping` | 1 or n | N·m·s/rad | `0.1`，仅 Nero / Nero only |
| `cartesian_impedance_joint_posture_stiffness` | 1 or n | N·m/rad | Nero=`[0,0.5,0.5,0.6,0,0,0]`; Piper-L=`0` |
| `cartesian_impedance_joint_posture_damping` | 1 or n | N·m·s/rad | Nero=`[0,0.08,0.08,0.12,0,0,0]`; Piper-L=`0` |
| `cartesian_impedance_model_scale` | 1 or n | — | `1.0`，范围 `[0,1]`，逐关节缩放 `Cqdot+g` / per-joint scale for `Cqdot+g` |
| `nero_mount` | string | — | YAML=`side`（横置，home=`[0°,90°,0°,0°,0°,0°,0°]`，顺序 J1→J2→J3…）；`horizontal`（平置，home=全零，顺序 J2→J1→J3→J4…） / `side`: J2 home at 90° in joint order; `horizontal`: all-zero home with J2 first |
| `workspace_outer_margin` | scalar | m | Nero=`0.0174482`，由物理最大 `0.7374482` 得 soft max=`0.72`；声明/Piper-L 默认=`0.10` / Nero-specific margin gives 0.72 m soft maximum; declared/Piper-L default is 0.10 m |
| `nero_horizontal_gravity_schedule_enabled` | bool | — | `true`；仅 `nero_mount=horizontal` 生效 / active only for horizontal Nero |
| `nero_horizontal_gravity_transition_angle` | scalar | rad | `0.0349066`（2°），范围 0.5°–15° / smoothstep half-width |
| `nero_horizontal_gravity_j2_scale` | 2 | — | `[1,1]`，顺序 `[q2<0,q2>0]`，每项范围 `[0,2]` / signed J2 model scales |
| `nero_horizontal_gravity_j2_bias_nm` | 2 | N·m | `[0,0]`，顺序 `[q2<0,q2>0]`，每项范围 `[-2,2]` / signed J2 biases |
| `nero_horizontal_gravity_j4_scale` | 2 | — | `[1,1]`，顺序 `[q4<0,q4>0]`，每项范围 `[0,2]` / signed J4 model scales |
| `nero_horizontal_gravity_j4_bias_nm` | 2 | N·m | `[0,0]`，顺序 `[q4<0,q4>0]`，每项范围 `[-2,2]` / signed J4 biases |
| `tool_configuration` | string | — | Nero 文件=`none` 裸臂 / bare arm；Piper-L 文件=`gripper` |
| `nero_velocity_estimation_enabled` | bool | — | `true`，仅 v111/v112 / v111/v112 only |
| `velocity_filter_time_constant` | scalar | s | `0.03`，一阶低通差分 / first-order filtered finite difference |
| `mit_command_rate` | scalar | Hz | `100.0` |
| `dynamics_state_topic` | string | — | 相对名 `arm_dynamics_state` → `/<ns>/arm_dynamics_state`; `effort=tau_motor` |
| `momentum_observer_enabled` | bool | — | `true`，只启动/停止独立观测进程 / only starts/stops the separate observer process |
| `momentum_observer_rate` | scalar | Hz | `100.0`，预期输入频率；计算由每条输入消息触发 / expected input rate; every input message triggers an update |
| `momentum_observer_gain` | 1 or n | 1/s | `10.0` |
| `momentum_observer_max_period` | scalar | s | `0.05`；较大数据间隙时重置 / reset after a larger stream gap |
| Stribeck `tau_s`, `tau_c` | 1 or n | N·m | 无运行默认；调用方必须标定，且 `tau_s>=tau_c>=0` / no runtime default; caller-calibrated |
| Stribeck `v_s`, `b`, `alpha` | 1 or n | rad/s, N·m·s/rad, — | 无运行默认；`v_s>0`, `b>=0`, `alpha>0` / no runtime default |
| Smith `A,B,C,D` | matrices | discrete model dependent | 无运行默认；必须是同一采样周期的离散标称模型 / no runtime default; one discrete nominal model at a common sample period |
| Smith `delay_samples` | scalar | samples | 无运行默认；非负整数纯延迟 / no runtime default; nonnegative integer pure delay |
| `external_torque_topic` | string | — | 相对名 `arm_external_joint_torque` → `/<ns>/arm_external_joint_torque`；`effort` 为 N·m / `effort` is N·m |
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
| `admittance_wrench_timeout` | scalar | s | `0.10`; stale or source-stamped-old wrench exits admittance/hybrid to measured-position hold |
| `move_home_on_start` | bool | — | `false`; startup never moves home unless explicitly requested |
| `reset_emergency_stop_on_start` | bool | — | direct-launch default=`false`; both Nero/Piper-L wrappers explicitly pass `true` |
| `disable_arm_on_shutdown` | bool | — | `false`; graceful shutdown disconnects without sending `disable()` so an external controller can take over |
| `admittance_mit_kp` | n | N·m/rad | Nero=`[0.32,0.24,0.32,0.24,0.32,0.28,0.28]`; Piper-L=`[0.3,0.5,0.5,0.5,1.0,0.3]` |
| `admittance_mit_kd` | n | N·m·s/rad | Nero=`0.08`; Piper-L=`0.01` per joint |
| `admittance_mit_model_scale` | scalar | — | `1.0`, range `[0,1]`; gravity feedforward scale |
| `admittance_task_weights` | 6 | — | `[0.4,0.4,0.4,1,1,1]` in `[angular;linear]` order |
| `admittance_velocity_dls_damping` | scalar | — | `0.02` |
| `admittance_singularity_slow_threshold` | scalar | — | `0.05`; `sigma_min(Jg)` below this value starts adaptive damping and Twist scaling |
| `admittance_singularity_stop_threshold` | scalar | — | `0.01`; rejects nonzero Twist at or below this `sigma_min(Jg)` |
| `admittance_singularity_damping` | scalar | — | `0.08`; maximum damping added near the stop threshold |
| `hybrid_admittance_axes` | string | — | `z`; local-frame names from `rx,ry,rz,x,y,z`, comma/space separated |
| `hybrid_admittance_frame` | string | — | `base`; `base`, mode-entry `tool`, or `custom` |
| `hybrid_admittance_frame_rotation` | 3 | rad | `[0,0,0]`; SO(3) rotation vector from custom compliance frame to base |
| `hybrid_desired_wrench` | 6 | N·m, N | `[0,0,0,0,0,0]` in `[Mx,My,Mz,Fx,Fy,Fz]` order; only selected admittance axes apply |
| `desired_twist` | `6` | rad/s, m/s | 由连续参考的 `Jg(q_ref) qdot_ref` 生成 / generated as `Jg(q_ref) qdot_ref` |
| `gravity` | `3` | m/s² | 由已有 URDF model 配置 / Existing URDF-model configuration |
| `control_sample_topic` | string | — | 相对名 `arm_control_sample` → `/<ns>/arm_control_sample`; schema-v1 JSON `std_msgs/String` |
| `control_event_topic` | string | — | 相对名 `arm_control_event` → `/<ns>/arm_control_event`; JSON `std_msgs/String` |
| `interaction_state_topic` | string | — | 相对名 `arm/interaction_state` → `/<ns>/arm/interaction_state`; transient-local schema-v1 JSON `std_msgs/String` |
| `normal_mode_service` | string | — | 相对名 `arm/set_normal_mode` → `/<ns>/arm/set_normal_mode`; idempotent `std_srvs/srv/Trigger` |
| `impedance_mode_service` | string | — | 相对名 `arm/set_impedance_mode` → `/<ns>/arm/set_impedance_mode`; idempotent `std_srvs/srv/Trigger` |
| `admittance_mode_service` | string | — | 相对名 `arm/set_admittance_mode` → `/<ns>/arm/set_admittance_mode`; idempotent `std_srvs/srv/Trigger` |
| `experiment_recording_enabled` | bool | — | `false`; 是否启动独立 recorder / whether to launch the separate recorder |
| `experiment_output_directory` | path | — | `~/.ros/agxarm_control_by_gamecontroller/experiments` |
| `experiment_name` | string | — | `manual_control` |
| `experiment_flush_every` | scalar | samples/events | `1`; 每条刷新，安全优先 / flush every record, prioritizing recoverability |

启用 URDF 重力/逆动力学补偿时，补偿项经过 `mit_gravity_scale` 和统一安全包络；
不再叠加 `mit_feedforward`。唯一的经验 bias 是 Nero 平置 J2/J4 调度中明确配置的
`*_bias_nm`，默认全零，它属于重力模型修正而不是摩擦补偿。`mit_feedforward`
只在没有 URDF 补偿的关节对照 backend 中作为显式手动力矩。 / With URDF
gravity/inverse-dynamics compensation enabled, model output passes through
`mit_gravity_scale` and the shared safety envelope without adding
`mit_feedforward`. The only empirical bias is the explicit, default-zero
horizontal Nero J2/J4 `*_bias_nm`; it corrects the gravity model and is not
friction compensation. `mit_feedforward` remains a manual torque only when
URDF compensation is absent from the joint comparison backend.

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

三个配置文件用 `/**/arm_keyboard_controller` 和
`/**/arm_momentum_observer` 选择节点，使同一参数层同时匹配 `/nero`、
`/piper_l` 和自定义命名空间。不能退回无通配符的根节点选择器，否则 ROS 2 会静默
忽略该参数块，并让命名空间节点使用参数声明默认值。 / All three files select
nodes through `/**/arm_keyboard_controller` and
`/**/arm_momentum_observer`, allowing the same layers to match `/nero`,
`/piper_l`, and custom namespaces. A root-only selector must not be restored:
ROS 2 would silently ignore that block and leave the namespaced node on its
declared fallback values.

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

- Nero 和 Piper-L 固件探测执行 `connect()` 后立即发送一次 `enable()`，再读取
  firmware。第二条正式连接建立前不发送 `disable()` 或 `disconnect()`；两条 SDK
  连接会并存，直到节点退出统一释放。若探测失败，机械臂可能保持使能，必须使用
  物理急停处理。 / Nero and Piper-L firmware probes send one `enable()`
  immediately after `connect()`, then read firmware. No `disable()` or
  `disconnect()` is sent before the second formal connection; both SDK
  connections coexist until node shutdown. If discovery fails, the arm may
  remain enabled and must be handled with the physical E-stop.
- 固件数据缺失、`software_version` 无法解析或检测 profile 不受当前 SDK 支持时，
  实机启动直接失败。 / Hardware startup fails closed when firmware data is
  absent, `software_version` cannot be parsed, or the detected profile is not
  supported by the installed SDK.
- 关节阻抗、笛卡尔阻抗、导纳和混合都从同一个
  `interaction_torque_limit` 与 `interaction_torque_rate_limit` 读取逐关节估算
  总力矩边界，默认 ±8 N·m 和 `20 N·m/s`，且包含 feedback、模型、任务和辅助
  项。 / Joint impedance, Cartesian impedance, admittance, and hybrid control
  share `interaction_torque_limit` and `interaction_torque_rate_limit`, default
  ±8 N·m and `20 N·m/s`, over the estimated total of feedback, model, task, and
  auxiliary terms.
- `Kx/Dx` 必须对称半正定，以避免明显的主动负刚度/负阻尼配置。 / `Kx/Dx`
  must be symmetric positive semidefinite, rejecting obvious active negative
  stiffness/damping.
- 笛卡尔 wrench 范数限制位于 `Jg^T` 之前，但不能替代之后的逐关节总力矩/变化率
  包络；奇异停止阈值也不能替代物理急停。 / Cartesian wrench norm limits act
  before `Jg^T` but do not replace the downstream per-joint total-torque/rate
  envelope; the singularity stop threshold does not replace a physical E-stop.
- Nero 默认启用的弱六维衰减积分不是无界理想积分器；旋转/平移增益分别只有
  `0.1 N·m/(rad·s)` 与 `0.2 N/(m·s)`，附加 wrench 范数限制为 `0.2 N·m` 和
  `0.75 N`。大推力、旋转/平移误差超过 `0.05 rad/0.02 m`、观测失效、wrench
  饱和或关节总力矩饱和都会阻止继续蓄能；观测失效还会重新锁定松手门控，恢复后
  必须再次低于松手阈值。饱和时已有状态按 10 s 时间常数衰减。笛卡尔目标变化和
  模式进入/退出清零，但仅零空间关节参考变化不会误清零。`[0,0,0,2,2,2]` 是显式
  高增益平移实验，不是默认值。动量观测值不是经标定的力传感器读数，因此这些门限
  不能作为碰撞安全认证。 / Nero's default weak six-axis leaky integral is not
  an unbounded ideal integrator. Rotation/translation gains are only
  `0.1 N·m/(rad·s)` and `0.2 N/(m·s)`, with additive wrench norm caps of
  `0.2 N·m` and `0.75 N`. A large push, rotation/translation error above
  `0.05 rad/0.02 m`, stale observation, wrench saturation, or total-joint-torque
  saturation prevents further accumulation. A stale observation re-arms the
  release gate, and existing state decays with a 10 s time constant while
  saturated. Cartesian-target and mode changes clear it, while a nullspace-only
  joint-reference change does not. `[0,0,0,2,2,2]` is an explicit high-gain
  translation experiment, not the default. The observer estimate is not a
  calibrated force sensor, so these gates are not certified collision limits.
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
- 平置 J2/J4 调度比例限制为 `[0,2]`，bias 限制为 `[-2,2] N.m`，过渡半宽限制为
  0.5°–15°；切换后仍通过统一总力矩和变化率包络。标定时必须卸载、支撑机械臂，
  逐段修改且保持物理急停可触及。 / Horizontal J2/J4 schedule scales are bounded
  to `[0,2]`, biases to `[-2,2] N.m`, and transition half-width to 0.5–15
  degrees; the shared total-torque/rate envelope still follows scheduling.
  Calibrate unloaded with the arm supported, change one region at a time, and
  keep the physical E-stop reachable.
- `move_mit()` 对各轴依次调用，CAN 层不提供整批原子提交或逐帧 ACK。 /
  `move_mit()` is called sequentially per axis; CAN provides neither atomic
  batch commit nor per-frame acknowledgement here.
- Space 归位不会一次下发整组目标。若正在阻抗模式，它先确认恢复普通
  planned-position 模式，然后按安装姿态对应的顺序逐轴进入 home：
  Nero 横置 `side` 为 `[0°,90°,0°,0°,0°,0°,0°]`，顺序为
  J1→J2→J3→…；平置 `horizontal` 为全零，顺序为 J2→J1→J3→J4→…。
  若 Nero 只有一个轴在软限位外，先将该轴最小幅度拉回最近软限位，再开始正式
  顺序；多个轴越界仍拒绝。每轴反馈进入
  `startup_home_tolerance` 后才移动下一轴，任一轴超过 `startup_home_timeout`
  会触发电子急停；该流程按控制周期轮询，因此键盘 `E` 始终可响应。导纳和混合
  模式继续锁定 home。 / Space homing never sends one whole-pose target. If
  impedance is active, it first confirms restoration of normal planned-position
  mode, then moves one joint at a time to the mount-specific home. Nero `side`
  uses `[0°,90°,0°,0°,0°,0°,0°]` in J1→J2→J3→… order; `horizontal` uses all
  zero in J2→J1→J3→J4→… order. If exactly
  one Nero joint starts outside its soft limit, it first makes the minimum
  inward recovery to the nearest limit; multiple out-of-limit joints are still
  rejected. Each joint must enter `startup_home_tolerance` before the next moves;
  exceeding `startup_home_timeout` on any joint triggers the electronic stop.
  The sequence is polled once per control cycle, so keyboard `E` remains
  responsive. Home remains locked during admittance and hybrid control.
- 软件计算上限不是驱动器电流硬限，也不是力传感器测量。 / A software
  command limit is neither a drive-current hard limit nor a force-sensor
  measurement.
- 静摩擦脚本不加入启动项，只能从源码手动运行，并要求普通控制器完全停止、无
  负载、测试轴处于重力矩尽量小的姿态、工作区清空且物理急停可触及。测试轴使用
  `kp=kd=0` 的纯前馈力矩，不加入重力、惯性或科氏模型项；正反向同姿态半差近似
  消除恒定重力/驱动偏置。它默认每档增加 `0.005 N·m`、至少保持 `0.25 s`。每个
  方向先采集 `0.8 s` 零力矩加速度基线；以 `0.12 s` 位置窗口、至少 `0.08 s/8`
  样本的中心二次拟合估计加速度，不使用相邻样本二阶差分。连续 3 周期超过
  `max(0.05 rad/s², median(|a|)+6 MAD)` 后冻结当前力矩档，并在 `0.20 s` 内以
  同向 `0.01°` 位移确认起动，否则作为噪声取消。进入下一档要求加速度低于
  自适应释放门限、最近 `0.15 s` 位置峰峰值不超过 `0.005°`、MIT 包络不再限制
  力矩变化，并且电机力矩反馈至少覆盖 `0.15 s`、中位绝对偏差不超过
  `0.02 N·m`。平均增加率不超过 `0.02 N·m/s`，在 `2 N·m` 停止。窗口速度不参与
  起动或档位有效性判定，只作为独立 `0.2 rad/s` 硬安全线，并在接受结果前检查；
  另有 `5 rad/s²` 加速度安全线。它还监视 `0.05 rad`
  限位余量、其他轴 `0.005 rad` 位移、±`8 N·m` 估算总力矩和 `20 N·m/s` 总力矩
  变化率。只有严格递增的 SDK 关节角时间戳才会更新加速度、候选和档位稳定计数；
  重复缓存只维持当前力矩而不升档，时间戳回退或 `0.1 s` 未刷新立即停止。基线中
  `0.01°` 仅限制被测轴；其余低增益保持轴可在 `0.005 rad` 边界内稳定，随后共同
  重锚。关节反馈一旦失信，收尾不再使用缓存位置，而是直接请求电子急停；其他异常
  也必须先确认两帧时间戳严格递增，才可尝试实测位置保持。任一边界失败即回到
  实测位置保持；若保持恢复失败则请求电子急停。该结果是当前姿态下的近似起动
  力矩，不是标定证书。 / The static-friction script may run only with the
  normal controller fully stopped, no payload, the tested axis near a
  low-gravity-torque pose, a clear workspace, and the physical E-stop
  reachable. The tested axis uses pure feedforward torque with `kp=kd=0` and
  no gravity, inertia, or Coriolis model term; the same-pose bidirectional half
  difference approximately removes constant gravity/drive bias. Each direction
  starts with a `0.8 s` zero-torque baseline. Acceleration is obtained by a
  centered quadratic fit over a `0.12 s` position window with at least
  `0.08 s/8` samples, not adjacent-sample second differences. Three consecutive
  samples above `max(0.05 rad/s^2, median(|a|)+6 MAD)` freeze the current torque
  plateau; same-direction `0.01 deg` displacement must confirm it within
  `0.20 s`, otherwise it expires as noise. A plateau advances only when
  acceleration is below its adaptive release threshold, the latest `0.15 s`
  position range is at most `0.005 deg`, the MIT envelope is no longer
  slew-limited, and at least `0.15 s` of motor-torque feedback has median
  absolute deviation at most `0.02 N.m`. The average increase is at most
  `0.02 N.m/s` up to `2 N.m`. Windowed speed is not a breakaway or plateau
  signal; it is an independent `0.2 rad/s` hard guard checked before accepting
  a result, alongside a `5 rad/s^2` acceleration guard, a `0.05 rad` limit
  margin, `0.005 rad` motion on other joints, ±`8 N.m` estimated total torque,
  and a `20 N.m/s` total-torque slew limit. Only strictly advancing SDK
  joint-angle timestamps update acceleration, candidates, or plateau stability.
  A repeated cache holds the current torque without advancing the ramp; a
  backward timestamp or `0.1 s` without refresh stops the test. During the
  baseline, `0.01 deg` applies only to the tested joint; low-gain held joints
  may settle inside `0.005 rad` before the common pose is re-anchored. Once
  joint feedback is untrusted, shutdown bypasses cached-position hold and
  requests E-stop directly. Other failures require two strictly advancing
  joint samples before measured-position hold is attempted. Any failure restores a
  measured-position hold; failure to restore
  hold requests electronic E-stop. The output is an approximate breakaway
  result at one pose, not a calibration certificate.
  正式 profile 连接使能后，脚本等待最多 `3 s` 取得完整关节角和电机力矩
  缓存，避免把 v112 的首帧尚未到达误判为反馈故障。 / After enabling the
  formal profile connection, the script waits up to `3 s` each for complete
  joint-angle and motor-torque caches, so a not-yet-arrived first v112 frame is
  not misclassified as a feedback failure.
  Nero v112 的关节位置由 J1/2、J3/4、J5/6、J7 多帧异步 CAN 缓存拼接；窗口二次
  拟合与 3 周期候选去抖共同抑制单帧量化跳变。窗口估速仍存在，但只用于硬安全
  停止。确认提示打印本次加速度门限下界、`0.01°` 确认值和速度/加速度安全线，
  便于识别旧脚本。 / Nero v112 joint positions are assembled from asynchronous
  J1/2, J3/4, J5/6, and J7 CAN caches. The windowed quadratic fit and three-cycle
  candidate debounce suppress one-frame quantization jumps. Windowed speed is
  retained only for hard safety. The confirmation prompt prints the acceleration
  floor, `0.01 deg` confirmation, and speed/acceleration guards so an old script
  is visible.
  每档还保存最近 `0.2 s` 电机力矩反馈的中位数。起动结果报告“上一稳定档—首次
  运动档”的命令区间、中点和半档不确定度，并同时报告反馈中位数；正反方向完成
  后分别计算命令侧和反馈侧静摩擦/零偏。电机力矩来自驱动反馈而非独立标定力矩
  传感器，因此两种结果都保留，不把反馈值当作标定真值。 / Each plateau also
  retains the median of the latest `0.2 s` motor-torque feedback. Breakaway
  reports the last-stable/first-moving command bracket, its midpoint and
  half-step uncertainty, and the feedback median. After both directions it
  computes command-derived and feedback-derived stiction/offset separately.
  Motor torque is drive feedback, not an independently calibrated torque
  sensor, so both results are retained rather than treating feedback as truth.
  默认输出为项目内 `config/nero_static_friction.yaml`，使用 schema v1 累积多个
  run。输入 `TEST` 后创建 run，固件识别和每个方向完成后都通过同目录临时文件
  原子替换；因此 Ctrl-C 或后续安全失败不会丢失先前方向。每个 run 保存 outcome、
  固件、CAN 接口、全部有效参数、共同姿态、命令区间、反馈中位数和双向汇总。
  后续控制在最新适用 run 内优先读取
  `joints.JN.summary.recommended_static_friction_nm`，并检查相邻
  `recommended_source`；可用 `--output PATH` 改路径。默认实测文件被 git 忽略。 /
  The default output is `config/nero_static_friction.yaml`, a schema-v1 file
  accumulating multiple runs. A run is created after `TEST`, then atomically
  replaced through a same-directory temporary file after firmware detection
  and every completed direction, preserving earlier data across Ctrl-C or a
  later safety failure. Each run stores outcome, firmware, CAN interface, all
  effective parameters, reference pose, command bracket, feedback median, and
  bidirectional summaries. Within the newest applicable run, downstream
  control should read `joints.JN.summary.recommended_static_friction_nm` and inspect the adjacent
  `recommended_source`; `--output PATH` overrides the location. The default
  hardware-data file is git-ignored.
- 独立动量观测器进程本身是被动的且不触发急停；其残差不再叠加到阻抗控制力矩，
  仍可作为诊断输出和导纳输入。 / The separate momentum-observer process is
  passive and never triggers an emergency stop. Its residual is no longer
  added to impedance torque and remains available for diagnostics and
  admittance input.
- 观测器进程禁止访问 SDK/CAN；输入只能来自控制器复用的 100 Hz
  `/<ns>/arm_dynamics_state`。 / The observer process must not access the SDK/CAN;
  its only input is the controller's reused 100 Hz `/<ns>/arm_dynamics_state`
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
- 主控制器同时检查关节角和每个电机状态的 pyAgxArm 源时间戳；单次 SDK 读取异常
  只复用 `0.10 s` 窗口内的最后完整 `q/dq/torque`。带时间戳的每个源必须先被
  观察到至少推进一次，之后也只有全部源均超过上一完整 bundle 时才刷新控制样本，
  因此启动前遗留的冻结 SDK 缓存不能进入 MIT。任一源停止推进超过该时限时，
  阻抗立即停止 MIT 输出并尝试以最后实测关节切回普通 planned-position。handoff
  样本最多额外允许一个名义 MIT 周期的调度余量，且逐关节必须满足
  `abs(qdot) * sample_age <= 0.03 rad`，绝不回退到历史关节目标。硬件模式确认期间
  `interaction_transitioning=true`；MOVE_J 必须由请求后的新状态确认，持位命令成功后
  才提交 normal。样本过旧、位移不确定度超限、模式未确认或持位发送失败都会锁存
  `interaction_fault_reason` 并触发电子急停，不能宣称已进入 normal。 / The main
  controller checks source timestamps on joint angles and every pyAgxArm motor
  state. Every timestamp-bearing source must first be observed advancing, and
  later control bundles refresh only after all sources pass the last accepted
  bundle, so a pre-existing frozen SDK cache cannot enter MIT. A single SDK read error reuses
  only the last complete `q/dq/torque` inside the `0.10 s` window. If any source stops advancing beyond it,
  impedance stops MIT output and attempts planned-position handoff at the last
  measured joints. That handoff sample receives at most one nominal MIT period
  of scheduling slack, must satisfy per-joint
  `abs(qdot) * sample_age <= 0.03 rad`, and never falls back to an old joint
  target. Output is suppressed while `interaction_transitioning=true`; a fresh
  post-request status must confirm MOVE_J, and the planned hold must succeed
  before normal is committed. Any failed guard latches
  `interaction_fault_reason`, triggers the electronic stop, and cannot claim
  normal mode.
- evdev 键盘的 EOF、短读或致命读取错误会锁存输入故障、关闭 fd、清零 25 个键，并
  持续发布全零；`EINTR` 重试，`EAGAIN/EWOULDBLOCK` 只是正常的暂无事件。 /
  Evdev EOF, partial reads, or fatal read errors latch an input fault, close the
  fd, clear all 25 keys, and keep publishing zero; `EINTR` retries and
  `EAGAIN/EWOULDBLOCK` only mean no event is currently available.
- 导纳/混合只消费时效窗口内且源时间戳递增的 wrench。超过 `0.10 s` 时退出到实测
  位置保持，不再用零向量伪装有效测量。 / Admittance and hybrid consume only
  wrench samples inside the freshness window with increasing source timestamps.
  At `0.10 s` timeout they exit to a measured-position hold instead of treating
  a zero vector as a valid measurement.
- UI 公开接口只允许幂等请求 `normal/impedance/admittance`，且复用同一生命周期
  与 normal 中间态，不提供进入 `hybrid` 的服务。状态 topic 仍如实报告键盘进入
  的 `hybrid`，避免前端误判。 / The public UI API only accepts idempotent
  `normal/impedance/admittance` requests, reuses the same lifecycle and normal
  intermediate state, and provides no service that enters `hybrid`. The state
  topic still reports keyboard-entered `hybrid` so a frontend cannot mistake
  the actual robot mode.
- 所有 `I/O/H` 模式的参考关节速度统一受
  `interaction_reference_joint_velocity_limit=1.0 rad/s` 限制，并在 ROS 硬件
  adapter 先经过同一个实测速度保护器。Nero 任一关节超过 `2.5 rad/s` 连续三个
  周期、Piper-L 超过 `1.5 rad/s` 连续三个周期，或 Nero 单周期超过
  `2.8 rad/s`、Piper-L 单周期超过 `2.5 rad/s`，都会触发电子急停。导纳与混合还保留各自的 wrench、虚拟
  Twist/位移和预测位置限制。 / Every `I/O/H` mode shares
  `interaction_reference_joint_velocity_limit=1.0 rad/s` and first passes the
  same measured-speed guard in the ROS hardware adapter. An electronic stop is
  triggered after three consecutive cycles above `2.5 rad/s` on Nero or
  `1.5 rad/s` on Piper-L, or immediately above `2.8 rad/s` on Nero and
  `2.5 rad/s` on Piper-L.
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
- Stribeck 与 Smith 当前都是无 ROS/CAN 的未接线数学模块，不会产生硬件命令；在
  完成实机标定、延迟辨识、总力矩包络和模式退出验证前不得直接启用补偿。 /
  Stribeck and Smith are currently unwired, ROS/CAN-free mathematical modules
  and cannot issue hardware commands. Do not enable compensation before
  hardware calibration, delay identification, total-torque-envelope, and
  mode-exit validation are complete.
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
- 平置正负区参数不是在线辨识结果，默认只是单位比例和零 bias。错误标定可能把
  静摩擦、线缆力或人工接触写进“重力”并造成主动漂移；零点平滑只能消除力矩
  跳变，不能修复错误系数。 / Horizontal signed parameters are not identified
  online and default to unit scale and zero bias. Bad calibration can absorb
  friction, cable force, or contact into “gravity” and cause active drift;
  zero-angle blending removes a torque jump but cannot repair bad coefficients.
- 静态 Stribeck 映射不描述零速预滑、滞回或真正的粘住状态；Smith 只补偿已知
  纯延迟，模型/延迟失配仍会进入闭环，且经典结构不应直接用于不稳定对象。 /
  Static Stribeck does not model zero-speed presliding, hysteresis, or a true
  stuck state. Smith compensates only a known pure delay; plant/delay mismatch
  remains in the loop, and the classical structure should not be applied
  directly to unstable plants.
- 因此观测残差包含接触、摩擦、齿隙、负载误差、力矩跟踪误差和编码器噪声；未
  验证前不能把它解释为纯接触力矩或安全碰撞阈值。 / The residual therefore
  combines contact, friction, backlash, payload error, torque-tracking error,
  and encoder noise; before validation it is neither pure contact torque nor
  a safe collision threshold.
- `/<ns>/arm_dynamics_state.effort` 是 SDK 电机状态中的力矩估计，不是六维力/力矩
  传感器测量，也不是硬件同步采样。 / `/<ns>/arm_dynamics_state.effort` is the
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
- 导纳依赖动量残差，因而摩擦/模型误差会形成假外力；超时保护能发现数据中断，
  不能区分仍在更新的模型偏差。 / Admittance inherits friction and model error as
  false external wrench; timeout protection detects a broken stream but cannot
  distinguish model bias that continues to update.
- 目标位姿不连续仍是不安全输入；`20 N·m/s` 变化率限制只约束最终估算总力矩，
  不能替代连续参考生成，也不能限制固件未建模的瞬态。 / A discontinuous pose
  reference remains unsafe. The `20 N·m/s` bound limits final estimated total
  torque but cannot replace continuous references or bound unmodeled firmware
  transients.
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
- `/<ns>/arm_control_sample` 在控制进程内进行 JSON 序列化；磁盘写入在独立进程，但
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
colcon build --packages-select agxarm_control_by_gamecontroller
source install/setup.bash
```

该命令执行普通复制安装；删除 `build/` 后，已安装的 Python、URDF 和 mesh 仍然可用。
本包无需先构建或安装 `nero_screw_dynamics`。 / This command performs a
regular copied install. Installed Python, URDF, and mesh files remain usable
after `build/` is removed. `nero_screw_dynamics` does not need to be built or
installed first.

### 配置文件 / Configuration file

ROS 2 YAML 负责参数值和机器人差异，但不能替代参数声明本身：节点仍需声明参数名、
类型/动态类型描述和无 YAML 时的默认值。因此 `config/*.yaml` 保持值配置，
`ros/parameters.py` 保持 Controller Parameter Surface；二者不重复实现控制逻辑。 /
ROS 2 YAML owns parameter values and robot-specific overrides but cannot
replace parameter declarations: the node still must declare names,
type/dynamic-type descriptors, and defaults for runs without YAML. Therefore
`config/*.yaml` remains value configuration and `ros/parameters.py` remains
the Controller Parameter Surface; neither implements control behavior.

配置文件的节点键采用 `/**/arm_keyboard_controller` 与
`/**/arm_momentum_observer`。这是 `arm_namespace` 的必要配套：它让 Nero、
Piper-L 及自定义 namespace 都得到同一 common/robot 参数层，并保留显式 launch
参数的最高优先级。 / Configuration node keys use
`/**/arm_keyboard_controller` and `/**/arm_momentum_observer`. This is required
by `arm_namespace`: Nero, Piper-L, and custom namespaces receive the same
common/robot layers while explicit launch arguments retain highest priority.

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

`arm_namespace` 的特殊默认值 `__robot__` 在 launch 展开时解析为当前
`robot_model`。因此 Nero 的节点、topic 和 service 位于 `/nero/...`，Piper-L
位于 `/piper_l/...`；两个启动脚本也显式传入对应值。所有接口默认参数都是相对
名，因而跟随命名空间；只有显式绝对名覆盖才会绕过隔离。命名空间只隔离
ROS/DDS endpoint，不会隔离默认的 `can0`，不能据此让两个硬件控制器争用同一
CAN interface。 / The special `arm_namespace=__robot__` default resolves to
`robot_model`, placing Nero interfaces below `/nero/...` and Piper-L
interfaces below `/piper_l/...`; both wrapper scripts pass those values
explicitly. Interface defaults are relative names and follow the namespace;
only an explicit absolute override bypasses it. This isolates ROS/DDS
endpoints, not the default `can0`: do not let two hardware controllers contend
for the same CAN interface.

Nero 平置全零 home 的工具半径约为 `0.7184 m`，位于 Nero 专属 Cartesian IK soft
workspace 上限 `0.72 m` 内，但径向余量只有约 `1.6 mm`。继续向外的 Cartesian jog
会被 workspace guard 拒绝，而且全伸展奇异性是独立风险；planned sequential home
仍有效，笛卡尔测试前应先在关节模式向内移动到有支撑、非奇异的安全位姿。该软
上限来自物理最大臂展 `0.7374482 m` 减 Nero 专属 `0.0174482 m` outer margin，未
修改 Piper-L/声明的 `0.10 m` 默认。 / Nero's horizontal all-zero home has a
tool radius of about `0.7184 m`, inside the Nero-specific `0.72 m` Cartesian IK
soft maximum but with only about `1.6 mm` radial headroom. Further outward jog
is rejected, and full-extension singularity remains an independent risk.
Planned sequential home remains valid; move inward in joint mode to a supported,
nonsingular pose before Cartesian testing. The bound is physical maximum
`0.7374482 m` minus Nero's `0.0174482 m` outer margin; Piper-L/the declared
`0.10 m` default is unchanged.

Nero 文件对应当前侧装、无手机械臂，显式设置 `tool_configuration=none`；
Piper-L 文件独立设置 `tool_configuration=gripper`。launch 中显式覆盖该参数时，
同一值会同时传给主控制器和动量观测器，避免两边载荷模型不一致。实机启动时，
两种机械臂都会先
用 `default` profile 连接、enable 并保存数据，保持该连接，再以检测 profile 建立
第二条正式连接；例如 Nero
`1.11 -> v111`、Piper-L `S-V1.8-8 -> v188`。`firmware=auto` 不再使用静态版本
猜测。 / Nero explicitly uses `tool_configuration=none`; Piper-L independently
uses `tool_configuration=gripper`. An explicit launch override is passed to
both the main controller and momentum observer, keeping their payload models
identical. On hardware, both arms first connect with
the `default` profile, enable, save the returned data, keep that connection,
and open a second connection with the detected profile; examples are Nero `1.11 -> v111` and Piper-L
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
python3 -m pytest -q test/test_friction_and_smith.py
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
colcon test --packages-select agxarm_control_by_gamecontroller
colcon test-result --verbose
```

Style lint 只检查 `src/`、`armbycontroller/`、`test/`、`launch/` 和
`CMakeLists.txt` 对应的源码范围；仓库内的 `build/`、`install/` 生成物不会被当作
源码。 / Style lint is scoped to the applicable source paths under `src/`,
`armbycontroller/`, `test/`, `launch/`, and `CMakeLists.txt`; generated
`build/` and `install/` trees are not treated as source.

### UI/前端模式接口 / UI and frontend mode API

三个服务是幂等 set-mode，不是 toggle。重复请求当前模式返回成功且
`changed=false`；阻抗与导纳之间的切换仍自动经过普通态。服务的 `success` 是
标准字段，`message` 是 schema-v1 JSON。混合模式没有公开服务。 / These three
services are idempotent set-mode commands, not toggles. Repeating the active
mode succeeds with `changed=false`; impedance/admittance cross-transitions
still pass through normal automatically. `success` is the standard service
field and `message` contains schema-v1 JSON. Hybrid has no public service.

```bash
ros2 service call /nero/arm/set_normal_mode std_srvs/srv/Trigger '{}'
ros2 service call /nero/arm/set_impedance_mode std_srvs/srv/Trigger '{}'
ros2 service call /nero/arm/set_admittance_mode std_srvs/srv/Trigger '{}'
ros2 topic echo /nero/arm/interaction_state
```

Piper-L 将以上 `/nero` 换成 `/piper_l`。`/<ns>/arm/interaction_state` 使用
reliable + transient-local QoS；后启动的 UI 也会
收到最近快照。`available_modes` 是可请求能力，固定不含 `hybrid`；
`interaction_mode` 是实机真实状态，因此键盘按 `H` 后可报告 `hybrid`。 /
Use `/piper_l` instead of `/nero` for Piper-L.
`/<ns>/arm/interaction_state` uses reliable, transient-local QoS so a UI started
later receives the latest snapshot. `available_modes` lists requestable
capabilities and never includes `hybrid`; `interaction_mode` reports the real
robot state and may therefore show keyboard-entered `hybrid`.

### 实验记录 / Experiment recording

默认不启动 recorder。显式启用会为每次 launch 创建新的 run directory；不会覆盖
已有实验。 / The recorder is not launched by default. Explicit enablement
creates a fresh run directory for every launch and never overwrites an existing
experiment.

```bash
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=piper_l \
  experiment_recording_enabled:=true \
  experiment_name:=joint_vs_cartesian \
  experiment_output_directory:=~/.ros/agxarm_control_by_gamecontroller/experiments
```

输出 / Outputs:

```text
<run-id>/manifest.json
<run-id>/samples.jsonl
<run-id>/events.jsonl
<run-id>/summary.json
```

单独启动 recorder 时，先用 `-r __ns:=/nero`（或 `/piper_l`）设置命名空间，再通过
`/<ns>/arm_experiment_recorder/recording` 的
`std_srvs/srv/SetBool` 开始/结束。`false` 收尾时才写 `summary.json`；急停事件会
写入 `events.jsonl`，但不会由 recorder 自己触发急停。 / When the recorder is
started independently, first set `-r __ns:=/nero` (or `/piper_l`), then use
`/<ns>/arm_experiment_recorder/recording`
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
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 execute_motion:=true \
  nero_mount:=horizontal \
  move_home_on_start:=false reset_emergency_stop_on_start:=false
```

使用 `scripts/start_nero.sh` 或 `scripts/start_piper_l.sh` 选择本地键盘时，设备
输入可写为编号 `3`、事件名 `event3` 或完整路径 `/dev/input/event3`；启动脚本
会统一解析为完整设备路径。命令行简写 `device:=3` 也采用同一规则；两个脚本随后
过滤原始 `device:=...`，只传一次规范化字符串，避免简写重新覆盖。evdev 断连后
读取器锁存故障并持续发布全按键释放状态。 / When
selecting a local keyboard through `scripts/start_nero.sh` or
`scripts/start_piper_l.sh`, the device may be entered as event number `3`, event
name `event3`, or full path `/dev/input/event3`; the startup script normalizes
all forms to the full device path. The `device:=3` command-line shorthand uses
the same rule. Both wrappers then remove every original `device:=...` token and
pass the canonical string exactly once, so a shorthand cannot overwrite it.
If evdev later disconnects, the reader latches the fault and continuously
publishes an all-keys-released state.

`scripts/start_nero.sh` 会先选择键盘输入，最后询问安装姿态：`1` 为横置 `side`，home 是
`[0°,90°,0°,0°,0°,0°,0°]`、顺序 J1→J2→J3→…；`2` 为平置
`horizontal`，home 是全零、顺序 J2→J1→J3→J4→…。显式传入
`nero_mount:=side|horizontal` 时跳过该问题。 / `scripts/start_nero.sh` also
asks for the mounting posture last, after keyboard input: `1` selects side mounting with J2 home at 90°
and joint-number order; `2` selects horizontal mounting with an all-zero home
and J2→J1→J3→… order. Passing
`nero_mount:=side|horizontal` skips this prompt.

### Nero 静摩擦测试 / Nero static-friction test

先停止 `start_nero.sh` 和其他所有 CAN controller，移除负载、清空工作区并保持
物理急停可触及；将测试轴放在重力矩尽量小的姿态，再手动运行源码脚本。该脚本
没有加入任何启动项： / First stop
`start_nero.sh` and every other CAN controller, remove payloads, clear the
workspace, keep the physical E-stop reachable, and place the tested axis near
a low-gravity-torque pose. Run the source script manually; it is not a startup
entry:

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
./scripts/setup_can.sh

# 默认顺序测试 J1→J2→…→J7 / Default: J1 through J7
./scripts/test_nero_static_friction.py

# 只测试 J4 / Test J4 only
./scripts/test_nero_static_friction.py --joint 4

# 改用其他累积 YAML / Use another cumulative YAML
./scripts/test_nero_static_friction.py --joint 4 --output /tmp/j4.yaml
```

测试轴采用 `kp=kd=0` 的纯前馈力矩，不计算重力、惯性或科氏补偿。默认每档增加
`0.005 N·m` 并至少保持 `0.25 s`。每个方向先采集 `0.8 s` 零力矩数据；在最近
`0.12 s` 位置上做中心二次拟合，至少需要 `0.08 s` 和 8 个样本，以二次项直接
得到加速度。加速度起动门限为
`a_on=max(0.05 rad/s², median(|a|)+6 MAD)`，释放/稳定门限为
`a_off=max(0.02 rad/s², median(|a|)+3 MAD)`。同向加速度连续 3 周期超过
`a_on` 后锁存且冻结当前力矩档；若 `0.20 s` 内同向位移达到 `0.01°`，确认起动并
立即制动，否则候选作为噪声失效。只有 `|a|<=a_off`、最近 `0.15 s` 位置峰峰值
不超过 `0.005°`、MIT 力矩包络已稳定，而且至少 `0.15 s` 电机力矩反馈的中位
绝对偏差不超过 `0.02 N·m` 时才进入下一档，因此平均增加率不超过
`0.02 N·m/s`，到 `2 N·m` 最长约需 100 秒/方向。窗口速度不参与起动检测或档位
稳定，只保留为独立 `0.2 rad/s` 安全停止线；另有 `5 rad/s²` 加速度安全线，两者
均在接受起动结果前检查。只有严格递增的 SDK 关节角时间戳才更新估计器、候选或
档位稳定计数；重复缓存只维持当前力矩，`0.1 s` 未刷新或时间戳回退立即停止。
基线的 `0.01°` 只约束被测轴，其他低增益保持轴可在 `0.005 rad` 安全界内稳定，
随后以稳定后的全关节位置重锚。反馈失信时直接请求电子急停；其他异常也必须确认
两帧严格递增的关节角反馈后才尝试实测位置保持。记录包含上一稳定档与触发档、中点、半档不确定度、触发
加速度、有效 `a_on/a_off` 和最近 `0.2 s` 电机反馈力矩中位数；随后先发送 50 ms
零前馈 MIT 阻尼/位置保持，再切回规划模式，使惯性与科氏影响尽可能小。默认执行
正、反两个方向，结果按
`tau_s=(tau_positive-tau_negative)/2` 估计静摩擦，并报告
`(tau_positive+tau_negative)/2` 零偏；零偏包含当前姿态的恒定重力和驱动
偏置。使用 `--help` 查看有界的爬升率、最大力矩、加速度门限、安全上限、位移确认
和采样率选项。 /
The tested joint uses pure feedforward torque with `kp=kd=0`; no gravity,
inertia, or Coriolis compensation is computed. By default torque advances in
`0.005 N.m` plateaus held for at least `0.25 s`. Each direction begins with a
`0.8 s` zero-torque baseline. A centered quadratic fit over the latest `0.12 s`
of positions, with at least `0.08 s` and eight samples, obtains acceleration
from its quadratic term. The trigger is
`a_on=max(0.05 rad/s^2, median(|a|)+6 MAD)` and the release/stability threshold
is `a_off=max(0.02 rad/s^2, median(|a|)+3 MAD)`. Three consecutive
same-direction samples above `a_on` latch a candidate and freeze the current
torque plateau. Same-direction `0.01 deg` displacement must confirm it within
`0.20 s`; otherwise it expires as noise. A plateau advances only with
`|a|<=a_off`, at most `0.005 deg` position range over `0.15 s`, a settled MIT
envelope, and at least `0.15 s` of motor feedback whose median absolute
deviation is at most `0.02 N.m`. The average increase stays at or below
`0.02 N.m/s`, so reaching `2 N.m` takes up to about 100 seconds per direction.
Windowed speed is not used for breakaway or plateau stability; it remains an
independent `0.2 rad/s` safety stop, alongside a `5 rad/s^2` acceleration stop,
and both run before a result is accepted. Only a strictly advancing SDK
joint-angle timestamp updates the estimators, candidate, or plateau-stability
count. Repeated cache reads hold the current torque; a backward timestamp or
`0.1 s` without refresh stops the test. During the baseline, `0.01 deg` limits
only the tested joint; low-gain held joints may settle within the `0.005 rad`
safety bound before the complete pose is re-anchored. Untrusted joint feedback
requests E-stop directly; other failures require two advancing joint samples
before measured-position hold. A result stores the last-stable and
trigger plateaus, midpoint, half-step uncertainty, trigger acceleration,
effective `a_on/a_off`, confirmation displacement, and latest `0.2 s`
pre-motion motor-torque median. It is followed by 50 ms of zero-feedforward MIT
damping/position hold before
switching to planned mode. This minimizes inertial and Coriolis effects. The
default runs both directions and estimates stiction as
`tau_s=(tau_positive-tau_negative)/2`, and reports
`(tau_positive+tau_negative)/2` as the zero offset, which includes constant
gravity and drive bias at the current pose. Use `--help` for bounded ramp rate,
torque step, maximum torque, acceleration thresholds/safety limits,
movement confirmation, and sample rate.
默认在确认后创建/更新 `config/nero_static_friction.yaml`。YAML schema v1 的根包含
`robot_model`、含 `rad/s²` 加速度在内的明确单位和 `runs`；每个 run 通过 `run_id`
原位更新而非重复追加。每个方向记录检测方法、有效加速度触发/释放门限、触发加速度
与确认位移。
正向完成即落盘，负向完成后补充 `summary`，其中
`recommended_static_friction_nm` 是供后续控制读取的命令区间中点结果。 /
After confirmation the default `config/nero_static_friction.yaml` is created or
updated. Schema v1 contains the robot model, explicit units including
`rad/s^2` acceleration, and `runs`; one run is replaced by `run_id` rather than
duplicated. Each direction records the detection method, effective acceleration
trigger/release thresholds, trigger acceleration, and confirmation displacement.
Positive completion is saved immediately, and negative completion adds the `summary`, whose
`recommended_static_friction_nm` is the command-bracket result intended for
downstream control.

```python
from armbycontroller.experiment.static_friction import (
    StaticFrictionResultStore,
)

store = StaticFrictionResultStore("config/nero_static_friction.yaml")
j4 = store.latest_joint_summary(4)
friction_nm = None if j4 is None else j4["recommended_static_friction_nm"]
```
不传 `--joint` 时按 J1→J7 顺序执行；每个轴先完成正向、回共同姿态、完成反向、
再回共同姿态，才进入下一轴。安全异常立即停止整个序列；仅“达到最大力矩仍未
起动”会记录为该轴未完成并继续下一轴。 / Without `--joint`, execution is
J1 through J7. Each joint completes positive, returns to the common pose,
completes negative, and returns again before the next joint. A safety exception
stops the whole sequence immediately; only a safe no-breakaway-at-maximum
outcome is recorded as incomplete before continuing to the next joint.

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
`/nero/arm_external_joint_torque` 新鲜后按 `O` 捕获锚定位姿并进入抗漂移柔顺零力；
默认平移 deadband 与虚拟摩擦分别为 `0.25 N`、`0.35 N`，且有 `5 N/m` 弱保持，
因此约小于 `0.6 N` 的静态模型残差不会开始移动；
下游从同一 PoE 模型取得旋量 Jacobian、构造工具几何 Jacobian，再由受限加权
DLS 生成 `dq_ref`，并用验证包同值低增益 MIT 跟踪；共享 Model Compensation
提供重力项，每周期参考从实测 `q` 重锚定，估算总力矩限制为 ±8 N·m、变化率为
`20 N·m/s`；参考关节速度
限制为 `1.0 rad/s`，实测任一关节超过 `2.5 rad/s` 连续三个周期或单周期超过
`2.8 rad/s` 会触发电子急停。
若要验证带阻力且松手回中的版本，在启动命令末尾追加
`admittance_mode:=resistive`。 / Nero defaults to
`admittance_mode=zero_force`. With the arm supported and a fresh
`/nero/arm_external_joint_torque`, press `O` to capture the anchor and enter
anti-drift soft zero force. The default translational deadband and virtual
friction are `0.25 N` and `0.35 N`, with weak `5 N/m` holding; a static model
residual below roughly `0.6 N` therefore does not initiate motion. Downstream,
the bounded screw-Jacobian velocity IK obtains the screw Jacobian from the
same PoE model, constructs the tool geometric Jacobian, and uses weighted DLS
to produce `dq_ref`. It is tracked with the verified package's low MIT gains;
shared Model Compensation supplies gravity, the reference is reanchored to
measured `q` every cycle, and estimated total torque is limited to ±8 N·m and
`20 N·m/s`.
Reference joint speed is limited to `1.0 rad/s`; measured speed above
`2.5 rad/s` for three consecutive cycles or above `2.8 rad/s` for one cycle
triggers the electronic stop. Append
`admittance_mode:=resistive` to test the strongly returning variant.

混合实机试验使用同一启动命令，默认无需修改 YAML：确认观测 wrench 新鲜、支撑
机械臂后按 `H`，当前末端位姿成为阻抗锚点，基座 Z 方向走导纳，其余五维走阻抗；
再次按 `H` 先退出到普通保持。可在启动命令显式追加
`hybrid_admittance_axes:=x,y` 选择多个局部方向；追加
`hybrid_admittance_frame:=tool` 可使用进入时工具轴，或使用
`hybrid_admittance_frame:=custom hybrid_admittance_frame_rotation:=[0,0,1.5708]`
把局部 X 旋到基座 Y。`I`、`O`、`H` 任意互切都
先经过普通模式，混合期间 `P`、home 和手动 jog 锁定。 / Use the same launch
command for a hybrid hardware trial without changing YAML. After confirming a
fresh observer wrench and physically supporting the arm, press `H`: the
current tool pose becomes the impedance anchor, base-frame Z uses admittance,
and the other five dimensions use impedance. Press `H` again to exit through
normal hold. Append `hybrid_admittance_axes:=x,y` for multiple local axes;
use `hybrid_admittance_frame:=tool` for mode-entry tool axes, or `custom` plus
`hybrid_admittance_frame_rotation:=[0,0,1.5708]` to rotate local X onto base Y.
Every switch among `I`, `O`, and `H` passes through
normal mode; `P`, home, and manual jog are locked while hybrid is active.

### Piper-L 实机接线 / Piper-L hardware wiring

```bash
cd /home/yang/demo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=piper_l device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto execute_motion:=true \
  impedance_backend:=cartesian impedance_enabled:=false \
  cartesian_impedance_base_z_rotation_stiffness:=4.0 \
  move_home_on_start:=false reset_emergency_stop_on_start:=false
```

启动后按 `I` 才进入 Cartesian MIT。进入时捕获当前姿态，使首周期任务误差为零；
按 `P` 后用旋量 IK 更新参考。首次实机只能在支撑机械臂、急停可触达和空旷环境下
进行。 / Press `I` after startup to enter Cartesian MIT. Entry captures the
current pose so the first task error is zero; press `P` to update references
through screw IK. First hardware trials require physical arm support, an
accessible emergency stop, and a clear workspace.

若显式设置 `impedance_enabled:=true`，启动流程会在切换 MIT 前等待完整且时间戳已
推进的 `q/dq/torque` 反馈（最长 `feedback_timeout`，默认 `3.0 s`）。首个 SDK
时间戳包只建立活性基线；若后续实时包未到达，节点保持普通位置模式并报告错误，
不会带着不完整反馈进入阻抗。Nero v111/v112 还会再等待一个时间戳推进样本，先
建立差分速度估计，再允许实测速度保护放行 MIT。 / With
`impedance_enabled:=true`, startup waits
up to `feedback_timeout` (default `3.0 s`) for a complete `q/dq/torque` bundle
whose SDK timestamps have advanced before switching to MIT. The first
timestamped bundle only establishes the liveness baseline; if a subsequent
live bundle does not arrive, the node reports the error and remains in normal
position mode. Nero v111/v112 additionally waits for one more advancing
sample so its finite-difference velocity estimate is initialized before the
measured-speed guard can permit MIT entry.

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

实机按 `O` 或 `H` 前必须已有新鲜的 `/<ns>/arm_external_joint_torque`（默认不超过
`0.10 s`）；否则控制器拒绝进入并提示检查该 topic。 / Before pressing `O` or
`H` on hardware, `/<ns>/arm_external_joint_torque` must be fresh (no older than
`0.10 s` by default); otherwise entry is rejected with a topic diagnostic.

两个实机命令都会默认启动 100 Hz 被动动量观测器。进入 MIT 后查看： / Both
hardware commands start the passive 100 Hz momentum observer by default.
After entering MIT, inspect:

```bash
ros2 topic echo /nero/arm_external_joint_torque
# Piper-L: ros2 topic echo /piper_l/arm_external_joint_torque
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

直接 launch 的声明默认 `reset_emergency_stop_on_start=false`：检测到锁存电子急停时
启动保持失能。
检查机械臂和故障原因后，操作者可在单次命令中显式传入 `true`；初始化在使能后
失败会主动 `disable()`，回零超时则触发电子急停。 / The declared direct-launch
default is `reset_emergency_stop_on_start=false`:
startup remains disabled when an
electronic stop is latched. After inspection, an operator may explicitly pass
`true` for one launch. Initialization failures after enable actively call
`disable()`, while a homing timeout triggers the electronic stop.
当前 `start_nero.sh` 和 `start_piper_l.sh` 两个封装脚本都显式传入 `true`，因此运行
任一脚本前必须先检查机械臂和故障原因；直接 `ros2 launch` 仍使用上述 `false`
默认值。 / Both current wrappers, `start_nero.sh` and `start_piper_l.sh`,
explicitly pass `true`, so inspect the arm and fault cause before running either
one; direct `ros2 launch` retains the `false` default.

平置启动时应同时看到控制器日志
`Nero horizontal gravity scheduling active` 和观测器日志
`gravity_schedule=True`。若缺少任一条，先检查两个节点的 `nero_mount` 以及六个
`nero_horizontal_gravity_*` 参数；在默认 `[1,1]`/`[0,0]` 下日志会出现但力矩
仍与原 URDF 相同。 / Horizontal startup should show both controller log
`Nero horizontal gravity scheduling active` and observer log
`gravity_schedule=True`. If either is missing, inspect `nero_mount` and the six
`nero_horizontal_gravity_*` parameters on both nodes. With neutral defaults,
the logs appear while torque remains identical to raw URDF gravity.

UI 服务返回 `success=false` 时，先解析 response `message` 中的 schema-v1 JSON，
再查看 `/<ns>/arm/interaction_state` 的 `interaction_mode`、`arm_ready`、
`emergency_stopped` 和 `reason`。服务拒绝不会绕过原有 preflight、反馈完整性、
wrench freshness 或 normal 中间态。 / When a UI service returns
`success=false`, parse the schema-v1 JSON in response `message`, then inspect
`interaction_mode`, `arm_ready`, `emergency_stopped`, and `reason` on
`/<ns>/arm/interaction_state`. A rejected service never bypasses the existing
preflight, feedback-completeness, wrench-freshness, or normal-intermediate
checks.

`mode_services` 应显示 `/nero/arm/...` 或 `/piper_l/arm/...`（也反映 remap 后的
名字），而不是裸的相对 `arm/...`。若日志出现 `impedance feedback lost`，检查 SDK
关节角和各电机状态的 `timestamp` 是否持续增加；控制器会尝试回到普通保持，若最后
完整样本已超过 `0.10 s + 1/mit_command_rate`、`abs(qdot)*sample_age` 超过
`0.03 rad`、MOVE_J 无法由新状态确认或持位命令失败，则按设计急停。 /
`mode_services` should show `/nero/arm/...` or `/piper_l/arm/...` and reflect
remaps, rather than bare relative `arm/...` names. On `impedance feedback lost`,
check that the SDK joint-angle and every motor-state timestamp continue to
advance. The controller attempts normal hold; it intentionally E-stops if the
last complete sample is older than `0.10 s + 1/mit_command_rate`, per-joint
`abs(qdot)*sample_age` exceeds `0.03 rad`, MOVE_J lacks a fresh confirmation,
or the planned hold command fails.

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
11. Nero 位置积分的 `position_integral_active`、`position_integral_limited`、
    `position_integral_pause_reason`、`position_integral_wrench` 与
    `position_integral_next_wrench` 是否表明积分只在松手、近目标且未饱和时增长；
    推动时应看到 `external_force_gate`/`external_torque_gate`，滞回区应看到
    `external_wrench_hysteresis`，观测过期应看到
    `external_wrench_unavailable` 并重新锁定 `position_integral_push_active`；
    饱和时 `position_integral_decay_rate` 应为 `0.1 s^-1`。 / Do the Nero
    position-integral signals show growth only after
    release, near target, and without saturation? A push should report an
    external force/torque gate, the hysteresis band should report
    `external_wrench_hysteresis`; a stale observation should report
    `external_wrench_unavailable` and re-arm `position_integral_push_active`;
    and saturation should report a
    `position_integral_decay_rate` of `0.1 s^-1`.
12. `/<ns>/arm_dynamics_state` 是否约为 100 Hz，实验 `summary.period.mean` 是否接近
    `0.01 s`；Nero v111/v112 的 `velocity` 是否为位置差分估计，其余 profile
    是否来自 SDK。 / Is
    `/<ns>/arm_dynamics_state` near 100 Hz, with finite-difference `velocity` for
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
14. `/<ns>/arm_control_sample` 的 `interaction_mode` 是否明确为
    `admittance_zero_force` 或 `admittance_resistive`；前者松手后是否停止在新
    偏置，后者是否回到锚点。 / Does `/<ns>/arm_control_sample` identify
    `admittance_zero_force` or `admittance_resistive`; does the former settle
    at its new offset while the latter returns to the anchor?
15. `ros2 topic echo /<ns>/arm_control_sample` 是否显示所选 controller、相同周期的
    state/reference/command 和 `schema_version: 1`。 / Does
    `/<ns>/arm_control_sample` show the selected controller, same-cycle
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
   O(n) 动量递推和残差发布；残差不补偿阻抗静摩擦，尚未完成碰撞阈值验证。 /
   The read-only 100 Hz topic seam, separate process, O(n) momentum recursion,
   and residual publishing are complete. The residual does not compensate
   impedance friction; validated collision thresholds remain incomplete.
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
   任意旋转参考系的正交 `S_a` 与互补 `S_i`、无跳变重配置、单一 controller 输出、奇异分级受限旋量速度 IK、共享
   速度/力矩保护和四态互锁；曲面法向动态投影与整合版本实机验证仍待完成。 /
   The new `hybrid/` module, `H` key, fixed-base-axis `S_a` and complementary
   `S_i`, one-owner controller output, bounded screw-velocity IK, shared
   speed/torque guards, and four-state interlock are implemented; dynamic
   automatic surface-normal estimation and hardware validation of this integration
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
12. 将 Nero 积分显式覆盖为仅平移的高增益 `2 N/(m·s)` 试验，依次模拟 `1.2 N -> 0.8 N ->
    0.4 N` 推力和 wrench 饱和，验证控制器依次处于推动、滞回、松手状态，仅最后
    一种未饱和周期会累积；附加力不超过 `0.75 N`，饱和衰减时间常数为 10 s。 /
    Explicitly override Nero with a translation-only high-gain integral trial
    at `2 N/(m·s)`;
    simulate `1.2 N -> 0.8 N -> 0.4 N` followed by wrench saturation, and
    verify pushed, hysteresis, then released states. Only the released,
    unsaturated case may accumulate; additive force stays below `0.75 N` and
    saturation decay has a 10 s time constant.

## 16. 术语表 / Glossary

| 中文 | English | 含义 / Meaning |
| --- | --- | --- |
| 关节阻抗 | Joint impedance | Joint-space spring-damper torque law |
| 笛卡尔阻抗 | Cartesian impedance | Tool-space wrench spring-damper law |
| 笛卡尔导纳 | Cartesian admittance | External wrench drives a virtual mass-damper-spring pose |
| 互补混合控制 | Complementary hybrid control | Selected task axes produce admittance Twist while the complementary axes produce impedance wrench inside one controller |
| 柔顺子空间投影 | Compliance subspace projector | Orthogonal task projector `S_a=G S_local G^T`, with complementary `S_i=I-S_a` |
| 柔顺零力 | Soft zero force | Near-zero desired wrench with weak holding, damping, and stick/slip anti-drift resistance |
| 阻力导纳 | Resistive admittance | Positive damping and stiffness resist motion and restore the anchor |
| 互锁 | Interlock | Normal/impedance/admittance/hybrid are mutually exclusive and every cross-mode switch passes through normal |
| 公开交互模式接口 | Public interaction-mode API | 与 ROS/Web/GUI 传输无关的幂等普通/阻抗/导纳 set-mode 合同；不公开进入混合模式 / Transport-neutral idempotent normal/impedance/admittance set-mode contract with no public hybrid entry |
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
| 交互安全边界 | Interaction Safety Limits | 四个 MIT adapter 共用且一次校验的力矩幅值/变化率、参考/实测速度和关节位置边界 / One validated source of torque magnitude/rate, reference/measured-speed, and joint-position bounds shared by all four MIT adapters |
| MIT 安全包络 | MIT Safety Envelope | 检查反馈力矩可行性，并在估算总力矩幅值和变化率限制内分配前馈 / Checks feedback-torque feasibility and bounds feedforward within estimated total-torque magnitude and slew limits |
| 控制周期保护器 | Control Cycle Guard | 每周期检查反馈完整性、周期、实测位置和速度 / Per-cycle checks for feedback completeness, period, measured position, and velocity |
| 持续速度保护器 | Sustained Velocity Guard | 将可去抖的实测速度停止阈值与单周期硬停止阈值分开 / Separates a debounced measured-speed stop threshold from an immediate hard-stop threshold |
| 交互模式生命周期 | Interaction Mode Lifecycle | 保持普通/阻抗/导纳/混合互斥，并强制跨模式经过普通态 / Keeps normal/impedance/admittance/hybrid mutually exclusive and forces cross-mode transitions through normal |
| 旋转向量 | Rotation vector | `Log(R_d R^T)^vee` 轴角姿态误差，不是 RPY 差值 / Axis-angle orientation error, not an RPY difference |
| 空间误差旋量 | Space-error twist | `Log(T_d T^-1)^vee`，表达在基坐标系并与 `J_s` 配对 / Base-frame SE(3) error paired with `J_s` |
| 广义动量 | Generalized momentum | `p=M(q)qdot` |
| 动量观测残差 | Momentum-observer residual | 外部关节力矩的一阶低通估计；也包含未建模扰动 / First-order estimate of external joint torque that also contains unmodelled disturbances |
| Stribeck 摩擦 | Stribeck friction | 从静摩擦峰值随速度指数下降到 Coulomb 摩擦并叠加粘性项的静态映射 / Static velocity map that decays exponentially from stiction level to Coulomb friction and adds a viscous term |
| Smith 预估器 | Smith predictor | 用 `y_hat=y+y0-yd` 从反馈特征方程中移除标称纯延迟的模型式补偿器 / Model-based compensator using `y_hat=y+y0-yd` to remove nominal pure delay from the feedback characteristic equation |
| 阻尼最小二乘 | Damped least squares | Regularized solve of `tau_ext=J^T F_ext` near rank loss |
| MIT 命令 | MIT command | Native motor command with `p_des/v_des/kp/kd/t_ff` |
| 控制输入 | Control Input | 一个周期的 state、reference、wrench、timestamp 与 period / Same-cycle state, reference, wrench, timestamp, and period |
| 控制结果 | Control Result | MIT/planned 命令及同周期诊断 signals / MIT/planned command plus same-cycle diagnostic signals |
| controller adapter | Controller adapter | 实现 `ControlInput -> ControlResult` 且不访问 ROS/CAN 的算法 / Algorithm implementing the common seam without ROS/CAN access |
| 控制样本 | Control Sample | 可记录的 schema-v1 input/result JSON / Recordable schema-v1 input/result JSON |
| 实验运行 | Experiment Run | 一个 manifest、顺序 sample/event 流和结束 summary / One manifest, ordered sample/event streams, and closing summary |

## 17. 主要资料 / Primary sources

- ICube Robotics, `cartesian_controllers_ros2`, Cartesian VIC reference-frame,
  rule, state, and singularity-diagnostic implementation:
  https://github.com/ICube-Robotics/cartesian_controllers_ros2
- Yifan Hou, `force_control`, hybrid force/velocity subspace switching and
  separate spring force/torque norm limits:
  https://github.com/yifan-hou/force_control/tree/mainline

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
- Y. F. Liu et al., “Experimental comparison of five friction models on the
  same test-bed,” *Mechanical Sciences* 6, 2015, pp. 15–28,
  https://doi.org/10.5194/ms-6-15-2015. 本实现采用其式 (6) 的 Stribeck 静态
  速度模型。 / Source of the implemented static Stribeck velocity equation.
- O. J. M. Smith, “Closer Control of Loops with Dead Time,” *Chemical
  Engineering Progress* 53(5), 1957, pp. 217–219. 经典 Smith 纯时延补偿结构。 /
  Original Smith dead-time compensation structure.
- Matthias Mayr and Julian M. Salt-Ducaju, “A C++ Implementation of a
  Cartesian Impedance Controller for Robotic Manipulators,” JOSS 9(93), 5194,
  2024, https://doi.org/10.21105/joss.05194. 本项目采用其 `J^T` 任务力矩
  结构作为对照，并在共享 MIT envelope 中采用独立、逐关节可配置的估算总力矩
  变化率限制。 / This project uses its `J^T` task-torque structure as a reference
  and applies an independent configurable per-joint estimated-total-torque slew
  limit in the shared MIT envelope.
- 参考实现 / Reference implementation:
  https://github.com/matthias-mayr/Cartesian-Impedance-Controller
- 本仓库 `armbycontroller/modeling/screw_model.py`：PoE、空间雅可比和 RNEA 的实际实现。
  / This repository's `armbycontroller/modeling/screw_model.py`: the implemented PoE,
  space Jacobian, and RNEA model.
- 本仓库 `armbycontroller/ros/node.py`：当前 ROS 状态机、
  AGX MIT/planned adapter 与互锁基线；`ros/main.py` 仅负责进程生命周期。 /
  This repository's `armbycontroller/ros/node.py`: the current ROS state
  machine, AGX MIT/planned adapters, and interlock baseline; `ros/main.py`
  owns only process lifecycle.
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
