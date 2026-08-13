# 项目学习指南 / Project Study Guide

## 1. 项目目标 / Project goal

本项目通过 ROS 2、键盘输入和 AGX SDK 控制 Nero 与 Piper-L。当前分支
`feature/cartesian-impedance-step-by-step` 的目标，是从已经存在的关节 MIT
阻抗出发，逐层建立公式清楚、坐标一致、可测试的空间笛卡尔阻抗。

This project controls Nero and Piper-L through ROS 2, keyboard input, and the
AGX SDK. The goal of `feature/cartesian-impedance-step-by-step` is to start
from the existing joint MIT impedance and build a formula-explicit,
frame-consistent, testable Cartesian impedance controller in stages.

当前阶段已把纯数学核心接入 `mit_tick()` 和 AGX MIT/CAN。默认
`impedance_backend:=cartesian`；设为 `joint` 可以保留旧关节 MIT 作为对照。
软件集成测试通过不代表已经验证真实机械臂的物理稳定性。

The pure formula core is now wired into `mit_tick()` and AGX MIT/CAN. The
default is `impedance_backend:=cartesian`; selecting `joint` retains the old
joint MIT backend for comparison. Passing software integration tests does not
establish physical stability on real hardware.

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
- 旋转误差使用 base-frame rotation vector。 / Orientation error is a
  base-frame rotation vector.

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

## 8. 模块地图 / Module map

| 模块 / Module | 责任 / Responsibility |
| --- | --- |
| `armbycontroller/cartesian_impedance.py` | 纯数学公式、坐标验证、完整双向阻抗等价关系（仅在逆关系唯一时允许） / Pure formula, frame validation, and full bidirectional impedance equivalence only when the inverse is unique |
| `armbycontroller/screw_model.py` | URDF PoE FK、空间雅可比、RNEA 逆动力学 / URDF PoE FK, space Jacobian, RNEA inverse dynamics |
| `armbycontroller/screw_ik.py` | 基于旋量模型的数值 IK / Numerical IK using the screw model |
| `armbycontroller/keyboard_controller.py` | JOINT/IK 参考、笛卡尔公式调用、绝对力矩上限和 MIT/CAN 发送；也保留旧关节 MIT 对照路径 / JOINT/IK reference, Cartesian formula invocation, absolute torque limit, and MIT/CAN transmission; also retains the old joint MIT comparison path |
| `test/test_cartesian_impedance.py` | 坐标、符号、耦合、功率与模型支撑契约 / Frame, sign, coupling, power, and model-support contracts |
| `test/test_arm_control.py` | 现有关节控制、IK、动力学和硬件 adapter 回归 / Existing joint control, IK, dynamics, and hardware-adapter regression |

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
| `desired_twist` | `6` | rad/s, m/s | 由连续参考的 `Jg(q_ref) qdot_ref` 生成 / generated as `Jg(q_ref) qdot_ref` |
| `gravity` | `3` | m/s² | 由已有 URDF model 配置 / Existing URDF-model configuration |

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

## 11. 已知风险 / Known risks

- `Log(R_d R^T)` 在接近 180° 时数值条件变差。 / `Log(R_d R^T)` becomes
  ill-conditioned near 180 degrees.
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
- 100 Hz Python/ROS 循环不是硬实时。 / A 100 Hz Python/ROS loop is not
  hard real time.
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
5. 是否满足 `tau^T qdot = F^T xdot`。 / Does
   `tau^T qdot = F^T xdot` hold?
6. `ID(q,qdot,0)` 是否只产生科氏/离心和重力支撑。 / Does
   `ID(q,qdot,0)` contain only Coriolis/centrifugal and gravity support?
7. Nero 是否满足 `J_g M^-1 tau_null≈0`。 / Does Nero satisfy
   `J_g M^-1 tau_null≈0`?
8. 日志中的 `task`、`null`、`model` 三项相加后是否等于限幅前 `total`。 /
   Do logged `task`, `null`, and `model` sum to the pre-clip `total`?

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

## 16. 术语表 / Glossary

| 中文 | English | 含义 / Meaning |
| --- | --- | --- |
| 关节阻抗 | Joint impedance | Joint-space spring-damper torque law |
| 笛卡尔阻抗 | Cartesian impedance | Tool-space wrench spring-damper law |
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
- 本仓库 `armbycontroller/keyboard_controller.py`：当前 AGX MIT 方程和硬件发送
  基线。 / This repository's `armbycontroller/keyboard_controller.py`: the
  current AGX MIT equation and hardware-transmission baseline.
