# 笛卡尔导纳 / Cartesian Admittance

本目录只放无 ROS、无 CAN 的导纳数学。空间向量固定为
`[Mx, My, Mz, Fx, Fy, Fz]`；前三项对应旋转，后三项对应平移。两种模式共用
wrench 低通/死区/限幅、虚拟速度/位移上限和 SE(3) 位姿生成。 / This directory
contains ROS/CAN-free admittance mathematics. Spatial vectors always use
`[Mx, My, Mz, Fx, Fy, Fz]`; rotation precedes translation. Both modes share
wrench filtering/deadband/clipping, virtual velocity/offset bounds, and SE(3)
pose construction.

## 两种模式 / Two modes

- `zero_force.py`：`M xdd + D0 xd + Kh x + Fstick/slip = Fext`。这是 Nero 优先
  的“柔顺零力”，不是理想零力：`Kh` 很弱，配合速度阻尼和有限粘/滑阻力吸收
  观测偏置与摩擦，避免积分漂移，但不会产生 `resistive` 的强回中手感。 / This
  Nero-first soft-zero-force law is intentionally non-ideal. Weak `Kh`,
  viscous damping, and bounded stick/slip resistance absorb observer bias and
  friction without the strong return feel of `resistive`.
- `resistive.py`：`M xdd + Dr xd + Kr x = Fext`。正阻尼产生速度相关阻力，正
  刚度使末端释放后回到按 `O` 时捕获的锚定位姿。 / Positive damping gives
  velocity-dependent resistance and positive stiffness returns the tool to
  the anchor captured when `O` was pressed.
- `core.py`：两种导纳共用的输入整形、积分器、状态、SE(3) 目标和运动边界。 /
  Input conditioning, integration, state, SE(3) targets, and motion bounds
  shared only by the two admittance modes.
- `controller.py`：把导纳速度交给 `ik/screw.py` 的受限旋量 Jacobian 速度 IK，
  后者从同一 PoE 模型取得空间 Jacobian、转换为工具几何 Jacobian并执行加权
  DLS；controller 以
  `q_ref=q_measured+dq_ref*dt` 每周期重新锚定，并生成低增益 MIT
  `ControlResult`。重力由共享 Model Compensation 生成，命令由共享 MIT Safety
  Envelope 检查估算总力矩。 / Sends the admittance twist to the bounded
  screw-Jacobian velocity IK in `ik/screw.py`; that solver obtains the space
  Jacobian from the same PoE model, converts it to the tool geometric Jacobian,
  and applies weighted DLS. The controller reanchors every cycle with
  `q_ref=q_measured+dq_ref*dt`, and produces a low-gain MIT `ControlResult`;
  shared Model Compensation supplies gravity and the shared MIT Safety
  Envelope protects estimated total torque.

关节参考速度和实测失控保护使用不同阈值：旋量速度 IK 用
`admittance_joint_velocity_limit=0.5 rad/s` 饱和参考速度；实测速度超过
`admittance_measured_joint_velocity_stop_limit=1.0 rad/s` 连续三个控制周期，或
单周期超过 `admittance_measured_joint_velocity_hard_limit=2.0 rad/s`，ROS 硬件
adapter 才触发电子急停。导纳 MIT 估算总力矩不得超过 `8 N·m`。 /
Reference speed and measured-runaway protection use separate thresholds. Screw
velocity IK saturates the reference at
`admittance_joint_velocity_limit=0.5 rad/s`; the ROS hardware adapter triggers
the electronic stop when measured speed exceeds
`admittance_measured_joint_velocity_stop_limit=1.0 rad/s` for three consecutive
control cycles, or exceeds the immediate
`admittance_measured_joint_velocity_hard_limit=2.0 rad/s`. Estimated total
admittance MIT torque may not exceed `8 N·m`.

三个 MIT controller 使用同一组诊断字段，包括 `torque_feedback`、
`torque_model_requested`、`torque_task_requested`、
`torque_feedforward_requested/sent`、`torque_total_requested/estimated` 和
`torque_saturation_reason`。 / All three MIT controllers use the same torque
diagnostics, including feedback, requested model/task/feedforward, sent
feedforward, requested/estimated total torque, and saturation reason.

阻抗和导纳真正共用的坐标、tool-origin Jacobian、SE(3) 校验及虚功映射位于
`armbycontroller/cartesian/`，不在本目录复制。 / Coordinate conventions,
tool-origin Jacobian construction, SE(3) validation, and virtual-work mappings
shared with impedance live in `armbycontroller/cartesian/` and are not copied
here.

`zero_force` 不是零阻尼或零保持模式。令全部抵抗为零会把短暂噪声积分成持续速度，
不适合有时延的实机。两种模式的输入均来自 URDF 动量观测残差，因此是模型扰动
估计，不是六维力传感器真值。 / `zero_force` is neither zero damping nor
zero holding. Removing all resistance turns brief noise into persistent
velocity and is unsafe with hardware delay. Both modes consume a URDF
momentum-observer residual, not a six-axis force-sensor ground truth.
