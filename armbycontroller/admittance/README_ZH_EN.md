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
- `controller.py`：将导纳目标经旋量 IK 变成 planned-position `ControlResult`。 /
  Converts the admittance target through screw IK into a planned-position
  `ControlResult`.

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
