# AGX Arm Control Experiments

本上下文描述 Nero/Piper-L 控制算法从统一周期输入到硬件命令与可复现实验证据的
领域语言。 / This context names the domain from one normalized Nero/Piper-L
control cycle through hardware commands and reproducible experiment evidence.

## Language

**控制输入 / Control Input**:
一个控制周期的实测关节状态、关节参考、外部 wrench、时间戳和周期。 / The
measured joint state, joint reference, external wrench, timestamp, and period
for one control cycle.
_Avoid_: tick arguments, backend inputs

**键盘控制意图 / Keyboard Control Intent**:
固定 25 键协议经边沿检测后产生的关节选择、限位内增量、回零、急停及模式请求；
不依赖 ROS 消息或 CAN。 / Joint selection, limit-safe increments, home,
emergency-stop, and mode requests produced by edge detection over the fixed
25-key protocol; independent of ROS messages and CAN.
_Avoid_: raw key array, ROS keyboard state

**控制参数面 / Controller Parameter Surface**:
主控制节点在接触硬件前一次性声明的 ROS 参数名称、类型和默认值，并根据机器人
profile 选择逐关节默认增益。 / The ROS parameter names, types, and defaults
declared once by the main control node before hardware access, including
per-joint defaults selected from the robot profile.
_Avoid_: init constants, scattered defaults

**控制设置快照 / Controller Settings Snapshot**:
从控制参数面读取并校验后冻结的单次运行配置；在任何 SDK/CAN 连接前创建，并作为
节点组合根的唯一启动配置证据。 / The validated, frozen configuration for one
run read from the Controller Parameter Surface; created before any SDK/CAN
connection and retained as the composition root's startup evidence.
_Avoid_: mutable parameter bag, repeated get_parameter calls

**控制结果 / Control Result**:
控制算法产生的 MIT 或 planned-position 命令，以及同周期诊断信号。 / The MIT
or planned-position command produced by an algorithm together with same-cycle
diagnostic signals.
_Avoid_: raw tuple, backend output

**控制器 adapter / Controller Adapter**:
不访问 ROS/CAN、实现 `ControlInput -> ControlResult` 的具体控制算法。 / A
specific control algorithm implementing `ControlInput -> ControlResult`
without ROS or CAN access.
_Avoid_: controller backend, control plugin

**笛卡尔任务几何 / Cartesian Task Geometry**:
阻抗与导纳共用的基坐标系 `[角; 线]` 顺序、tool-origin 几何雅可比、SE(3) 校验和
`tau=Jg.T F` 虚功映射；不包含任一控制律的 K/D/M。 / The base-frame
`[angular; linear]` convention, tool-origin geometric Jacobian, SE(3)
validation, and `tau=Jg.T F` virtual-work mapping shared by impedance and
admittance; it contains no K/D/M from either control law.
_Avoid_: impedance geometry, admittance Jacobian helper

**笛卡尔柔顺子空间 / Cartesian Compliance Subspace**:
在 `base`、进入模式时捕获的 `tool` 或自定义旋转参考系中，以正交投影矩阵划分导纳
和互补阻抗方向；重配置时投影虚拟速度，并只为新增刚性方向捕获实测位姿。 / An
orthogonal projector partitions admittance and complementary impedance
directions in the `base`, mode-entry `tool`, or a custom rotated frame;
reconfiguration projects virtual velocity and captures measured pose only for
newly rigid directions.
_Avoid_: axis mask, mixed force mode

**受限旋量速度 IK / Bounded Screw Velocity IK**:
从共享 PoE 模型取得空间 Jacobian，转换为工具原点几何 Jacobian，并用带关节速度、
预测位置和任务权重约束的 DLS 将导纳 twist 映射为关节速度参考。 / Uses the
shared PoE model's space Jacobian, converts it to the tool-origin geometric
Jacobian, and maps admittance twist to joint velocity through DLS with joint
speed, predictive-position, and task-weight constraints.
_Avoid_: ordinary DLS, admittance-local IK

**模型补偿 / Model Compensation**:
控制律之外的共享 URDF 力矩请求；按 controller adapter 选择重力 `g(q)`、偏置
`C(q,qdot)qdot+g(q)` 或完整逆动力学，并统一应用模型比例。 / Shared URDF
torque requested outside a control law; each controller adapter selects
gravity, bias, or full inverse dynamics and uses one model-scaling rule.
_Avoid_: gravity hidden in impedance, admittance feedforward helper

**Stribeck 摩擦模型 / Stribeck Friction Model**:
按关节速度从静摩擦峰值指数过渡至 Coulomb 摩擦并叠加粘性项的纯数学映射；当前未
接入控制周期。 / A pure mathematical per-joint velocity map with exponential
transition from stiction level to Coulomb friction plus viscosity; currently
not wired into the control cycle.
_Avoid_: zero-speed stick simulator, enabled friction compensation

**Smith 预估器 / Smith Predictor**:
以标称离散状态空间模型和纯样本延迟构造 `y_hat=y+y0-yd` 的无 ROS/CAN 反馈预测
模块；当前未接入控制周期。 / A ROS/CAN-free feedback prediction module using
`y_hat=y+y0-yd` over a nominal discrete state-space model and pure sample
delay; currently not wired into the control cycle.
_Avoid_: observer, generic future-state extrapolator

**MIT 安全包络 / MIT Safety Envelope**:
统一组合 MIT 的 PD 反馈估计和显式前馈，实施逐关节绝对总力矩上限，并报告请求、
实际下发、饱和与不可满足原因。 / Combines estimated MIT PD feedback and
explicit feedforward under one per-joint absolute total-torque limit, reporting
requested, sent, saturated, and infeasible values.
_Avoid_: controller-specific torque clipping

**控制周期 guard / Control Cycle Guard**:
在 controller adapter seam 校验反馈完整性、关节数、实测位置、实测速度和周期边界，
并用稳定原因标识拒绝不安全周期。 / Validates feedback completeness, joint
count, measured position, measured velocity, and period limits at the
controller-adapter seam, rejecting unsafe cycles with stable reason codes.
_Avoid_: tick-local safety check

**交互模式生命周期 / Interaction Mode Lifecycle**:
拥有 `normal`、`impedance`、`admittance` 的互斥不变量和合法迁移路径；阻抗与导纳
跨模式必须提交 `normal` 后才能进入目标模式。 / Owns the mutual-exclusion
invariant and legal transitions among normal, impedance, and admittance;
cross-mode changes must commit normal before the target mode.
_Avoid_: two independent enable booleans, direct impedance-admittance switch

**柔顺零力 / Soft Zero Force**:
Nero 优先的导纳手感；目标外力接近零，但用弱保持、速度阻尼和有限粘/滑阻力抑制
观测偏置、摩擦和积分漂移。 / The Nero-first admittance feel: desired wrench
is near zero, while weak holding, velocity damping, and bounded stick/slip
resistance suppress observer bias, friction, and integration drift.
_Avoid_: ideal zero force, free integrator

**控制样本 / Control Sample**:
控制输入与控制结果按稳定 JSON schema 合并后的单周期证据。 / One cycle of
evidence combining a Control Input and Control Result under the stable JSON
schema.
_Avoid_: debug log, telemetry blob

**控制事件 / Control Event**:
使能、退出、急停等不属于周期采样的离散状态变化。 / A discrete state change,
such as enable, exit, or emergency stop, outside periodic sampling.
_Avoid_: status string, log message

**实验运行 / Experiment Run**:
共享同一 manifest、控制样本序列、控制事件序列和结束汇总的一次实验。 / One
experiment owning a manifest, ordered Control Samples, ordered Control Events,
and a closing summary.
_Avoid_: bag, log session, test folder
