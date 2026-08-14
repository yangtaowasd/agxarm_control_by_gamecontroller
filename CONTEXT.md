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
