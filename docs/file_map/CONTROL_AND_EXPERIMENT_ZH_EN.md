# 控制与实验 / Control and Experiments

## 目标 / Goal

这些 module 定义算法替换 seam 和可复现实验证据；它们不包含 ROS/CAN 调用。 /
These modules define the algorithm-substitution seam and reproducible evidence;
they contain no ROS or CAN calls.

## 文件 / Files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `armbycontroller/control/core.py` | `ControlState`、`ControlReference`、`ControlInput`、命令类型、`ControlResult`、`ControllerAdapter` interface、`ControlEngine` 和稳定控制样本 schema / normalized types, controller interface, engine, and stable sample schema |
| `armbycontroller/control/adapters.py` | 关节 MIT、笛卡尔阻抗和当前位姿导纳 adapter；力矩限制也集中在这里 / joint MIT, Cartesian impedance, current pose-admittance adapters, and centralized torque limiting |
| `armbycontroller/control/__init__.py` | 控制 module 的稳定导出面 / stable exports for the control module |
| `armbycontroller/experiment/core.py` | `ExperimentRun` 生命周期、指标、sink interface、内存与 JSONL adapter / experiment lifecycle, metrics, sink interface, in-memory and JSONL adapters |
| `armbycontroller/experiment/__init__.py` | 实验 module 的稳定导出面 / stable exports for the experiment module |
| `test/test_control_interface.py` | 所有 controller adapter 共用的输入/输出、限幅和 schema 契约 / shared input/output, limit, and schema contracts |
| `test/test_experiment.py` | manifest、sample/event 顺序、JSONL 文件和汇总指标契约 / manifest, ordering, JSONL, and summary-metric contracts |

## 数据契约 / Data contracts

```text
ControlInput -> Controller Adapter -> ControlResult
     |                                  |
     +------------- control_sample -----+
                         |
                    ExperimentRun
                         |
             Memory sink | JSONL sink
```

`ControlInput` 始终包含有限数值数组；反馈缺失通过 `*_valid` 标志表达，不使用 NaN。
`ControlResult.command` 只能是 MIT batch 或 planned joint position。 / A
`ControlInput` always carries finite arrays; missing feedback is represented by
`*_valid` flags rather than NaN. `ControlResult.command` is either an MIT batch
or a planned joint position.

## 扩展规则 / Extension rules

新增算法实现 `name/reset/step` 后注册到 `ControlEngine`。算法只能计算命令和诊断
信号；订阅、发布、CAN 发送、磁盘写入都留给 adapter。 / A new algorithm implements
`name/reset/step` and is registered with `ControlEngine`. It computes commands
and signals only; subscription, publication, CAN transmission, and disk writes
remain in adapters.
