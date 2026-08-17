# 控制与实验 / Control and Experiments

## 目标 / Goal

这些 module 定义算法替换 seam 和可复现实验证据；它们不包含 ROS/CAN 调用。 /
These modules define the algorithm-substitution seam and reproducible evidence;
they contain no ROS or CAN calls.

## 文件 / Files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `armbycontroller/cartesian/spatial.py` | 阻抗/导纳唯一共用的任务几何：坐标顺序、SE(3)、tool-origin Jacobian 与虚功映射 / sole task-geometry module shared by impedance/admittance: ordering, SE(3), tool-origin Jacobian, and virtual-work mappings |
| `armbycontroller/cartesian/__init__.py` | 共享笛卡尔任务几何的稳定 interface / stable Cartesian Task Geometry interface |
| `armbycontroller/admittance/core.py` | 两种导纳共用的输入整形、二阶积分、SE(3) 目标和边界 / input conditioning, second-order integration, SE(3) targets, and bounds shared by admittance modes |
| `armbycontroller/admittance/zero_force.py` | Nero 优先的弱保持、阻尼、粘/滑抗漂移柔顺零力 / Nero-first anti-drift soft zero force with weak holding, damping, and stick/slip resistance |
| `armbycontroller/admittance/resistive.py` | 带正阻尼和回中刚度的阻力导纳 / resistive admittance with positive damping and restoring stiffness |
| `armbycontroller/admittance/controller.py` | 导纳 SE(3) 目标到旋量 IK/planned-position 的 controller adapter / controller adapter from admittance SE(3) target to screw-IK planned position |
| `armbycontroller/admittance/__init__.py` | 两种导纳模式的工厂和稳定导出面 / mode factory and stable admittance exports |
| `armbycontroller/admittance/README_ZH_EN.md` | 两种公式、坐标顺序和安全语义 / formulas, coordinate order, and safety semantics for both modes |
| `armbycontroller/control/core.py` | `ControlState`、`ControlReference`、`ControlInput`、命令类型、`ControlResult`、`ControllerAdapter` interface、`ControlEngine` 和稳定控制样本 schema / normalized types, controller interface, engine, and stable sample schema |
| `armbycontroller/control/__init__.py` | 控制 module 的稳定导出面 / stable exports for the control module |
| `armbycontroller/experiment/core.py` | `ExperimentRun` 生命周期、指标、sink interface、内存与 JSONL adapter / experiment lifecycle, metrics, sink interface, in-memory and JSONL adapters |
| `armbycontroller/experiment/__init__.py` | 实验 module 的稳定导出面 / stable exports for the experiment module |
| `test/test_control_interface.py` | 所有 controller adapter 共用的输入/输出、限幅和 schema 契约 / shared input/output, limit, and schema contracts |
| `test/test_cartesian_common.py` | 两种控制共同依赖的几何 Jacobian、SE(3) 和虚功双向映射契约 / shared geometric-Jacobian, SE(3), and bidirectional virtual-work contracts |
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
