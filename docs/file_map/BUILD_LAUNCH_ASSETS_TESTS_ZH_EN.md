# 构建、启动、资源与测试 / Build, Launch, Assets, and Tests

## 仓库入口 / Repository entry files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `CMakeLists.txt` | C++/Python 安装、节点/资源安装、十一组 pytest，以及限定到源码目录的 style lint 注册 / C++/Python install, node/resource install, eleven pytest groups, and source-scoped style-lint registration |
| `package.xml` | ROS 2 build/runtime/test 依赖；实验 recorder 需要 `std_srvs` / ROS 2 dependencies, including `std_srvs` for experiment recording |
| `resource/agxarm_control_by_gamecontroller` | ament ROS package 索引 marker / ament ROS-package index marker |
| `README.md` | 快速构建与运行入口 / quick build and run entry point |
| `PROJECT_STUDY_GUIDE_ZH_EN.md` | 完整双语目标、公式、架构、安全与学习路径 / complete bilingual goal, equations, architecture, safety, and learning path |
| `CONTEXT.md` | 控制实验领域词汇 / control-experiment domain language |
| `AGENTS.md`, `docs/agents/*.md` | 工程技能的 issue tracker、标签和领域文档消费规则 / engineering-skill tracker, label, and domain-doc rules |

## Launch 文件 / Launch files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `launch/keyboard_control.launch.py` | 主键盘控制、动量观测器和可选实验 recorder；按 `arm_namespace` 隔离 Nero/Piper-L DDS 接口 / main keyboard control, observer, and optional experiment recorder, with Nero/Piper-L DDS isolation through `arm_namespace` |
| `launch/pose_rviz.launch.py` | Nero/Piper-L 运动学 RViz 预览，不是动力学仿真 / kinematic RViz preview, not dynamics simulation |
| `launch/motion_link.launch.py` | 手机 bridge 与位姿控制器的安全默认组合 / safe-default phone bridge plus pose controller |

`arm_namespace` 默认值 `__robot__` 会解析为所选 `robot_model`：Nero 使用
`/nero/...`，Piper-L 使用 `/piper_l/...`。Topic/service 参数均为相对名，因此会
跟随命名空间；显式绝对名覆盖会绕过该隔离。 / The `arm_namespace` default
`__robot__` resolves to the selected `robot_model`: `/nero/...` for Nero and
`/piper_l/...` for Piper-L. Topic/service parameters are relative and follow
the namespace; an explicit absolute-name override bypasses that isolation.

## 配置文件 / Configuration files

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `config/common.yaml` | 两种机械臂共用的控制/观测周期、默认 backend 和固件探测时序 / shared control/observer rates, default backend, and firmware-probe timing |
| `config/nero.yaml` | Nero 独立侧装裸臂、7 轴补偿/阻抗/轨迹、两种导纳、导纳 MIT、J2/J3/J4 混合姿态与速度估计参数 / Nero-only side-mounted bare-arm, seven-axis compensation/impedance/trajectory, both admittance modes, admittance MIT, hybrid J2/J3/J4-posture, and velocity-estimation parameters |
| `config/piper_l.yaml` | Piper-L 独立夹爪、6 轴补偿/阻抗/轨迹、两种导纳与导纳 MIT 参数 / Piper-L-only gripper, six-axis compensation/impedance/trajectory, both admittance modes, and admittance-MIT parameters |

三份参数文件均以 `/**/arm_keyboard_controller` 和
`/**/arm_momentum_observer` 为节点选择器，从而匹配 `/nero`、`/piper_l` 或自定义
命名空间；去掉通配符会使 ROS 2 静默退回代码默认参数。 / All three parameter
files select `/**/arm_keyboard_controller` and `/**/arm_momentum_observer`, so
they match `/nero`, `/piper_l`, or a custom namespace. Removing the wildcard
would silently fall back to code defaults.

## 机器人资源 / Robot assets

| 路径 / Path | 分类责任 / Classified responsibility |
| --- | --- |
| `agx_arm_urdf/nero/urdf/*` | Nero 本体及 gripper/Revo2 固定附件模型 / Nero and fixed accessory variants |
| `agx_arm_urdf/piper_l/urdf/*` | Piper-L 本体及 gripper/Revo2 固定附件模型 / Piper-L and fixed accessory variants |
| `agx_arm_urdf/revo2/urdf/*` | 左右 Revo2 手模型 / left/right Revo2 hand models |
| `agx_arm_urdf/**/meshes/*` | RViz/robot-description 几何资源；不参与控制公式 / visualization geometry; not part of control equations |

Revo2 的 URDF/Xacro 和全部 mesh 是为后续工具配置预留的受支持资产。即使当前
`tool_configuration` 没有引用其中某个 mesh，也不得只依据当前引用可达性删除。
/ All Revo2 URDF/Xacro files and meshes are supported assets reserved for
future tool configurations. A mesh must not be removed solely because the
current `tool_configuration` does not reference it.

这些模型资源由当前 ROS 包安装并由当前包的模型解析器直接使用；Nero 默认启动
不依赖 `nero_screw_dynamics` 或相邻源码仓库。 / These model assets are
installed and resolved directly by the current ROS package; default Nero
startup does not depend on `nero_screw_dynamics` or a neighboring source
repository.

## 测试 / Tests

| 文件 / File | 分类责任 / Classified responsibility |
| --- | --- |
| `test/test_arm_control.py` | 状态机、轨迹、AGX adapter、安全启动和模式互锁集成回归 / state machine, trajectory, AGX adapter, startup safety, and interlock regression |
| `test/test_momentum_observer.py` | 观测器公式、时间戳与 ROS 隔离 / observer formula, timestamp, and ROS isolation |
| `test/test_control_interface.py` | 统一控制器 interface 与三个 adapter / common controller interface and three adapters |
| `test/test_cartesian_common.py` | 阻抗/导纳共用笛卡尔任务几何的独立契约 / independent contracts for shared Cartesian Task Geometry |
| `test/test_experiment.py` | 实验生命周期、sink 与指标 / experiment lifecycle, sinks, and metrics |
| `test/test_hardware_connection.py` | 两阶段连接、版本映射和失败清理；详细语义见硬件连接分类 / two-stage connection, version mapping, and failure cleanup; see the hardware-connection category for semantics |
| `test/test_keyboard_disconnect.py` | FIFO 注入真实 evdev 事件，验证按下后断连会清零并持续发布释放状态 / FIFO-backed real evdev injection proving disconnect clears and continuously releases keys |

`test/test_cartesian_impedance.py` 属于用户指定排除的 `impedance` 主题；
`test/test_cartesian_admittance.py` 验证独立导纳目录的两种公式、模式工厂和安全
边界。 / `test/test_cartesian_impedance.py` belongs to the explicitly excluded
impedance topic; `test/test_cartesian_admittance.py` verifies both formulas,
the mode factory, and safety bounds in the separate admittance package.
