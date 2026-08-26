# 启动说明 / 起動手順 / Startup Guide

## 中文

进入项目目录：

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
```

启动 Nero：

```bash
./scripts/start_nero.sh nero_mount:=horizontal
```

该指令只在本次启动把 Nero 重力方向设为平置；YAML 默认仍保持 `side`。

启动 Piper-L：

```bash
./scripts/start_piper_l.sh
```

启动时选择键盘：

```text
1：X11（NoMachine 或桌面键盘）
2：本地键盘（/dev/input/eventN）
```

选择 `2` 后，设备提示可输入编号（如 `3`）、事件名（如 `event3`）或完整路径
（如 `/dev/input/event3`）。

直接按回车默认选择 X11。两个封装脚本都会自动配置 `can0`、检测固件并显式请求
解除电子急停，但不会自动回零；运行任一脚本前必须先检查故障原因、支撑机械臂并
清空周围环境。直接 `ros2 launch` 的参数默认仍为
`reset_emergency_stop_on_start:=false`。编号、事件名和完整路径都会归一化为唯一的
字符串设备参数；evdev 断连后节点会清零并持续发布全部按键释放状态。

### 键盘控制

| 按键 | 功能 |
| --- | --- |
| `1`～`7` | 选择关节；Piper-L 只使用 `1`～`6` |
| `A` / `D` | 关节模式下减小/增大关节角度 |
| `P` | 切换关节模式和笛卡尔模式 |
| `W` / `S` | 笛卡尔模式下沿 X 轴移动 |
| `A` / `D` | 笛卡尔模式下沿 Y 轴移动 |
| `Z` / `X` | 笛卡尔模式下沿 Z 轴移动 |
| 方向键 | 调整末端方向 |
| `PageUp` / `PageDown` | 末端左右倾斜 |
| `I` | 开关阻抗控制 |
| `O` | 开关导纳控制 |
| `H` | 开关混合控制 |
| `SPACE` | 机械臂回零 |
| `E` | 电子急停 |

### 阻抗和导纳

- 阻抗控制（`I`）：机械臂像“弹簧＋阻尼器”。受到外力时可以偏移，松手后向
  开启时记住的姿态恢复。刚度越大越难推动，阻尼越大越平稳。
- 导纳控制（`O`）：估算外力先生成末端速度，经 PoE 旋量 Jacobian 和工具几何
  Jacobian 的受限加权 DLS 转成关节速度，再由
  低增益 MIT 跟踪。每周期从实测关节位置重新锚定；参考关节速度上限为
  `1.0 rad/s`，共享重力补偿已开启，估算总力矩上限为 `8 N·m`，变化率上限为
  `20 N·m/s`。Nero 实测关节速度超过 `2.5 rad/s` 连续三个控制周期，或单周期
  超过 `2.8 rad/s`，触发电子急停；Piper-L 的持续/单周期阈值分别为
  `1.5/2.5 rad/s`。力数据超过 `0.10 s` 未更新时，导纳/混合退出到实测位置保持。
- `I`、`O` 和 `H` 不能同时开启。交互模式之间切换时，控制器严格执行
  `当前模式 -> 普通 planned-position 模式 -> 目标模式`；如果回到普通模式失败，
  则拒绝进入目标模式。再次按下当前模式的按键会回到普通模式。
- 阻抗反馈源时间戳停止推进超过 `0.10 s` 时停止 MIT，并仅在缓存位置的
  `abs(qdot)*age <= 0.03 rad`、MOVE_J 得到新状态确认且持位命令成功时回到普通
  模式；任何条件失败都会触发电子急停。

按 `Ctrl-C` 默认只断开本控制节点，不发送失能命令，以便远程控制器接管。需要
退出时明确失能可追加 `disable_arm_on_shutdown:=true`。

---

## 日本語

プロジェクトディレクトリへ移動します。

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
```

Nero を起動します。

```bash
./scripts/start_nero.sh nero_mount:=horizontal
```

この引数は今回の起動だけ Nero の重力方向を水平設置へ変更します。YAML の
既定値は `side` のままです。

Piper-L を起動します。

```bash
./scripts/start_piper_l.sh
```

起動時にキーボード入力を選択します。

```text
1：X11（NoMachine またはデスクトップキーボード）
2：ローカルキーボード（/dev/input/eventN）
```

`2` を選択した後、デバイス番号（例：`3`）、イベント名（例：`event3`）、
または完全なパス（例：`/dev/input/event3`）を入力できます。

何も入力せず Enter を押すと X11 が選択されます。両ラッパースクリプトは
`can0` の設定、ファームウェア検出、および電子非常停止の解除要求を明示的に
行いますが、自動ゼロ復帰は行いません。どちらを実行する前にも原因、支持、周囲を
確認してください。直接 `ros2 launch` した場合の既定値は
`reset_emergency_stop_on_start:=false` のままです。番号、イベント名、完全パスは
一つの文字列デバイス引数へ正規化され、evdev 切断時は全キーの解放状態を継続して
配信します。

### キーボード操作

| キー | 機能 |
| --- | --- |
| `1`～`7` | 関節を選択。Piper-L は `1`～`6` のみ使用 |
| `A` / `D` | 関節モードで選択関節の角度を減少／増加 |
| `P` | 関節モードと直交座標モードを切り替え |
| `W` / `S` | 直交座標モードで X 軸方向へ移動 |
| `A` / `D` | 直交座標モードで Y 軸方向へ移動 |
| `Z` / `X` | 直交座標モードで Z 軸方向へ移動 |
| 矢印キー | エンドエフェクタの向きを調整 |
| `PageUp` / `PageDown` | エンドエフェクタを左右に傾ける |
| `I` | インピーダンス制御のオン／オフ |
| `O` | アドミッタンス制御のオン／オフ |
| `H` | ハイブリッド制御のオン／オフ |
| `SPACE` | アームをゼロ位置へ戻す |
| `E` | 電子非常停止 |

### インピーダンス制御とアドミッタンス制御

- インピーダンス制御（`I`）：アームを「ばね＋ダンパ」のように動かします。
  外力で変位し、手を離すと開始時に記憶した姿勢へ戻ります。剛性が高いほど
  動かしにくく、減衰が高いほど動きが安定します。
- アドミッタンス制御（`O`）：推定外力から手先速度を生成し、PoE スクリュー
  Jacobian から工具幾何 Jacobian を構成した制限付き重み付き DLS で
  関節速度へ変換して低ゲイン MIT で追従します。各周期で実測関節位置へ
  再アンカーします。関節参照速度上限は `1.0 rad/s`、推定総トルク上限は
  `8 N·m`、変化率上限は `20 N·m/s` です。Nero の実測関節速度が
  `2.5 rad/s` を3制御周期連続で超えるか、1周期でも `2.8 rad/s` を超えると
  電子非常停止します。Piper-L の連続/単周期しきい値はそれぞれ
  `1.5/2.5 rad/s` です。力データが `0.10 s` 更新されない場合は通常保持へ戻ります。
- `I`、`O`、`H` は同時に有効化できません。各相互作用モードを
  切り替えるときは、必ず `現在のモード -> 通常 planned-position モード ->
  目的のモード` の順で遷移します。通常モードへの復帰に失敗した場合、目的の
  モードには入りません。現在のモードのキーをもう一度押すと通常モードへ戻ります。
- インピーダンスのフィードバック時刻が `0.10 s` 更新されない場合は MIT を
  停止します。キャッシュ位置の `abs(qdot)*age <= 0.03 rad`、MOVE_J の新しい
  状態確認、保持指令の成功がすべて成立した場合だけ通常モードへ戻り、失敗時は
  電子非常停止します。

`Ctrl-C` では既定で、この制御ノードだけを切断し、外部コントローラへの引継ぎ
のため無効化コマンドは送りません。終了時に明示的に無効化する場合は
`disable_arm_on_shutdown:=true` を追加します。

---

## English

Open the project directory:

```bash
cd /home/yang/demo_ws/src/agxarm_control_by_gamecontroller
```

Start Nero:

```bash
./scripts/start_nero.sh nero_mount:=horizontal
```

This overrides Nero's gravity direction for this launch only; the YAML default
remains `side`.

Start Piper-L:

```bash
./scripts/start_piper_l.sh
```

Select the keyboard input during startup:

```text
1: X11 (NoMachine or desktop keyboard)
2: Local keyboard (/dev/input/eventN)
```

After selecting `2`, enter an event number such as `3`, an event name such as
`event3`, or a full path such as `/dev/input/event3`.

Press Enter to select X11 by default. Both wrapper scripts configure `can0`,
detect firmware, and explicitly request an electronic-stop reset, but they do
not move home automatically. Inspect the fault, support the arm, and clear the
workspace before either wrapper runs. Direct `ros2 launch` retains
`reset_emergency_stop_on_start:=false`. Event numbers, names, and full paths are
normalized to one string device argument; after an evdev disconnect the reader
continues publishing an all-keys-released state.

### Keyboard controls

| Key | Function |
| --- | --- |
| `1`–`7` | Select a joint; Piper-L only uses `1`–`6` |
| `A` / `D` | Decrease/increase the selected joint angle in joint mode |
| `P` | Switch between joint and Cartesian modes |
| `W` / `S` | Move along the X axis in Cartesian mode |
| `A` / `D` | Move along the Y axis in Cartesian mode |
| `Z` / `X` | Move along the Z axis in Cartesian mode |
| Arrow keys | Adjust the end-effector orientation |
| `PageUp` / `PageDown` | Tilt the end effector left/right |
| `I` | Toggle impedance control |
| `O` | Toggle admittance control |
| `H` | Toggle hybrid control |
| `SPACE` | Return the arm to zero |
| `E` | Electronic emergency stop |

### Impedance and admittance

- Impedance control (`I`) makes the arm behave like a spring and damper. An
  external force can displace it, and it returns toward the pose captured when
  the mode was enabled. Higher stiffness makes it harder to push; higher
  damping makes the response steadier.
- Admittance control (`O`) turns estimated force into tool velocity, obtains a
  tool geometric Jacobian from the PoE screw Jacobian, maps it to bounded joint
  velocity with weighted DLS, and tracks it with low-gain MIT. Shared gravity
  compensation is enabled. The joint
  reference is reanchored to measured position every cycle. Reference joint
  speed is capped at `1.0 rad/s`, estimated total torque at `8 N·m`, and its
  rate at `20 N·m/s`. On Nero, measured joint speed above `2.5 rad/s` for three
  consecutive cycles, or above `2.8 rad/s` once, triggers the electronic stop.
  Piper-L uses `1.5/2.5 rad/s` sustained/immediate thresholds. A wrench stream
  gap above `0.10 s` returns admittance/hybrid to measured-position hold.
- `I`, `O`, and `H` cannot be enabled together. Switching interaction modes
  strictly follows `current mode -> normal planned-position mode ->
  target mode`; failure to restore normal mode rejects the target transition.
  Press the active mode key again to return to normal mode.
- If an impedance feedback timestamp stops advancing for `0.10 s`, MIT stops.
  Normal mode is committed only when cached-position uncertainty satisfies
  `abs(qdot)*age <= 0.03 rad`, a fresh status confirms MOVE_J, and the hold
  command succeeds; any failed condition triggers the electronic stop.

By default, `Ctrl-C` disconnects this controller without sending `disable()`,
allowing an external controller to take over. Add
`disable_arm_on_shutdown:=true` when shutdown must explicitly disable the arm.
