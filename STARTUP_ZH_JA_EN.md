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

直接按回车默认选择 X11。脚本会自动配置 `can0`、检测固件、解除电子急停并使能
机械臂。启动前请释放物理急停，并确保机械臂周围没有人员或障碍物。

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
| `SPACE` | 机械臂回零 |
| `E` | 电子急停 |

### 阻抗和导纳

- 阻抗控制（`I`）：机械臂像“弹簧＋阻尼器”。受到外力时可以偏移，松手后向
  开启时记住的姿态恢复。刚度越大越难推动，阻尼越大越平稳。
- 导纳控制（`O`）：估算外力先生成末端速度，经 PoE 旋量 Jacobian 和工具几何
  Jacobian 的受限加权 DLS 转成关节速度，再由
  低增益 MIT 跟踪。每周期从实测关节位置重新锚定；参考关节速度上限为
  `0.5 rad/s`，共享重力补偿已开启，估算总力矩上限为 `8 N·m`。实测关节速度
  超过 `1.0 rad/s` 连续三个控制周期，或单周期超过 `2.0 rad/s`，触发电子急停。
- `I` 和 `O` 不能同时开启。阻抗与导纳之间切换时，控制器严格执行
  `当前模式 -> 普通 planned-position 模式 -> 目标模式`；如果回到普通模式失败，
  则拒绝进入目标模式。再次按下当前模式的按键会回到普通模式。

按 `Ctrl-C` 停止程序。

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

何も入力せず Enter を押すと X11 が選択されます。スクリプトは `can0` の設定、
ファームウェア検出、電子非常停止の解除、アームの有効化を自動で行います。
起動前に物理非常停止を解除し、アームの周囲に人や障害物がないことを確認して
ください。

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
| `SPACE` | アームをゼロ位置へ戻す |
| `E` | 電子非常停止 |

### インピーダンス制御とアドミッタンス制御

- インピーダンス制御（`I`）：アームを「ばね＋ダンパ」のように動かします。
  外力で変位し、手を離すと開始時に記憶した姿勢へ戻ります。剛性が高いほど
  動かしにくく、減衰が高いほど動きが安定します。
- アドミッタンス制御（`O`）：推定外力から手先速度を生成し、PoE スクリュー
  Jacobian から工具幾何 Jacobian を構成した制限付き重み付き DLS で
  関節速度へ変換して低ゲイン MIT で追従します。各周期で実測関節位置へ
  再アンカーします。関節参照速度上限は `0.5 rad/s`、推定総トルク上限は
  `8 N·m` です。共有重力補償を使用し、実測関節速度が `1.0 rad/s` を3制御周期
  連続で超えるか、1周期でも `2.0 rad/s` を超えると電子非常停止します。
- `I` と `O` は同時に有効化できません。インピーダンスとアドミッタンスを
  切り替えるときは、必ず `現在のモード -> 通常 planned-position モード ->
  目的のモード` の順で遷移します。通常モードへの復帰に失敗した場合、目的の
  モードには入りません。現在のモードのキーをもう一度押すと通常モードへ戻ります。

`Ctrl-C` でプログラムを停止します。

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

Press Enter to select X11 by default. The script automatically configures
`can0`, detects the firmware, resets the electronic emergency stop, and enables
the arm. Before starting, release the physical emergency stop and make sure
there are no people or obstacles around the arm.

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
  speed is capped at `0.5 rad/s` and estimated total torque at `8 N·m`.
  Measured joint speed above `1.0 rad/s` for three consecutive control cycles,
  or above `2.0 rad/s` once, triggers the electronic stop.
- `I` and `O` cannot be enabled together. Switching between impedance and
  admittance strictly follows `current mode -> normal planned-position mode ->
  target mode`; failure to restore normal mode rejects the target transition.
  Press the active mode key again to return to normal mode.

Press `Ctrl-C` to stop the program.
