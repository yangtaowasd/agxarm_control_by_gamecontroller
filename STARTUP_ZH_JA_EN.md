# 启动说明 / 起動手順 / Startup Guide

## 中文

进入项目目录：

```bash
cd /home/techshare/demo_ws/src/agxarm_control_by_gamecontroller
```

启动 Nero：

```bash
./scripts/start_nero.sh
```

启动 Piper-L：

```bash
./scripts/start_piper_l.sh
```

启动时选择键盘：

```text
1：X11（NoMachine 或桌面键盘）
2：本地键盘（/dev/input/eventN）
```

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
- 导纳控制（`O`）：机械臂根据估算到的外力主动生成运动目标，适合拖动和柔顺
  操作。虚拟质量越大反应越慢，阻尼越大越不容易晃动。
- `I` 和 `O` 不能同时开启；进入一个模式会退出另一个模式。

按 `Ctrl-C` 停止程序。

---

## 日本語

プロジェクトディレクトリへ移動します。

```bash
cd /home/techshare/demo_ws/src/agxarm_control_by_gamecontroller
```

Nero を起動します。

```bash
./scripts/start_nero.sh
```

Piper-L を起動します。

```bash
./scripts/start_piper_l.sh
```

起動時にキーボード入力を選択します。

```text
1：X11（NoMachine またはデスクトップキーボード）
2：ローカルキーボード（/dev/input/eventN）
```

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
- アドミッタンス制御（`O`）：推定外力から新しい移動目標を生成します。手で
  押して動かす操作や柔軟な動作に適しています。仮想質量が大きいほど反応が
  遅く、減衰が大きいほど振動しにくくなります。
- `I` と `O` は同時に有効化できません。一方へ入ると他方は終了します。

`Ctrl-C` でプログラムを停止します。

---

## English

Open the project directory:

```bash
cd /home/techshare/demo_ws/src/agxarm_control_by_gamecontroller
```

Start Nero:

```bash
./scripts/start_nero.sh
```

Start Piper-L:

```bash
./scripts/start_piper_l.sh
```

Select the keyboard input during startup:

```text
1: X11 (NoMachine or desktop keyboard)
2: Local keyboard (/dev/input/eventN)
```

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
- Admittance control (`O`) generates a motion target from the estimated external
  force. It is suitable for hand-guiding and compliant motion. Higher virtual
  mass slows the response; higher damping reduces oscillation.
- `I` and `O` cannot be enabled together. Entering one mode exits the other.

Press `Ctrl-C` to stop the program.
