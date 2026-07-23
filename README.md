# armbycontroller

ROS 2 control for AGX Nero, Piper-L, and a Revo2 hand. Nero and Piper-L share
the same controller, keyboard topic, key indices, IK core, and launch files.

## Build

```bash
cd /home/yang/demo_ws
colcon build --packages-select armbycontroller
source install/setup.bash
```

Install `pytracik` if needed:

```bash
sudo apt install libboost-all-dev libeigen3-dev liborocos-kdl-dev \
  libnlopt-dev libnlopt-cxx-dev
python3 -m pip install pytracik
```

## Unified keyboard control

Both arms use `/arm_keyboard_state` and exactly the same keys:

- `1` ... `7`: select joint; Piper-L ignores `7` because it has six joints
- Joint mode: `A/D` decreases/increases the selected joint
- `P`: switch between joint mode and Cartesian IK mode
- `I`: switch between planned position control and MIT joint impedance
- IK mode: `W/S` = `+X/-X`, `A/D` = `+Y/-Y`, `Z/X` = `+Z/-Z`
- IK mode: arrows point the end effector up/down/left/right
- IK mode: `PageUp/PageDown` tilt the end effector left/right
- `SPACE`: all joints return to zero
- `E`: electronic emergency stop

For keyboard IK through MIT, wait for startup zeroing, press `I` to enter MIT,
then press `P` to enter IK. `W/S/A/D/Z/X`, arrows, and PageUp/PageDown update
the IK joint target. Both IK target generation and the MIT command loop run at
100 Hz. A jerk-limited joint trajectory supplies continuous `q_des`, `dq_des`,
and `ddq_des`; `P` and `I` are independent, so either order works.

MIT impedance uses the SDK equation
`τ_ref = Kp(q_des-q) + Kd(dq_des-dq) + τ_ff`. It therefore changes the joint
control backend, while `P` only changes how the desired joint target is
generated. Piper-L uses independent per-joint gains:
`Kp=[0.3, 0.5, 0.5, 0.5, 1.0, 0.3]` and `Kd=0.01`. Nero retains
`Kp=1.0, Kd=0.2`.
The configured `mit_kp` and `mit_kd` values are sent unchanged on every MIT
command. There is no takeover stiffness, gain ramp, or speed-adaptive damping.
The residual feed-forward bias defaults to zero and the refresh rate is 100 Hz.
Pressing `I` first requires complete position/velocity feedback and a valid
inverse-dynamics result. It captures the current joints and applies full gravity
support on the first MIT frame while leaving Kp/Kd unchanged.

MIT mode reads the bundled, unmodified Nero or Piper-L model and computes full
rigid-body inverse dynamics at every tick:
`tau_ff = M(q) ddq_des + C(q,dq_des) dq_des + g(q) + tau_bias`.
It uses measured arm positions for `q` and the continuous trajectory references
for velocity and acceleration. The calculation uses recursive Newton-Euler
dynamics and includes the URDF link inertias; it does not estimate joint
friction or unmodelled cable forces.
Piper-L uses its gripper model and Nero uses its Revo2 left-hand model. The
accessory joints are evaluated at their URDF zero positions and only the arm's
6/7 joints receive MIT commands. Model torque is active from the first MIT
frame. After adding it, the controller estimates the native MIT PD term from
measured `q/dq` and adjusts `t_ff` so the combined reference is as close as
possible to the per-joint ±10 N·m limit. The `t_ff` channel is itself kept
inside ±10 N·m. If the PD term alone is too large to counteract within that
range, the controller warns and applies the maximum available cancellation.
This is a command-level estimate, not measured contact torque feedback.
`mit_feedforward` is an additional residual calibration bias and defaults to
zero. Set `mit_gravity_scale` within `[0, 1]` to reduce its contribution or tune
`mit_gravity_torque_limit`. Disabling the model also disables entry into MIT on
real hardware. The default gravity
vector is `[0, 0, -9.80665]` in `base_link`, assuming an upright base.
The `mit_gravity_*` parameter names are retained for launch-file compatibility:
scale applies to model torque, while the limit applies to both `t_ff` and the
estimated combined MIT reference.

Run Nero:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto nero_mount:=horizontal
```

Nero requires an explicit mounting choice. Use `nero_mount:=horizontal` when
the base is normally mounted on a horizontal surface (`gravity_vector=[0,0,-g]`),
or `nero_mount:=side` for the project's `pitch=-90°` side-mount convention
(`gravity_vector=[-g,0,0]`). Left/right side-mount yaw does not change gravity
in `base_link`. Omitting the choice intentionally stops launch.

Run Piper-L with the same keys:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto

# Start directly in MIT impedance mode by adding impedance_enabled:=true.
```

Dry run:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero execute_motion:=false
```

The controller defaults to 100 Hz IK/control scheduling. Per-tick keyboard
increments are scaled so the original Cartesian, orientation, and joint jog
speeds are preserved at the higher rate. The MIT reference generator defaults
to 0.5 rad/s velocity, 1 rad/s² acceleration, and 5 rad/s³ jerk limits, exposed
as `mit_trajectory_max_velocity`, `mit_trajectory_max_acceleration`, and
`mit_trajectory_max_jerk`. Planned mode retains 20 percent speed and 1 rad/s²
maximum joint acceleration. IK keeps the tool
local `+Z` axis pointing toward `base_link -Z`; rotation around that axis is
free. It retains ten verified states and pauses for two seconds after recovery.
Acceleration limits are written and verified one joint at a time because the
Piper S-V1.8-8 batch (`joint_index=255`) ACK path is unreliable.
The controller also waits for `CAN_CTRL/MOVE_J` feedback before sending the
first startup target so Piper does not discard it during a mode transition.
Real-hardware launch explicitly resets a latched electronic stop, enables the
joints, and forces them to zero strictly in joint order from 1 through the
final joint. Each joint must reach zero before the next command is sent. Use
`reset_emergency_stop_on_start:=false move_home_on_start:=false` to preserve
the current state instead. Only use automatic homing when the complete path to
zero is clear. `firmware:=auto` selects
`v112` for Nero and `v188` for Piper-L; an explicit firmware argument still
overrides it.
If the arm reports a latched electronic emergency stop, normal startup refuses
to move. After physically checking the arm, explicitly add
`reset_emergency_stop_on_start:=true` to reset the controller before enabling.

## RViz pose simulation

```bash
# Nero
ros2 launch armbycontroller pose_rviz.launch.py robot_model:=nero

# Piper-L
ros2 launch armbycontroller pose_rviz.launch.py robot_model:=piper_l
```

In another interactive terminal:

```bash
ros2 run armbycontroller terminal_teleop.py --ros-args \
  -p topic_prefix:=/nero -p step:=0.01

# Use /piper_l instead when running Piper-L.
```

Terminal keys use the same Cartesian directions and orientation keys:
`W/S`, `A/D`, `Z/X`, arrows, and `PageUp/PageDown`. `R` resets the complete
pose to feedback and `Q` quits.

## Standalone pose controller

```bash
ros2 run armbycontroller pose_controller.py --ros-args \
  -p robot_model:=piper_l -p topic_prefix:=/piper_l \
  -p tip_link:=link6 -p firmware:=default \
  -p initial_joint_positions:="[0.0, 1.3939753, -1.0158306, 0.0, 1.2799181, 0.0]" \
  -p robot_min_reach:=0.0 -p robot_max_reach:=0.8738043 \
  -p execute_motion:=true
```

The safe workspace keeps 5 cm inside the minimum reach and 10 cm inside the
maximum reach. The pose controller verifies every IK result using FK before
sending `move_j()`.

## Revo2 hand

```bash
ros2 run armbycontroller hand_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=v111 -p execute_motion:=true
```
