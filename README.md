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
the IK joint target; the 100 Hz MIT loop tracks that target with impedance and
fixed feed-forward torque. `P` and `I` are independent, so either order works.

MIT impedance uses the SDK equation
`τ_ref = Kp(q_des-q) + Kd(dq_des-dq) + τ_ff`. It therefore changes the joint
control backend, while `P` only changes how the desired joint target is
generated. Piper-L uses independent per-joint gains:
`Kp=[0.2, 0.4, 0.3, 0.4, 0.2, 0.4]` and
minimum `Kd=[0.12, 0.18, 0.15, 0.15, 0.12, 0.15]`. Nero retains
`Kp=1.0, Kd=0.2`.
During normal MIT operation, each joint increases damping smoothly with its
measured speed:
`D(v)=Dmin+(Dmax-Dmin)v²/(vc²+v²)`. The controller converts
`τd,max*tanh(D(v)*v/τd,max)` to an equivalent dynamic `Kd`, so the motor's
native MIT damping term has a smooth torque ceiling instead of growing without
limit. Launch defaults are `Dmax=0.6`, `vc=0.3 rad/s`, and
`τd,max=1.0 N·m` per joint. Tune them with `mit_kd_max`,
`mit_damping_transition_velocity`, and `mit_damping_torque_limit`; each accepts
one value for all joints or a complete per-joint list. `Kp` never changes with
speed. If motor velocity feedback is unavailable, damping safely falls back to
`Dmin`.
Feed-forward defaults to zero and the refresh rate is 100 Hz. Pressing `I`
captures the current joints and starts with higher `Kp` to support the arm,
then uses a smoothstep ramp to reach the configured soft `Kp` in 0.5 seconds.
Piper-L takeover `Kp` is 10 on joints 2/3/5 and 6 on joints 1/4/6.
Only stiffness uses this takeover: speed-adaptive damping and its torque limit
apply from the first frame, and `Kd` is never switched to an uncapped high value.

Run Nero:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto
```

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

The controller defaults to a 5 mm Cartesian step, 0.005 rad joint step,
20 percent speed, and 1 rad/s² maximum joint acceleration. IK keeps the tool
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
