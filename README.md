# armbycontroller

ROS 2 control for AGX Nero, Piper-L, and a Revo2 hand. Nero and Piper-L share
the same controller, keyboard topic, key indices, IK core, and launch files.

## Project layout

- `armbycontroller/`: controllers, kinematics/dynamics, and backend adapters
- `launch/`: keyboard, RViz pose, and backend-integrated launch entries
- `agx_arm_urdf/`: bundled Nero, Piper-L, and Revo2 model assets
- `test/`: functional and regression tests
- [`docs/architecture.md`](docs/architecture.md): module responsibilities and
  runtime paths
- [`docs/phone-remotation-api.md`](docs/phone-remotation-api.md): HTTP/SSE
  backend contract and Motion Link compatibility

## Build

```bash
cd /home/yang/demo_ws
python3 -m pip install "modern_robotics>=1.1.1"
colcon build --packages-up-to armbycontroller
source install/setup.bash
```

The controller uses the PoE, analytic Jacobian, and RNEA implementation
validated in `nero_screw_dynamics`. Nero URDF/Xacro files are resolved from
that package first. Piper-L remains supported through the model bundled in
`armbycontroller`. Both models use the same screw-theory IK/FK and
inverse-dynamics module; `pytracik` is no longer required.

## Unified keyboard control

Both arms use `/arm_keyboard_state` and exactly the same keys:

- `1` ... `7`: select joint; Piper-L ignores `7` because it has six joints
- Joint mode: `A/D` decreases/increases the selected joint
- `P`: switch between joint mode and Cartesian IK mode
- `I`: switch between planned position control and Cartesian impedance
- IK mode: `W/S` = `+X/-X`, `A/D` = `+Y/-Y`, `Z/X` = `+Z/-Z`
- IK mode: arrows point the end effector up/down/left/right
- IK mode: `PageUp/PageDown` tilt the end effector left/right
- `SPACE`: all joints return to zero
- `E`: electronic emergency stop

For keyboard Cartesian impedance, wait for startup zeroing, press `I`, then
press `P` to enter IK. `W/S/A/D/Z/X`, arrows, and PageUp/PageDown update the
desired tool pose. The target is still checked by IK for workspace and joint
limits, but IK joint positions are used only as the redundant-arm posture
reference. In joint input mode, the selected joint goal is converted to a tool
pose by FK. All FK, IK, and Jacobian calculations use the same PoE screw model;
there is no classical-IK fallback. `P` and `I` are independent, so either order
works.

The 100 Hz controller evaluates
`tau = Jg.T @ (Kx * pose_error - Dx * tool_twist) + C(q,dq)dq + g(q)`.
Pose error, twist, stiffness, and damping use
`[rx, ry, rz, x, y, z]` base-frame order. The defaults are
`cartesian_stiffness=[4,4,4,80,80,80]` and
`cartesian_damping=[1.5,1.5,1.5,12,12,12]`; rotational entries have
N·m/rad and N·m·s/rad units, while translational entries have N/m and N·s/m
units. Nero's redundant seventh axis also receives an exact-nullspace posture
term configured by `cartesian_nullspace_stiffness` and
`cartesian_nullspace_damping`. Piper-L has no nominal kinematic null space.

The SDK MIT interface is used only as a joint-torque transport. Every command
sets native MIT `kp=0`, `kd=0`, holds `p_des` at measured `q`, and sends the
computed torque through `t_ff`, so no joint-space impedance is superimposed.
Legacy `mit_kp` and `mit_kd` parameters are accepted but intentionally ignored.
Pressing `I` requires complete position/velocity feedback and a valid dynamics
result, then captures the current tool pose for a zero-error takeover.

The dynamics term uses measured `q/dq`, zero joint acceleration, and the
selected unmodified Nero or Piper-L URDF. Piper-L uses its gripper model and
Nero uses its Revo2 left-hand model; accessory joints remain at their URDF zero
positions. `mit_feedforward` remains an optional residual calibration bias.
`mit_gravity_scale` scales model compensation, and
`mit_gravity_torque_limit` is retained as the compatibility name for the hard
limit on the final Cartesian-impedance joint torque (default ±10 N·m). This is
a commanded torque limit, not measured contact-force feedback. The default
gravity vector is `[0, 0, -9.80665]` in `base_link`.

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

# Start directly in Cartesian impedance mode with impedance_enabled:=true.
```

Dry run:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero execute_motion:=false
```

The controller defaults to 100 Hz IK/control scheduling. Per-tick keyboard
increments are scaled so the original Cartesian, orientation, and joint jog
speeds are preserved at the higher rate. Planned mode retains 20 percent speed
and 1 rad/s² maximum joint acceleration. IK keeps the tool
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

## Phone control seam

The versioned HTTP command API, SSE feedback stream, authentication rules, and
launch profiles for backend integration are documented in
[`docs/phone-remotation-api.md`](docs/phone-remotation-api.md).
The existing Motion Link WebSocket bridge remains the default command
transport; use `backend_transport:=http` when the backend calls the HTTP API
directly. The launch file gives command ownership to only the selected
transport unless `backend_transport:=both` is explicitly requested.

`phoneremotation` should publish `geometry_msgs/msg/PoseStamped` to the same
stable target interface used by terminal control:

- Nero: `/nero/target_pose`
- Piper-L: `/piper_l/target_pose`

Use `base_link` as `header.frame_id`. Feedback is available on
`/<model>/current_pose` and `/<model>/ik_status`. This keeps phone transport
outside the robot model and hardware adapters.

The separate Phone Remotation project provides the phone, desktop controller,
and robot WebSocket roles. From that project's directory, start the web service
and then select the arm and end effector in the desktop page:

```bash
export AGILEX_CAN_INTERFACE=can0
./motion-link-control.sh
```

Start performs a read-only probe for complete joint feedback. A detected arm
uses the real controller; otherwise the same model starts in simulation.
Both paths publish `/joint_states` at 30 Hz and report those joints to the
browser. Model and end-effector selectors are locked while control is running.

Once started, the page expands a virtual gamepad and joint keys 1–7. It sends
the same 23-key `/arm_keyboard_state` protocol as the native keyboard reader,
including joint/IK (`P`), planned/MIT (`I`), home, and emergency stop. The
bridge releases all keys on browser or WebSocket disconnect. Phone pose input
continues to use `/<model>/target_pose`; an active virtual key temporarily
takes priority and clears the phone reference before phone control resumes.

The first valid phone sample captures both the current phone orientation and
the current tool pose as zero. Commands stop when samples are older than
250 ms, orientation is limited to 0.6 rad from the captured pose, and phone
acceleration is deliberately not integrated into position because its drift
would create unsafe Cartesian motion.

## Revo2 hand

```bash
ros2 run armbycontroller hand_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=v111 -p execute_motion:=true
```
