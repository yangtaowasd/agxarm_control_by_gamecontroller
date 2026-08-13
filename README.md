# armbycontroller

ROS 2 control for AGX Nero, Piper-L, and a Revo2 hand. Nero and Piper-L share
the same controller, keyboard topic, key indices, IK core, and launch files.

## Cartesian impedance development branch

`feature/cartesian-impedance-step-by-step` starts from the validated joint MIT
impedance baseline and adds Cartesian impedance in independently tested stages.
The formula core is now connected to the existing keyboard state machine and
the AGX MIT/CAN adapter. `impedance_backend:=cartesian` is the default;
`impedance_backend:=joint` retains the previous joint MIT controller for
comparison.

Task vectors use `[angular; linear] = [rx, ry, rz, x, y, z]`. Because the PoE
model returns a Modern Robotics space Jacobian, it is first converted to the
tool-origin geometric Jacobian:

```text
Jg = [Jw; Jv - [p]x Jw]
e  = [Log(Rd R^T)^vee; pd - p]
xd = Jg qdot
Fc = Kx e + Dx (xd_des - xd)
tau_task = Jg^T Fc
tau_0 = K0 (q0-q) + D0 (dq0-dq)
N_tau = I - Jg^T (Jg M^-1 Jg^T)^+ Jg M^-1
tau_null = N_tau tau_0                 # Nero 7-axis only
tau_model = C(q,qdot)qdot + g(q)
tau_cmd = tau_task + tau_null + tau_model
```

The rotational stiffness about base-frame Z is independently configurable as
`cartesian_impedance_base_z_rotation_stiffness` (default `4.0 N.m/rad`). The
shared `cartesian_impedance_rotation_stiffness` value (default `0.4 N.m/rad`)
continues to set base-frame X/Y rotation. This strengthens the principal J1
return direction without raising all wrist rotation axes together.

Nero's seventh joint is controlled by a dynamically consistent nullspace
impedance. Its defaults are `0.4 N.m/rad` stiffness and `0.1 N.m.s/rad`
damping. The torque projector uses the URDF mass matrix and keeps
`Jg M^-1 tau_null = 0` to numerical tolerance, so redundant-joint restoration
does not intentionally move the Cartesian task. This term is disabled for the
six-axis Piper-L.

`tau_cmd` is the immediate, stateless equation result. The MIT adapter applies
only a per-joint absolute limit (±8 N·m by default, including model support).
It does not apply a previous-cycle torque-rate limiter (`delta_tau` or
`delta_tau_max`).

The local joint-space equivalence is the full matrix relation
`Kq=Jg.T Kx Jg`, `Dq=Jg.T Dx Jg`. Off-diagonal coupling is retained; this
stage does not replace it with diagonal MIT gains. For a nonsingular six-axis
pose only, the unique reverse relation is
`Kx=Jg^-T Kq Jg^-1`, `Dx=Jg^-T Dq Jg^-1`; redundant or singular cases are
explicitly rejected instead of silently using a pseudoinverse. See
`PROJECT_STUDY_GUIDE_ZH_EN.md` for the bilingual derivation and staged plan.

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
- `I`: switch between planned position control and the selected MIT impedance
  backend (Cartesian by default)
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

The Cartesian backend uses the SDK equation
`τ_ref=Kp(q_des-q)+Kd(dq_des-dq)+τ_ff` with `Kp=0`, `Kd=0`,
`p_des=q_measured`, `v_des=0`, and
`τ_ff=clip(Jg.T Fc+tau_null+C(q,dq)dq+g(q), ±τ_limit)`. Thus the firmware does
not add a second joint spring or damper. `tau_null` is nonzero only for Nero.
A continuous joint reference supplies the target pose, target twist, and Nero
nullspace posture through FK/Jacobian; this reference continuity is not a
torque-rate limiter.

Piper-L kinematics and inverse dynamics both use
`piper_l_with_gripper_description.xacro`. Accessory joints are fixed at their
URDF zero positions, their mass/inertia contributes to `C*dq+g`, and only the
six arm joints receive MIT commands. The controlled task point remains
`link6`; it is not the fingertip contact point. Complete measured `q/dq` and a
valid URDF model are required before entry.

For comparison, `impedance_backend:=joint` retains the previous native MIT
joint impedance and its `mit_kp`, `mit_kd`, `mit_feedforward`, and
`mit_gravity_*` parameters.

Run Nero:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto nero_mount:=horizontal \
  impedance_backend:=cartesian impedance_enabled:=false \
  cartesian_impedance_nullspace_stiffness:=0.4 \
  cartesian_impedance_nullspace_damping:=0.1 \
  move_home_on_start:=false reset_emergency_stop_on_start:=true
```

Nero requires an explicit mounting choice. Use `nero_mount:=horizontal` when
the base is normally mounted on a horizontal surface (`gravity_vector=[0,0,-g]`),
or `nero_mount:=side` for the project's `pitch=-90°` side-mount convention
(`gravity_vector=[-g,0,0]`). Left/right side-mount yaw does not change gravity
in `base_link`. Omitting the choice intentionally stops launch.
Nero kinematics and dynamics use `nero_with_left_revo2_description.xacro` by
default. The fixed Revo2 mass/inertia is included while MIT commands are sent
only to joints 1 through 7. Press `I` to capture the current tool pose and
nullspace posture before enabling Cartesian MIT.

Run Piper-L with the same keys:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto \
  impedance_backend:=cartesian \
  cartesian_impedance_base_z_rotation_stiffness:=4.0

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

## Phone control seam

`phoneremotation` should publish `geometry_msgs/msg/PoseStamped` to the same
stable target interface used by terminal control:

- Nero: `/nero/target_pose`
- Piper-L: `/piper_l/target_pose`

Use `base_link` as `header.frame_id`. Feedback is available on
`/<model>/current_pose` and `/<model>/ik_status`. This keeps phone transport
outside the robot model and hardware adapters.

The Motion Link project at
`/media/yang/Windows/Users/yang.tao/Desktop/demo/phone remotation` already
provides the required phone and robot WebSocket roles. Configure the real
hardware in the backend environment, then start the web service:

```bash
cd "/media/yang/Windows/Users/yang.tao/Desktop/demo/phone remotation"
export AGILEX_ROBOT_MODEL=nero
export AGILEX_END_EFFECTOR=revo2
export AGILEX_CAN_INTERFACE=can0
./motion-link-control.sh
```

NERO/PIPER-L and none/Gripper/Revo2 selectors affect only the independent
simulation preview. The desktop's separate real-hardware button uses the
immutable backend configuration above; the browser cannot submit a model,
tool, or CAN interface. Starting real hardware locks preview configuration,
and stopping it unlocks preview switching.

The bridge reports live arm joints back to Motion Link. `simulate` lets
phone-relative orientation command the simulated arm while holding the current
tool position. Real hardware requires explicit confirmation:

```bash
./motion-link-control.sh hardware --confirm-move
```

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
