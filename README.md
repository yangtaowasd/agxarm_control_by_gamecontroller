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
tau_posture = Kp_j (q_ref-q) + Dp_j (dq_ref-dq)
tau_model = model_scale .* (C(q,qdot)qdot + g(q))
tau_cmd = tau_task + tau_null + tau_posture + tau_model
```

The Cartesian-impedance orientation error is an SO(3) logarithmic rotation
vector expressed in the base frame, not an RPY/Euler-angle subtraction.
Translation uses the direct tool-origin difference `pd-p`; this preserves the
physical N/m spring and geometric-Jacobian wrench interpretation.

The numerical screw IK uses the full base-frame SE(3) error and the PoE space
Jacobian instead:

```text
V_error = Log(Td T^-1)^vee
delta_q = Js(q)^# V_error
```

SO(3)/SE(3) logarithms, exponentials, and pose-error construction are
concentrated in `armbycontroller/modeling/lie.py`. Keyboard orientation increments,
Motion Link orientation limiting, URDF RPY conversion, Cartesian orientation
error, and FK verification reuse that module. Quaternion conversion remains
at ROS message seams, while URDF and phone RPY values remain input formats.

The rotational stiffness about base-frame Z is independently configurable as
`cartesian_impedance_base_z_rotation_stiffness`. Piper-L uses its dedicated
`[0.4, 0.4, 4.0] N.m/rad` rotational tuning to strengthen base-Z return
without raising all wrist rotation axes together. Nero deliberately disables
that Piper-L-specific reinforcement and uses isotropic
`[1.9, 1.9, 1.9] N.m/rad` task-space rotation stiffness. Its translation
stiffness is `70 N/m`; rotational and translational damping are respectively
`0.24 N.m.s/rad` and `1.4 N.s/m`.

Nero's seventh joint is controlled by a dynamically consistent nullspace
impedance. Its defaults are `0.4 N.m/rad` stiffness and `0.1 N.m.s/rad`
damping. The torque projector uses the URDF mass matrix and keeps
`Jg M^-1 tau_null = 0` to numerical tolerance, so redundant-joint restoration
does not intentionally move the Cartesian task. This term is disabled for the
six-axis Piper-L.

Pure Cartesian impedance restores tool pose but does not guarantee an
independent spring at every joint. In the recorded Nero configurations, J4 is
effectively absent from the one-dimensional kinematic nullspace, so projected
nullspace gains cannot give it a joint spring. Nero therefore adds a narrowly
configured, unprojected outer-loop posture term on J2/J3/J4. Stiffness is
`[0, 0.5, 0.5, 0.6, 0, 0, 0] N.m/rad` and damping is
`[0, 0.08, 0.08, 0.12, 0, 0, 0] N.m.s/rad`. This hybrid term intentionally
changes the Cartesian task and is disabled by default for Piper-L. It is
computed in software and sent through `tau_ff`; firmware MIT `kp=kd=0`
remains unchanged.

`model_scale` is a scalar or per-joint vector in `[0, 1]`; it is independent
from task/nullspace/posture gains and defaults to `1.0`. `tau_cmd` is the
immediate, stateless equation result. The MIT adapter applies only a per-joint
absolute limit (±8 N·m by default, including model support and posture
torque). It does not apply a previous-cycle torque-rate limiter (`delta_tau`
or `delta_tau_max`).

## Passive momentum observer and Piper-L admittance

The launch file also starts a passive generalized-momentum observer. The
controller reads one cached `q`, `qdot`, and motor-torque sample per 100 Hz
cycle and publishes it on `/arm_dynamics_state` in every backend. Nero v111
and v112 do not expose motor velocity through the current SDK, so `qdot` is a
low-pass filtered finite difference of joint position for those profiles. A
separate process subscribes to that stream and publishes estimated external
joint torque in the `effort` field of `/arm_external_joint_torque`:

```text
p    = M(q) qdot
beta = g(q) - dT/dq
r    = K [p - p(0) - integral(tau_motor - beta + r) dt]
```

Here `beta` is the known URDF dynamics term, not an empirical calibration
bias. Fixed calibration/feedforward bias is zero for both robot profiles.
When URDF compensation is active, the controller now ignores
`mit_feedforward` in that compensation path and sends only the scaled,
absolute-bounded inverse-dynamics result.

The observer never connects to CAN or calls the arm SDK. It updates once for
every timestamped 100 Hz dynamics-state message, using the previous cycle's
measured motor torque for the interval that just elapsed. The default gain is
`momentum_observer_gain:=10.0 1/s`. It is monitor-only: `r` does not alter
impedance torque and cannot trigger an emergency stop. Disable the process
with `momentum_observer_enabled:=false`.

On Piper-L, `O` toggles Cartesian admittance. The observed external joint
torque is mapped to a base-frame wrench by damped least squares, integrated
through `M_a*xdd + D_a*xd + K_a*x = F_ext`, then converted to planned joint
targets by screw IK. `I` and `O` are strictly interlocked: entering impedance
exits admittance first, and entering admittance exits impedance first. Nero
admittance is intentionally not enabled yet.

Because the URDF omits friction, backlash, cable forces, payload error, motor
torque-tracking error, and joint elasticity, the residual is a total model
disturbance estimate rather than a calibrated contact-force measurement.

The local joint-space equivalence is the full matrix relation
`Kq=Jg.T Kx Jg`, `Dq=Jg.T Dx Jg`. Off-diagonal coupling is retained; this
stage does not replace it with diagonal MIT gains. For a nonsingular six-axis
pose only, the unique reverse relation is
`Kx=Jg^-T Kq Jg^-1`, `Dx=Jg^-T Dq Jg^-1`; redundant or singular cases are
explicitly rejected instead of silently using a pseudoinverse. See
`PROJECT_STUDY_GUIDE_ZH_EN.md` for the bilingual derivation and staged plan.

## Unified controller interface and experiments

All three in-tree interaction algorithms now run behind the same ROS/CAN-free
controller seam:

```text
ControlInput(state, reference, wrench, timestamp, period)
    -> ControllerAdapter.reset/step
    -> ControlResult(MIT or planned-position command, diagnostic signals)
```

`joint_impedance`, `cartesian_impedance`, and the current
`cartesian_admittance` are adapters registered with `ControlEngine`. The ROS
node remains responsible for mode interlocks, feedback acquisition, command
transmission, and emergency stop. Each executed cycle is published as schema
version 1 JSON on `/arm_control_sample`; enable/disable and emergency-stop
events are published on `/arm_control_event`.

The workspace also contains the standalone `piper_l_admittance_mit` and
`nero_admittance_mit` packages. They implement a different velocity-admittance
chain (`wrench -> admittance twist -> bounded weighted DLS -> MIT reference`).
This repository intentionally retains its pose-offset chain
(`wrench -> bounded SE(3) offset -> screw IK -> planned move_j`) for now; the
standalone implementation is a future alternative adapter, not silently mixed
into the current controller.

Start a self-describing JSONL experiment together with keyboard control:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l \
  experiment_recording_enabled:=true \
  experiment_name:=cartesian_impedance_gain_a \
  experiment_output_directory:=~/.ros/armbycontroller/experiments
```

Each fresh run directory contains `manifest.json`, `samples.jsonl`,
`events.jsonl`, and `summary.json`. The summary reports sample/event counts,
controller and command-mode counts, period statistics, joint-reference RMSE,
maximum absolute joint error, and maximum estimated torque. Recording is in a
separate process and never accesses CAN. When launched independently, control
it with:

```bash
ros2 run armbycontroller experiment_recorder_node.py --ros-args \
  -p experiment_name:=manual_comparison
ros2 service call /arm_experiment_recorder/recording \
  std_srvs/srv/SetBool '{data: true}'
ros2 service call /arm_experiment_recorder/recording \
  std_srvs/srv/SetBool '{data: false}'
```

The file map outside `ros/`, `ik/`, and `impedance/` is split by category under
`docs/file_map/README_ZH_EN.md`.

## CAN and robot startup

Configure both CAN interfaces once:

```bash
./scripts/setup_can.sh
```

Then start the required robot:

```bash
# Nero
./scripts/start_nero.sh

# Or Piper-L
./scripts/start_piper_l.sh
```

The robot scripts do not reconfigure CAN, automatically move home, or clear a
latched emergency stop. They use the X11 keyboard backend for NoMachine; pass
`device:=/dev/input/eventN` to use a local evdev keyboard instead.

## Build

```bash
cd /home/techshare/demo_ws
python3 -m pip install "modern_robotics>=1.1.1"
colcon build --packages-up-to armbycontroller
source install/setup.bash
```

The controller uses the PoE, analytic Jacobian, and RNEA implementation
validated in `nero_screw_dynamics`. Nero URDF/Xacro files are resolved from
that package first. Piper-L remains supported through the model bundled in
`armbycontroller`. Both models use the same screw-theory IK/FK and
inverse-dynamics module; `pytracik` is no longer required.
The dynamically consistent nullspace mass matrix uses one CRBA tree sweep
instead of seven repeated inverse-dynamics calls on Nero. Cartesian impedance
also caches target FK/Jacobian while the joint reference is unchanged; current
measured-state kinematics and dynamics are never cached. On the captured
1,218-sample Nero trace, these changes reduced offline pure-control replay from
about 9.72 ms to about 2.4 ms per cycle without changing recorded torques
beyond `1.8e-11 N.m`.
Internal callers use `create_screw_solver`, `UrdfScrewModel`, and
`project_gravity_vector` directly; the old TRAC-IK and gravity-model
compatibility aliases have been removed.

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
`τ_ff=clip(Jg.T Fc+tau_null+tau_posture+C(q,dq)dq+g(q), ±τ_limit)`. Thus the
firmware does not add a second joint spring or damper. `tau_null` is nonzero
only for Nero; the unprojected `tau_posture` is configured only on Nero
J2/J3/J4. A continuous joint reference supplies the target pose, target twist,
nullspace posture, and joint-posture reference; this reference continuity is
not a torque-rate limiter.

Piper-L kinematics and inverse dynamics both use
`piper_l_with_gripper_description.xacro`. Accessory joints are fixed at their
URDF zero positions, their mass/inertia contributes to `C*dq+g`, and only the
six arm joints receive MIT commands. The controlled task point remains
`link6`; it is not the fingertip contact point. Complete measured `q/dq` and a
valid URDF model are required before entry.

For comparison, `impedance_backend:=joint` retains the previous native MIT
joint impedance and its `mit_kp`, `mit_kd`, `mit_feedforward`, and
`mit_gravity_*` parameters.

## Controller configuration

`keyboard_control.launch.py` loads two ordered ROS parameter files:
`config/common.yaml`, followed by exactly one robot profile,
`config/nero.yaml` or `config/piper_l.yaml`. The common file contains only the
shared controller/observer rates, default interaction backend, and firmware
probe timing. Firmware configuration checks, tool, gravity/model compensation,
joint gains, Cartesian gains, torque limits, trajectory limits, and observer
tuning are explicit in each robot file.
Piper-L admittance parameters exist only in `piper_l.yaml`; Nero velocity,
nullspace, and joint-selective posture parameters exist only in `nero.yaml`.

The Nero profile explicitly uses `nero_mount: side` and
`tool_configuration: none`, so it loads the bare `nero_description.urdf`.
The Piper-L profile independently uses `tool_configuration: gripper`.
The physical tool and mount must match the selected robot profile.

Explicit launch arguments override both files. Use `common_config:=...` to
replace the shared layer or `controller_config:=...` to replace the selected
robot layer. With the default `controller_config:=__robot__`, `robot_model`
selects `nero.yaml` or `piper_l.yaml` automatically.

For Nero J1 through J7, use a seven-value
`cartesian_impedance_model_scale`. J4 is the fourth value. Keep all values at
`1.0` until a supported, static torque calibration identifies a scale; do not
guess a J4 value from motion drift alone. Removing the incorrectly modeled
Revo2 removes about `1.57 N.m` from the previously reproduced J4 model torque
at the logged pose. The latest hardware log reported software `1.11` while
real hardware now always uses two connections. A first SDK `default` instance
reads and saves the complete firmware dictionary, then disconnects. After the
shared `firmware_reconnect_delay` (default `0.5 s`), a distinct second instance
is created with the detected profile (`1.11 -> v111` for Nero,
`S-V1.8-8 -> v188` for Piper-L). The probe neither enables the arm nor sends
motion. Failure to obtain or parse `software_version` aborts startup.

Run Nero:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 execute_motion:=true \
  move_home_on_start:=false reset_emergency_stop_on_start:=true
```

The default configuration uses the project's `pitch=-90°` side-mount
convention (`gravity_vector=[-g,0,0]`). Override it with
`nero_mount:=horizontal` only when the base is on a horizontal surface
(`gravity_vector=[0,0,-g]`). Left/right side-mount yaw does not change gravity
in `base_link`. Nero `tool_configuration: none` loads the bare
`nero_description.urdf`; no gripper or Revo2 mass/inertia is included. Nero
also uses equal X/Y/Z rotational stiffness; the stronger base-Z setting is
reserved for Piper-L. Press `I` to capture the current tool pose, nullspace
posture, and J2/J3/J4 posture reference before enabling Cartesian MIT.

Run Piper-L with the same keys:

```bash
ros2 launch armbycontroller keyboard_control.launch.py \
  robot_model:=piper_l device:=/dev/input/event3 \
  can_interface:=can0 firmware:=auto \
  impedance_backend:=cartesian \
  cartesian_impedance_base_z_rotation_stiffness:=4.0

# Start directly in MIT impedance mode by adding impedance_enabled:=true.
```

Inspect the observer output after entering MIT with `I`:

```bash
ros2 topic echo /arm_external_joint_torque
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
zero is clear. `firmware:=auto` uses the two-stage hardware result. An explicit
firmware argument is checked and reported, but the detected hardware profile
wins for the formal connection; it remains the profile used by dry-run mode.
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
