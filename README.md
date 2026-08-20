# agxarm_control_by_gamecontroller

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
immediate formula result. The shared MIT envelope applies a per-joint absolute
limit (±8 N·m by default, including model support and posture torque) and an
estimated-total-torque slew limit (`interaction_torque_rate_limit`, default
`20 N·m/s`). Rate history is reset from measured motor torque on mode entry.

## Passive momentum observer and Cartesian admittance

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

The observer process never connects to CAN or calls the arm SDK. It updates
once for every timestamped 100 Hz dynamics-state message, using the previous
cycle's measured motor torque for the interval that just elapsed. The default
gain is `momentum_observer_gain:=10.0 1/s`. Its residual is not added to
Cartesian-impedance torque; it remains a diagnostic output and an input to
admittance. Disable the process with `momentum_observer_enabled:=false`.

On both Nero and Piper-L, `O` toggles Cartesian admittance. The observed
external joint torque is mapped to a base-frame wrench by damped least squares,
integrated into a Cartesian velocity, and passed to the bounded screw-Jacobian
velocity IK. That solver obtains the PoE space Jacobian from the shared robot
model, converts it to the tool-origin geometric Jacobian, then applies weighted
DLS with joint-speed and predictive-position bounds. The resulting joint
velocity is tracked with low-gain MIT. Every cycle uses
`q_ref=q_measured+dq_ref*dt`, so tracking error cannot accumulate into a
pullback toward an old joint target. `dq_ref` is capped at `1.0 rad/s` per
joint. On Nero, measured speed above `2.5 rad/s` for three consecutive 100 Hz
cycles, or above the `2.8 rad/s` hard limit once, triggers the electronic stop
(Piper-L keeps a `1.5 rad/s` sustained threshold).
Estimated MIT total torque is capped at `8 N·m` and slewed at `20 N·m/s` by
default. If observer wrench data or its source timestamp exceeds the `0.10 s`
freshness window, admittance or hybrid control exits to a measured-position
hold; failure to confirm that handoff triggers the electronic stop. The separate
`armbycontroller/admittance/`
package provides two explicit modes:

- `zero_force` (soft zero force):
  `M_a*xdd + D_0*xd + K_h*x + F_stick/slip = F_ext`. `K_h` is deliberately
  weak; together with light damping and bounded virtual friction it rejects
  observer bias and avoids drift without creating the strong return feel of
  `resistive`.
- `resistive`: `M_a*xdd + D_r*xd + K_r*x = F_ext`, with damping and a spring
  back to the pose captured on entry.

Nero defaults to `admittance_mode:=zero_force`; Piper-L defaults to
`admittance_mode:=resistive`. Each robot has independent mass, damping,
stiffness, deadband, and motion limits in its own YAML file.

`H` toggles one-owner hybrid Cartesian control. Its default Cartesian
Compliance Subspace is `hybrid_admittance_axes:=z` in
`hybrid_admittance_frame:=base`. Set the frame to `tool` to capture the tool
orientation on entry, or to `custom` with
`hybrid_admittance_frame_rotation:=[rx,ry,rz]` to orient compliant axes by an
SO(3) rotation vector. The resulting orthogonal projector and its complement
assign every task direction to exactly one outer loop. Runtime subspace
reconfiguration projects virtual velocity and re-anchors newly rigid
directions at measured pose.
Both paths are combined inside `HybridCartesianController` and sent through
one bounded screw-velocity IK, model compensation, low-gain MIT envelope, and
one command stream. `hybrid_desired_wrench` uses
`[Mx,My,Mz,Fx,Fy,Fz]` and defaults to zero.

Admittance and hybrid velocity IK continuously increase DLS damping and scale
the requested Twist as the minimum Jacobian singular value falls from
`0.05` to `0.01`; nonzero motion is rejected at or below `0.01`. Cartesian
impedance wrench is limited before `J^T` to `10 N` translational-force norm
and `4 N.m` rotational-torque norm, in addition to the joint torque envelope.

All MIT interaction modes now use the same `interaction_*` joint-safety
configuration. The shared defaults are an `8 N.m` estimated-total-torque
limit, `20 N.m/s` estimated-total-torque rate limit, `1.0 rad/s`
reference-speed limit, three-cycle sustained-speed debounce, and `0.03 rad`
joint-limit margin. Nero uses `2.5 rad/s` sustained and `2.8 rad/s` immediate
measured-speed stops; Piper-L retains `1.5 rad/s` and `2.5 rad/s` respectively.
Admittance-specific wrench, virtual Twist, and offset bounds remain
separate because they are task-space control-law parameters.

`I`, `O`, and `H` are strictly interlocked. Every cross-mode transition follows
`current interaction mode -> normal planned-position mode -> target interaction
mode`. The target mode is rejected if restoring normal planned-position
control fails.

The interaction controllers share six ROS/CAN-free control modules. Cartesian
task geometry owns the base-frame `[angular; linear]` convention, SE(3)
validation, tool-origin geometric Jacobian, and `tau=Jg.T*wrench` mappings.
Model Compensation selects gravity, bias, or full inverse dynamics.
Interaction Safety Limits validates the shared torque, speed, and joint-limit
boundaries once. The MIT Safety Envelope checks feedback feasibility and caps
feedforward against both estimated total-torque and torque-rate limits. The
Control Cycle Guard
validates feedback, period, position, and velocity. The Interaction Mode
Lifecycle enforces the normal-mode intermediate transition. The bounded
screw-Jacobian velocity IK lives beside the full-pose solver in
`armbycontroller/ik/screw.py`.

All MIT adapters expose the same torque diagnostics:
`torque_feedback`, `torque_model_requested`, `torque_task_requested`,
`torque_auxiliary_requested`, `torque_feedforward_requested`,
`torque_feedforward_sent`, `torque_total_requested`,
`torque_total_estimated`, `torque_rate_limited`, `torque_rate_limit`, and
`torque_saturation_reason`. Joint MIT requests
full inverse dynamics, Cartesian impedance requests bias compensation, and
Cartesian admittance requests gravity compensation.

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

All four in-tree interaction adapters now run behind the same ROS/CAN-free
controller seam:

```text
ControlInput(state, reference, wrench, timestamp, period)
    -> ControllerAdapter.reset/step
    -> ControlResult(MIT command, diagnostic signals)
```

`joint_impedance`, `cartesian_impedance`, `cartesian_admittance`, and
`hybrid_cartesian` are adapters registered with `ControlEngine`. The ROS
node remains responsible for mode interlocks, feedback acquisition, command
transmission, and emergency stop. Each executed cycle is published as schema
version 1 JSON on `/arm_control_sample`; enable/disable and emergency-stop
events are published on `/arm_control_event`.

The UI-facing interaction API is transport-neutral in
`armbycontroller/api/interaction.py` and has thin ROS adapters. These standard
`std_srvs/srv/Trigger` services set modes idempotently:

```bash
ros2 service call /arm/set_normal_mode std_srvs/srv/Trigger '{}'
ros2 service call /arm/set_impedance_mode std_srvs/srv/Trigger '{}'
ros2 service call /arm/set_admittance_mode std_srvs/srv/Trigger '{}'
```

Each response uses `success` plus a schema-v1 JSON `message` containing
`requested_mode`, `active_mode`, `changed`, and a human-readable message.
Cross-mode requests retain the normal-mode intermediate transition. There is
deliberately no public hybrid-mode service. A transient-local schema-v1 JSON
snapshot on `/arm/interaction_state` exposes the actual mode, public
`available_modes`, configured service names, backend selections, readiness,
connection, emergency-stop, and dry-run state. It may report `hybrid` when the
keyboard entered that mode, but `hybrid` is never listed as requestable.

The low-gain MIT reference path is based on the hardware-validated
`nero_admittance_mit` chain:
`wrench -> admittance twist -> bounded screw-Jacobian velocity IK ->
measured-state-reanchored MIT reference`. This repository retains its explicit `zero_force`
and `resistive` virtual dynamics and its shared observer/interlock architecture.

Start a self-describing JSONL experiment together with keyboard control:

```bash
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=piper_l \
  experiment_recording_enabled:=true \
  experiment_name:=cartesian_impedance_gain_a \
  experiment_output_directory:=~/.ros/agxarm_control_by_gamecontroller/experiments
```

Each fresh run directory contains `manifest.json`, `samples.jsonl`,
`events.jsonl`, and `summary.json`. The summary reports sample/event counts,
controller and command-mode counts, period statistics, joint-reference RMSE,
maximum absolute joint error, and maximum estimated torque. Recording is in a
separate process and never accesses CAN. When launched independently, control
it with:

```bash
ros2 run agxarm_control_by_gamecontroller experiment_recorder_node.py --ros-args \
  -p experiment_name:=manual_comparison
ros2 service call /arm_experiment_recorder/recording \
  std_srvs/srv/SetBool '{data: true}'
ros2 service call /arm_experiment_recorder/recording \
  std_srvs/srv/SetBool '{data: false}'
```

The file map outside `ros/`, `ik/`, and `impedance/` is split by category under
`docs/file_map/README_ZH_EN.md`.

## CAN and robot startup

The robot startup scripts configure `can0` before launching. To
configure CAN without starting a robot, run:

```bash
./scripts/setup_can.sh
```

Start the required robot directly:

```bash
# Nero on a horizontal base; YAML still defaults to side mounting
./scripts/start_nero.sh nero_mount:=horizontal

# Or Piper-L
./scripts/start_piper_l.sh
```

The robot scripts configure CAN but do not automatically move home or reset a
latched electronic stop. If the arm reports an electronic stop, startup remains
disabled until the operator inspects the arm and explicitly adds
`reset_emergency_stop_on_start:=true`.
CAN setup uses `sudo` when the current user is not root. The scripts use the
interactive terminal to select either the X11 keyboard backend for NoMachine
or a local `/dev/input/eventN` evdev keyboard. In non-interactive use they
default to X11. Pass `device:=x11` or `device:=/dev/input/eventN` to skip the
menu. Local evdev selection accepts `3`, `event3`, or `/dev/input/event3`;
`device:=3` is also expanded to `/dev/input/event3`. Any launch setting can be
overridden as a trailing argument, for example
`./scripts/start_nero.sh reset_emergency_stop_on_start:=true`.

## Build

```bash
cd /home/yang/demo_ws
python3 -m pip install "modern_robotics>=1.1.1"
colcon build --packages-up-to agxarm_control_by_gamecontroller
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
- `O`: toggle the selected low-gain MIT Cartesian admittance mode
- `H`: toggle hybrid control; default Z admittance plus five-axis impedance
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
nullspace posture, and joint-posture reference. The shared MIT envelope then
applies the independent torque-rate limit; reference continuity remains
necessary and is not replaced by that final safety bound.

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
Nero and Piper-L admittance parameters are independent in their respective
YAML files, including MIT gains, total-torque bounds, joint-velocity bounds,
task weights, and DLS damping. Nero velocity, nullspace, and joint-selective
posture parameters remain Nero-only.

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
real hardware now always uses two simultaneous connections. A first SDK
`default` instance connects, sends one enable request, and saves the complete
firmware dictionary. It stays connected while a distinct second instance is
created immediately with the detected profile (`1.11 -> v111` for Nero,
`S-V1.8-8 -> v188` for Piper-L). There is no `disable()` or `disconnect()`
between the two `connect()` calls. Both instances are retained until node
shutdown. The probe sends no mode, motion, or firmware-write command.

Run Nero:

```bash
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=nero device:=/dev/input/event3 \
  can_interface:=can0 execute_motion:=true \
  nero_mount:=horizontal \
  move_home_on_start:=false reset_emergency_stop_on_start:=false
```

The YAML default remains the project's `pitch=-90°` side-mount convention
(`gravity_vector=[-g,0,0]`). The command above overrides it without editing the
default; use `nero_mount:=horizontal` only when the base is horizontal
(`gravity_vector=[0,0,-g]`). Left/right side-mount yaw does not change gravity
in `base_link`. Nero `tool_configuration: none` loads the bare
`nero_description.urdf`; no gripper or Revo2 mass/inertia is included. Nero
also uses equal X/Y/Z rotational stiffness; the stronger base-Z setting is
reserved for Piper-L. Press `I` to capture the current tool pose, nullspace
posture, and J2/J3/J4 posture reference before enabling Cartesian MIT.

Run Piper-L with the same keys:

```bash
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
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
ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=nero execute_motion:=false
```

The controller defaults to 100 Hz IK/control scheduling. Per-tick keyboard
increments are scaled so the original Cartesian, orientation, and joint jog
speeds are preserved at the higher rate. The MIT reference generator defaults
to 1.0 rad/s velocity, 1 rad/s² acceleration, and 5 rad/s³ jerk limits, exposed
as `interaction_reference_joint_velocity_limit`,
`mit_trajectory_max_acceleration`, and `mit_trajectory_max_jerk`. Planned mode
retains 20 percent speed and 1 rad/s²
maximum joint acceleration. IK keeps the tool
local `+Z` axis pointing toward `base_link -Z`; rotation around that axis is
free. It retains ten verified states and pauses for two seconds after recovery.
Acceleration limits are written and verified one joint at a time because the
Piper S-V1.8-8 batch (`joint_index=255`) ACK path is unreliable.
The controller also waits for `CAN_CTRL/MOVE_J` feedback before sending the
first startup target so Piper does not discard it during a mode transition.
Real-hardware launch preserves a latched electronic stop and the current joint
position by default. Explicit automatic homing moves joints to zero strictly in
order and refuses to start if feedback is outside configured soft limits; it
never disables joint limits. Only use it when the complete path to zero is
clear. Any homing timeout triggers the electronic stop. `firmware:=auto` uses
the two-stage hardware result. An explicit
firmware argument is checked and reported, but the detected hardware profile
wins for the formal connection; it remains the profile used by dry-run mode.
If the arm reports a latched electronic emergency stop, normal startup refuses
to move. After physically checking the arm, explicitly add
`reset_emergency_stop_on_start:=true` to reset the controller before enabling.
Initialization failures after an enable request actively disable the arm, and
homing timeouts trigger the electronic stop. Graceful shutdown disconnects
without sending `disable()` so another controller can take over. Set
`disable_arm_on_shutdown:=true` when shutdown must explicitly disable the arm.

## RViz pose simulation

```bash
# Nero
ros2 launch agxarm_control_by_gamecontroller pose_rviz.launch.py robot_model:=nero

# Piper-L
ros2 launch agxarm_control_by_gamecontroller pose_rviz.launch.py robot_model:=piper_l
```

In another interactive terminal:

```bash
ros2 run agxarm_control_by_gamecontroller terminal_teleop.py --ros-args \
  -p topic_prefix:=/nero -p step:=0.01

# Use /piper_l instead when running Piper-L.
```

Terminal keys use the same Cartesian directions and orientation keys:
`W/S`, `A/D`, `Z/X`, arrows, and `PageUp/PageDown`. `R` resets the complete
pose to feedback and `Q` quits.

## Standalone pose controller

```bash
ros2 run agxarm_control_by_gamecontroller pose_controller.py --ros-args \
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
ros2 run agxarm_control_by_gamecontroller hand_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=v111 -p execute_motion:=true
```
