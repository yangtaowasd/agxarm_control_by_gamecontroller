# armbycontroller

Independent ROS 2 keyboard controllers for AgileX Piper and NERO arms.

## Build

```bash
colcon build --packages-select armbycontroller
source install/setup.bash
```

## NERO: joint/Cartesian keyboard control

The NERO program is separate from the Piper controller. At startup it connects,
enables the arm, reads the current seven-joint feedback, commands all joints to
zero, and waits for the home position before accepting keyboard motion. Pressing
Ctrl-C only disconnects CAN; it does not move, enable, or disable the arm.

```bash
ros2 launch armbycontroller keyboard_control.launch.py robot_model:=nero \
  device:=/dev/input/event3 \
  can_interface:=can0 \
  firmware:=default
```

Key mapping:

- `P`: switch between joint mode and Cartesian IK mode
- `1` ... `7`: select NERO joint 1 ... 7
- Joint mode: `A/D` decrease/increase the selected joint angle
- IK mode: `W/S` = `+X/-X`, `A/D` = `+Y/-Y`, `Z/X` = `+Z/-Z`
- `SPACE`: command all seven joints to zero (within configured limits)
- `E`: electronic emergency stop; restart the controller after resolving the
  cause because motion stays locked for the rest of that process

IK changes only XYZ. The tool keeps the configured pointing direction
(`link7 +Z` points toward `base_link -Z` by default), rather than locking the
orientation captured at mode entry. Rotation about that pointing axis remains
free; the controller samples it and chooses the solution nearest the current
joints. The Cartesian increment is
`cartesian_step:=0.005` m per 20 Hz tick. The controller retains 10 valid IK
targets; if IK fails it returns to the latest one, logs the cause, and pauses
Cartesian input for two seconds.

The default joint increment is `step_rad:=0.005` rad per 20 Hz control tick and
the default arm speed is `speed_percent:=20`. Joint acceleration is constrained
with `joint_max_acceleration:=1.0`. Start conservatively and keep a hand
near the physical emergency stop. Startup home defaults to a `0.01` rad
tolerance and a 30-second timeout. These can be changed with
`startup_home_tolerance` and `startup_home_timeout`; startup homing can be
disabled with `move_home_on_start:=false`. To verify keyboard input and targets
without sending hardware commands, use:

```bash
ros2 launch armbycontroller keyboard_control.launch.py robot_model:=nero \
  execute_motion:=false
```

Supported NERO firmware settings:

- `default`: firmware 1.10 and earlier
- `v111`: firmware 1.11
- `v112`: firmware 1.12
- `v120`: firmware 1.20 and later

The keyboard node reads Linux input events directly. Set `device` to the correct
entry from `/dev/input/by-id/` or `/dev/input/event*`, and make sure the current
user has read permission. The CAN interface must already be configured and up.

The two NERO nodes can also be run separately:

```bash
ros2 run armbycontroller keyboard --ros-args \
  -p profile:=nero -p device:=/dev/input/event3
ros2 run armbycontroller nero_joint_keyboard_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=default
```

## Piper keyboard control

The package has one launch file. Start both nodes with:

```bash
ros2 launch armbycontroller keyboard_control.launch.py robot_model:=piper
```

The C++ keyboard reader and Python controller can still be run separately:

```bash
ros2 run armbycontroller keyboard
ros2 run armbycontroller piper_keyboard_controller.py
```

Key mapping:

- `A` / `D`: move `j1` left / right
- `W` / `S`: move `j2` and `j3` together to lift / lower the arm
- `PAGEUP` / `PAGEDOWN`: rotate `j6` in the negative / positive direction
- `-/_`: reset only the accumulated `j3` orientation correction to zero
- Arrow keys: control wrist orientation using a two-dimensional IK that solves
  `j4` and `j5`, then enforces `j6=-j4`. The default
  `orientation_mapping=square` projects a virtual square onto the tool Z-axis
  front hemisphere and solves the unit direction vector directly. It does not
  use Euler yaw at the pole. Use `orientation_mapping:=euler` for direct Euler
  yaw/pitch increments, or `arrow_mode:=joint` for direct joint-axis control.
- The default virtual square is `[-1, 1]` on both axes. Its center is zero wrist
  tilt and `orientation_square_max_tilt_deg=80` maps every boundary point to an
  80-degree tilt. A left/right edge midpoint reaches `abs(j4)=90 degrees` and
  `abs(j5)=80 degrees`.
- `orientation_yaw_joint_step_scale` makes the `j4=-j6` yaw pair move faster
  than `j5`.
- The whole-arm speed defaults to `40%`. J4/J6 keyboard increments use the
  `j4_j6_speed_percent=80` ratio, so they update at twice the other joints'
  rate. When up/down reaches the J5 80-degree orientation limit, J3 adds an
  independent correction (up to 10 degrees by default) to cover the remaining
  front hemisphere. Press `-/_` to remove only that correction.
- Only the square center is a pole. The solver searches both equivalent wrist
  charts, so it can cross `j5=0`, use positive and negative `j5`, and change
  chart before the physical `j4` limit blocks motion. A whole `y=0` dead zone
  is intentionally not used because it would erase valid left/right targets.
- `orientation_ik_tolerance`, `orientation_ik_refine_iterations`,
  `orientation_ik_refine_damping`, `orientation_ik_fd_eps`, and
  `orientation_ik_line_search_steps` tune final IK precision after the fast
  approach phase. If FK output is noisy, raise `orientation_ik_fd_eps` a little;
  if the final pose is still loose, raise `orientation_ik_refine_iterations`.
  `orientation_ik_coordinate_iterations` enables a final coordinate-search
  fallback for cases where the damped solver stalls near a joint limit.
- `orientation_j5_limit_deg` limits the wrist pitch search range. The default
  is `80`, leaving 9.5 degrees inside the configured physical j5 limit.
  `avoid_j5_zero_deg` defaults to `0`, so j5 is not blocked at zero.
- Wrist limits default to `j4=+/-127`, `j5=+/-89.5`, and `j6=+/-170` degrees.
  They are applied both to controller IK and the pyAgxArm configuration.
- `SPACE`: return target joints to `[0, 0, 0, 0, 0, 0]`
- In `arrow_mode=orientation`, startup visits mechanical home
  `[0, 0, 0, 0, 0, 0]`. `SPACE` returns to that same mechanical home.
- On Ctrl-C, the controller commands shutdown home
  `[0, 0, 0, 0, shutdown_j5_deg, 0]` and waits until every joint is within
  `shutdown_home_tolerance` before disconnecting. The default j5 target is
  `30` degrees and the default tolerance is `0.0001` rad; if the target is not
  reached, disconnect is blocked. The controller does not disable the arm on
  shutdown.
# agxarm_control_by_gamecontroller

## NERO Cartesian IK keyboard control and RViz simulation

This package also contains the merged `pytracik` Cartesian controller. It uses
the official AGX NERO URDF, verifies every IK result with FK, and sends only
joint-space `move_j()` targets. The real controller configures and verifies
joint acceleration limits before motion.

Install the non-ROS solver dependency once:

```bash
sudo apt install libboost-all-dev libeigen3-dev liborocos-kdl-dev \
  libnlopt-dev libnlopt-cxx-dev
python3 -m pip install pytracik
```

Start the CAN-free RViz simulation:

```bash
ros2 launch armbycontroller agx_ik_rviz.launch.py robot_model:=nero
```

In another interactive terminal, start Cartesian keyboard control:

```bash
ros2 run armbycontroller nero_keyboard_teleop.py --ros-args -p step:=0.01
```

Key mapping in the `base_link` frame:

- `W/S`: `+X/-X`
- `A/D`: `+Y/-Y`
- `Z/X`: `+Z/-Z`
- `R`: reset the accumulated target to the current position
- `Q`: quit

The target is a fixed pointing direction, not a fully locked quaternion;
keyboard commands change XYZ only and roll about the pointing axis remains
free. By default, `link7` local `+Z` points along
`base_link -Z` (downward). Change it from the keyboard node command line:

```bash
ros2 run armbycontroller nero_keyboard_teleop.py --ros-args \
  -p step:=0.01 \
  -p pointing_direction:="[1.0, 0.0, 0.0]" \
  -p roll_reference:="[0.0, 0.0, 1.0]"
```

`roll_reference` provides the first roll candidate. The controller searches
around the pointing axis and chooses the candidate closest to the seed joints.
Simulation applies joint velocity and acceleration limits. The controller keeps
the latest 10 IK/FK-valid targets. On an IK failure or workspace violation it
returns to the latest valid target, pauses input for two seconds, and publishes
details on `/nero/ik_status`.

The default radial workspace, measured from `base_link` to `link7`, is
`0.1947354 ... 0.6374482 m`: 5 cm inside the URDF-derived minimum reach and
10 cm inside the maximum reach.

Run the Cartesian controller against real NERO hardware separately:

```bash
ros2 run armbycontroller nero_ik_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=v111 -p execute_motion:=true
```

## Piper-L URDF, IK, and RViz

The same pytracik/FK controller supports the official six-axis Piper-L URDF
(`piper_l_description.urdf`). Start the CAN-free RViz simulation with:

```bash
ros2 launch armbycontroller agx_ik_rviz.launch.py robot_model:=piper_l
```

Then run the terminal keyboard publisher against the Piper-L topics:

```bash
ros2 run armbycontroller nero_keyboard_teleop.py --ros-args \
  -p topic_prefix:=/piper_l -p step:=0.01
```

For real Piper-L hardware, do not run the simulation launch controller. Run:

```bash
ros2 run armbycontroller nero_ik_controller.py --ros-args \
  -p robot_model:=piper_l -p topic_prefix:=/piper_l \
  -p tip_link:=link6 -p firmware:=default \
  -p initial_joint_positions:="[0.0, 1.3939753, -1.0158306, 0.0, 1.2799181, 0.0]" \
  -p robot_min_reach:=0.0 -p robot_max_reach:=0.8738043 \
  -p execute_motion:=true
```

Its allowed radial shell is `0.05 ... 0.7738043 m`, retaining the requested
5 cm inner and 10 cm outer safety margins. Position targets use
`/piper_l/target_pose`; FK feedback uses `/piper_l/current_pose`.

## Revo2 bridge validation

The Revo2 bridge test node is installed as part of this package:

```bash
ros2 run armbycontroller revo2_hand_test.py --ros-args \
  -p can_interface:=can0 -p firmware:=v111 -p execute_motion:=true
```
