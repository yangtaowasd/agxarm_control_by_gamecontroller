# armbycontroller

Independent ROS 2 keyboard controllers for AgileX Piper and NERO arms.

## Build

```bash
colcon build --packages-select armbycontroller
source install/setup.bash
```

## NERO: independent joint keyboard control

The NERO program is separate from the Piper controller. At startup it connects,
enables the arm, reads the current seven-joint feedback, commands all joints to
zero, and waits for the home position before accepting keyboard motion. Pressing
Ctrl-C only disconnects CAN; it does not move, enable, or disable the arm.

```bash
ros2 launch armbycontroller nero_joint_keyboard_control.launch.py \
  device:=/dev/input/event3 \
  can_interface:=can0 \
  firmware:=default
```

Key mapping:

- `1` ... `7`: select NERO joint 1 ... 7
- `A`: decrease the selected joint angle
- `D`: increase the selected joint angle
- `SPACE`: command all seven joints to zero (within configured limits)
- `E`: electronic emergency stop; restart the controller after resolving the
  cause because motion stays locked for the rest of that process

The default increment is `step_rad:=0.005` rad per 20 Hz control tick and the
default arm speed is `speed_percent:=20`. Start conservatively and keep a hand
near the physical emergency stop. Startup home defaults to a `0.01` rad
tolerance and a 30-second timeout. These can be changed with
`startup_home_tolerance` and `startup_home_timeout`; startup homing can be
disabled with `move_home_on_start:=false`. To verify keyboard input and targets
without sending hardware commands, use:

```bash
ros2 launch armbycontroller nero_joint_keyboard_control.launch.py \
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
ros2 run armbycontroller nero_keyboard --ros-args -p device:=/dev/input/event3
ros2 run armbycontroller nero_joint_keyboard_controller.py --ros-args \
  -p can_interface:=can0 -p firmware:=default
```

## Piper keyboard control

The package has one launch file. Start both nodes with:

```bash
ros2 launch armbycontroller piper_keyboard_control.launch.py
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
