# Architecture

`armbycontroller` separates robot control from backend transport. The arm
controller owns safety, IK, trajectory generation, and hardware access. Backend
adapters translate external protocols into that controller's stable command
inputs and report its feedback.

## Runtime paths

```text
                         +----------------------+
keyboard device --------> keyboard_controller  +------> AGX arm / simulation
                         |                      |
Motion Link WebSocket -->| internal ROS seam    |
HTTP backend API -------->|                      |
                         +----------+-----------+
                                    |
                    joint state / pose / IK status
```

Only one backend adapter should own commands in normal operation. The
`backend_transport` launch argument enforces that choice:

- `motion_link` keeps compatibility with the existing Phone Remotation
  `role=robot` WebSocket protocol.
- `http` omits the WebSocket bridge and gives command ownership to the local
  versioned HTTP API.
- `both` is an explicit migration/debug mode.

## Modules

| Module | Responsibility |
| --- | --- |
| `model_profiles.py` | Canonical static Nero/Piper-L facts |
| `control_protocol.py` | Canonical 23-key indices and semantic action mapping |
| `keyboard_controller.py` | Unified joint, Cartesian IK, planned, MIT, simulation, and hardware control |
| `pose_controller.py` | Standalone absolute-pose IK controller used by RViz workflows |
| `backend_protocol.py` | Transport-independent command and feedback validation |
| `backend_api.py` | Local versioned HTTP command API and SSE feedback adapter |
| `motion_link_bridge.py` | Existing Motion Link WebSocket adapter and phone-orientation mapping |
| `ik_core.py`, `screw_ik.py`, `screw_model.py` | Shared PoE FK/IK implementation |
| `gravity_compensation.py` | URDF rigid-body inverse dynamics adapter |
| `hardware_probe.py` | Read-only hardware detection for backend startup |

## Stable seams

Backend callers should use the HTTP/SSE interface documented in
[`phone-remotation-api.md`](phone-remotation-api.md). The ROS topics between
adapters and controllers are internal integration seams and remain available
for native ROS tooling.

Model-specific joint counts, initial poses, tip links, topic prefixes, and
workspace radii must be added to `model_profiles.py`, rather than copied into
controllers or launch files.

## Safety ownership

The adapters validate transport shape and authorization. The controller owns
workspace limits, IK/FK verification, stale keyboard timeout, motion enablement,
and electronic emergency stop. HTTP `202 Accepted` means a command entered the
controller pipeline; feedback must be observed to determine its outcome.
