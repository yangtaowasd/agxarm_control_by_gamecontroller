# Phone Remotation Backend API

`armbycontroller` exposes a versioned HTTP API for the Phone Remotation
backend. ROS 2 topics remain an internal transport between the API gateway and
the arm controller; the backend does not need ROS message knowledge.

The default origin is:

```text
http://127.0.0.1:8765/api/v1
```

## Interaction model

```text
Phone / desktop UI
        ⇅
Phone Remotation backend
        ⇅  HTTP commands + SSE state events
armbycontroller backend_api
        ⇅  internal ROS 2 messages
Nero / Piper-L controller
```

## Is the bridge still needed?

Yes for the current Phone Remotation backend. Its phone sensor and virtual
controller messages are delivered to a `role=robot` WebSocket client, so
`motion_link_bridge.py` is the compatibility adapter that consumes them.

The launch file prevents the WebSocket and HTTP adapters from silently
competing for command ownership. Select one transport:

| `backend_transport` | WebSocket bridge | HTTP command API | Intended use |
| --- | --- | --- | --- |
| `motion_link` | command owner | read-only | Current Phone Remotation backend; default |
| `http` | not started | command owner | Backend calls this API directly |
| `both` | command owner | command owner | Temporary migration/testing only |

State reads remain available from the HTTP API in `motion_link` mode. The
`both` setting is explicit because two command producers require coordination
in the backend and should not be enabled accidentally.

Commands return HTTP `202` when they have been validated and published to the
controller. Execution is asynchronous. The backend should use the state event
stream to observe `ikStatus`, current pose, and joint feedback.

## Read endpoints

### `GET /api/v1/health`

No token is required. It reports API availability, robot model, feedback
connectivity, and whether commands are enabled.

```json
{
  "ok": true,
  "apiVersion": "v1",
  "robotModel": "nero",
  "connected": true,
  "commandsEnabled": true,
  "runtimeMode": "hardware",
  "simulationMode": false,
  "executeMotion": true
}
```

### `GET /api/v1/state`

Returns the latest complete backend snapshot. Joint positions use radians.

```json
{
  "ok": true,
  "state": {
    "apiVersion": "v1",
    "robotModel": "nero",
    "commandsEnabled": true,
    "runtimeMode": "hardware",
    "simulationMode": false,
    "executeMotion": true,
    "connected": true,
    "jointState": {
      "names": ["joint1", "joint2"],
      "positionRad": [0.0, 0.2],
      "velocityRadS": [0.0, 0.0]
    },
    "currentPose": {
      "frameId": "base_link",
      "position": {"x": 0.3, "y": 0.0, "z": 0.4},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    "ikStatus": {"state": "ok", "message": "phone pose accepted"},
    "updatedAt": 1785800000.0
  }
}
```

### `GET /api/v1/events`

Long-lived Server-Sent Events stream. Feedback is coalesced into `state`
events at up to 20 Hz:

```text
event: state
data: {"apiVersion":"v1","robotModel":"nero",...}
```

Node.js can consume it with `fetch()` and `response.body`, while browser
backends can use `EventSource`. Reconnect after a network interruption and
then fetch `/state` once to obtain a fresh snapshot.

## Command endpoints

All command endpoints require `enable_commands:=true` and return:

```json
{"ok":true,"accepted":true,"requestId":"..."}
```

### `POST /api/v1/commands/pose`

```json
{
  "frameId": "base_link",
  "position": {"x": 0.3, "y": 0.0, "z": 0.4},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

The gateway rejects missing/non-finite values and normalizes the quaternion.
Workspace and IK/FK validation remain the controller's responsibility and are
reported asynchronously through `ikStatus`.

### `POST /api/v1/commands/keys`

```json
{"keys":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}
```

This accepts exactly 23 binary values for the existing virtual controller.
The backend must send all zeroes on button release and disconnect.

### `POST /api/v1/commands/action`

Use semantic actions when the backend does not need to manage key edges:

```json
{"action":"home"}
```

Supported actions are `home`, `estop`, `toggle_control_mode`,
`toggle_impedance`, and `release`. The gateway generates a 50 ms key edge and
automatic release for the first four actions.

## Errors

Errors use one stable envelope:

```json
{
  "ok": false,
  "error": {
    "code": "invalid_request",
    "message": "orientation quaternion must be non-zero"
  }
}
```

Common statuses are `400 invalid_request`, `401 unauthorized`,
`403 commands_disabled`, and `404 not_found`.

## Authentication and binding

The API binds only to `127.0.0.1` by default, which is suitable when the Node
backend and ROS run on the same computer. To bind to a LAN interface, a token
is mandatory:

```bash
ros2 launch armbycontroller motion_link.launch.py \
  backend_api_host:=0.0.0.0 backend_api_token:="change-me"
```

Send the token as either:

```text
Authorization: Bearer change-me
X-Armby-Token: change-me
```

## Launch

Backend interaction in simulation:

```bash
ros2 launch armbycontroller motion_link.launch.py \
  robot_model:=nero simulation_mode:=true \
  backend_transport:=http \
  enable_commands:=true execute_motion:=false
```

Read-only hardware status:

```bash
ros2 launch armbycontroller motion_link.launch.py \
  robot_model:=nero simulation_mode:=false \
  backend_transport:=http \
  enable_commands:=false execute_motion:=false
```

Hardware motion must opt into both gates:

```bash
ros2 launch armbycontroller motion_link.launch.py \
  robot_model:=nero simulation_mode:=false \
  backend_transport:=http \
  enable_commands:=true execute_motion:=true can_interface:=can0
```

For the existing Phone Remotation WebSocket protocol, omit
`backend_transport`; its default is `motion_link`. Switch to `http` only after
the Node backend sends commands through the endpoints above.

## Minimal Node.js backend call

Node.js 18+ can call the gateway without another dependency:

```js
const armApi = process.env.ARMBY_API || "http://127.0.0.1:8765/api/v1";

export async function sendArmAction(action) {
  const response = await fetch(`${armApi}/commands/action`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action })
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error?.message || "arm API failed");
  return result.requestId;
}

export async function readArmState() {
  const response = await fetch(`${armApi}/state`, { cache: "no-store" });
  return (await response.json()).state;
}
```

Configure the Phone Remotation backend with `ARMBY_API` when a non-default
port is used. Keep this URL server-side; phone clients should continue talking
to the Phone Remotation backend rather than bypassing it.
