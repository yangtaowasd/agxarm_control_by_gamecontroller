#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIRECTORY="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd
)"
"${SCRIPT_DIRECTORY}/setup_can.sh"
KEYBOARD_DEVICE="$(
  "${SCRIPT_DIRECTORY}/select_input_device.sh" "$@"
)"
readonly KEYBOARD_DEVICE

SOURCE_WORKSPACE_DIRECTORY="$(
  cd -- "${SCRIPT_DIRECTORY}/../../.." && pwd
)"
SOURCE_WORKSPACE_SETUP="${SOURCE_WORKSPACE_DIRECTORY}/install/setup.bash"
INSTALLED_WORKSPACE_SETUP="${SCRIPT_DIRECTORY}/../../../../setup.bash"

if [[ -f "${SOURCE_WORKSPACE_SETUP}" ]]; then
  WORKSPACE_SETUP="${SOURCE_WORKSPACE_SETUP}"
elif [[ -f "${INSTALLED_WORKSPACE_SETUP}" ]]; then
  WORKSPACE_SETUP="${INSTALLED_WORKSPACE_SETUP}"
else
  echo "Error: ROS workspace setup.bash was not found." >&2
  echo "Build the workspace first:" >&2
  echo "  cd ${SOURCE_WORKSPACE_DIRECTORY}" >&2
  echo "  colcon build --packages-up-to agxarm_control_by_gamecontroller" >&2
  exit 1
fi
readonly WORKSPACE_SETUP

set +u
# shellcheck source=/dev/null
source "${WORKSPACE_SETUP}"
set -u

exec ros2 launch agxarm_control_by_gamecontroller keyboard_control.launch.py \
  robot_model:=piper_l \
  execute_motion:=true \
  can_interface:=can0 \
  device:="${KEYBOARD_DEVICE}" \
  move_home_on_start:=false \
  reset_emergency_stop_on_start:=true \
  "$@"
