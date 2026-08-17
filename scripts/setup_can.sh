#!/usr/bin/env bash

set -euo pipefail

readonly CAN_BITRATE=1000000
readonly CAN_INTERFACES=(can0 can1)

if ! command -v ip >/dev/null 2>&1; then
  echo "Error: the 'ip' command was not found. Install the iproute2 package." >&2
  exit 1
fi

if (( EUID == 0 )); then
  IP_COMMAND=(ip)
elif command -v sudo >/dev/null 2>&1; then
  IP_COMMAND=(sudo ip)
else
  echo "Error: root privileges are required. Run this script as root." >&2
  exit 1
fi

for interface in "${CAN_INTERFACES[@]}"; do
  if ! ip link show dev "${interface}" >/dev/null 2>&1; then
    echo "Error: CAN interface '${interface}' does not exist." >&2
    exit 1
  fi
done

for interface in "${CAN_INTERFACES[@]}"; do
  "${IP_COMMAND[@]}" link set dev "${interface}" down
  "${IP_COMMAND[@]}" link set dev "${interface}" type can bitrate "${CAN_BITRATE}"
  "${IP_COMMAND[@]}" link set dev "${interface}" up
  echo "Configured ${interface}: up, bitrate ${CAN_BITRATE} bit/s"
done
