#!/usr/bin/env bash

set -euo pipefail

normalize_keyboard_device_path() {
  local device_path="$1"

  if [[ "${device_path}" =~ ^[0-9]+$ ]]; then
    printf '/dev/input/event%s\n' "${device_path}"
  elif [[ "${device_path}" =~ ^event[0-9]+$ ]]; then
    printf '/dev/input/%s\n' "${device_path}"
  else
    printf '%s\n' "${device_path}"
  fi
}

for argument in "$@"; do
  if [[ "${argument}" == device:=* ]]; then
    normalize_keyboard_device_path "${argument#device:=}"
    exit 0
  fi
done

if [[ ! -t 0 ]]; then
  printf '%s\n' "x11"
  exit 0
fi

printf '%s\n' "Select keyboard input / 选择键盘输入:" >&2
printf '%s\n' "  1) x11 (NoMachine/desktop)" >&2
printf '%s\n' "  2) local keyboard (/dev/input/eventN)" >&2
printf '%s' "Choice [1]: " >&2
read -r input_choice

case "${input_choice:-1}" in
  1 | x11 | X11)
    printf '%s\n' "x11"
    ;;
  2 | keyboard | evdev)
    if [[ -d /dev/input/by-id ]]; then
      printf '%s\n' "Available stable keyboard paths:" >&2
      for device_path in /dev/input/by-id/*-event-kbd; do
        if [[ -e "${device_path}" ]]; then
          printf '  %s\n' "${device_path}" >&2
        fi
      done
    fi
    printf '%s' "Keyboard device number or path [3]: " >&2
    read -r device_path
    device_path="$(
      normalize_keyboard_device_path "${device_path:-3}"
    )"
    if [[ ! -e "${device_path}" ]]; then
      printf 'Error: keyboard device does not exist: %s\n' \
        "${device_path}" >&2
      exit 1
    fi
    if [[ ! -r "${device_path}" ]]; then
      printf 'Error: keyboard device is not readable: %s\n' \
        "${device_path}" >&2
      printf '%s\n' \
        "Add the user to the input group, then log out and back in." >&2
      exit 1
    fi
    printf '%s\n' "${device_path}"
    ;;
  *)
    printf 'Error: invalid keyboard selection: %s\n' \
      "${input_choice}" >&2
    exit 1
    ;;
esac
