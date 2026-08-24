#!/usr/bin/env bash

set -euo pipefail

for argument in "$@"; do
  if [[ "${argument}" == nero_mount:=* ]]; then
    mount="${argument#nero_mount:=}"
    case "${mount}" in
      side | horizontal)
        printf '%s\n' "${mount}"
        exit 0
        ;;
      *)
        printf 'Error: nero_mount must be side or horizontal: %s\n' \
          "${mount}" >&2
        exit 1
        ;;
    esac
  fi
done

if [[ ! -t 0 ]]; then
  printf '%s\n' "side"
  exit 0
fi

printf '%s\n' "Select Nero mounting / 选择 Nero 安装姿态:" >&2
printf '%s\n' \
  "  1) 横置 / side — home [0°,90°,0°,0°,0°,0°,0°], J1→J2→J3…" >&2
printf '%s\n' \
  "  2) 平置 / horizontal — home all 0°, J2→J1→J3→J4…" >&2
printf '%s' "Choice [1]: " >&2
read -r mount_choice

case "${mount_choice:-1}" in
  1 | side | SIDE | 横置)
    printf '%s\n' "side"
    ;;
  2 | horizontal | HORIZONTAL | 平置)
    printf '%s\n' "horizontal"
    ;;
  *)
    printf 'Error: invalid Nero mounting selection: %s\n' \
      "${mount_choice}" >&2
    exit 1
    ;;
esac
