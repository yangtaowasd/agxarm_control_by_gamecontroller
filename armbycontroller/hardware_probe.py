#!/usr/bin/env python3
"""Probe an AGX arm without enabling it or issuing motion commands."""

import argparse
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, PiperFW
from pyAgxArm import create_agx_arm_config

from armbycontroller.keyboard_controller import extract_joint_angles
from armbycontroller.model_profiles import get_arm_profile


HARDWARE_PROFILES = {
    "nero": (ArmModel.NERO, NeroFW.V112),
    "piper_l": (ArmModel.PIPER_L, PiperFW.V188),
}


def hardware_available(model, can_interface, timeout):
    """Return true only when complete joint feedback can be read."""
    arm_model, firmware = HARDWARE_PROFILES[model]
    joint_count = get_arm_profile(model).joint_count
    config = create_agx_arm_config(
        robot=arm_model,
        firmeware_version=firmware,
        interface="socketcan",
        channel=can_interface,
    )
    arm = AgxArmFactory.create_arm(config)
    try:
        arm.connect()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if extract_joint_angles(
                arm.get_joint_angles(), joint_count
            ) is not None:
                return True
            time.sleep(0.05)
        return False
    except Exception:
        return False
    finally:
        try:
            arm.disconnect()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model", choices=sorted(HARDWARE_PROFILES), required=True
    )
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--timeout", type=float, default=1.5)
    arguments = parser.parse_args()
    if arguments.timeout <= 0.0:
        parser.error("--timeout must be positive")
    return 0 if hardware_available(
        arguments.model,
        arguments.can_interface,
        arguments.timeout,
    ) else 1


if __name__ == "__main__":
    sys.exit(main())
