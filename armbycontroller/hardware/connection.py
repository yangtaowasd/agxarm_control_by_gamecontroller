"""Two-stage AGX arm discovery and control connection."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import re
import time


class FirmwareDetectionError(RuntimeError):
    """Raised when the probe connection cannot identify a safe profile."""


@dataclass(frozen=True)
class ArmConnection:
    """The formal arm instance and data saved by the probe connection."""

    arm: object
    firmware_info: dict
    firmware_profile: str
    config: dict


def _software_version(firmware_info):
    if not isinstance(firmware_info, Mapping):
        raise FirmwareDetectionError(
            "firmware response must be a mapping containing software_version"
        )
    version = str(firmware_info.get("software_version", "")).strip()
    if not version:
        raise FirmwareDetectionError(
            "firmware response does not contain software_version"
        )
    return version


def _nero_profile(version):
    match = re.search(r"\d+(?:\.\d+)?", version)
    if match is None:
        raise FirmwareDetectionError(
            f"cannot parse Nero software version {version!r}"
        )
    try:
        numeric = Decimal(match.group(0))
    except InvalidOperation as error:
        raise FirmwareDetectionError(
            f"cannot parse Nero software version {version!r}"
        ) from error
    if numeric >= Decimal("1.20"):
        return "v120"
    if numeric >= Decimal("1.12"):
        return "v112"
    if numeric >= Decimal("1.11"):
        return "v111"
    return "default"


def _piper_profile(version):
    match = re.search(r"(\d+)\.(\d+)-(\d+)", version)
    if match is None:
        raise FirmwareDetectionError(
            f"cannot parse Piper software version {version!r}"
        )
    numeric = tuple(int(value) for value in match.groups())
    if numeric >= (1, 8, 9):
        return "v189"
    if numeric >= (1, 8, 8):
        return "v188"
    if numeric >= (1, 8, 3):
        return "v183"
    return "default"


def firmware_profile_from_info(robot_model, firmware_info):
    """Map saved firmware metadata to one pyAgxArm driver profile."""
    model = str(robot_model).lower()
    version = _software_version(firmware_info)
    if model == "nero":
        return _nero_profile(version)
    if model == "piper_l":
        return _piper_profile(version)
    raise ValueError("robot_model must be nero or piper_l")


def _wait_for_firmware(
    arm,
    timeout,
    poll_period,
    *,
    report: Callable[[str], None] | None = None,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    announce = report if report is not None else lambda message: None
    deadline = monotonic() + timeout
    last_error = None
    query_count = 0
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            detail = f"; last query error: {last_error}" if last_error else ""
            raise FirmwareDetectionError(
                f"timed out after {timeout:.3f} s and {query_count} queries "
                f"waiting for firmware{detail}"
            )
        query_count += 1
        query_timeout = min(1.0, remaining)
        announce(
            f"firmware probe query {query_count}: requesting device data "
            f"with timeout={query_timeout:.3f} s"
        )
        query_error = None
        try:
            firmware_info = arm.get_firmware(
                timeout=query_timeout, min_interval=0.0
            )
        except Exception as error:  # Hardware query errors are retried.
            last_error = error
            query_error = error
            firmware_info = None
            announce(
                f"firmware probe query {query_count} failed: "
                f"{type(error).__name__}: {error}"
            )
        if firmware_info is not None:
            announce(
                f"firmware probe query {query_count} received: "
                f"{firmware_info!r}"
            )
            _software_version(firmware_info)
            return dict(firmware_info)
        if query_error is None:
            announce(
                f"firmware probe query {query_count}: no response"
            )
        sleep(min(poll_period, max(0.0, deadline - monotonic())))


def connect_arm_two_stage(
    *,
    robot_model,
    arm_model,
    firmware_profiles,
    can_interface,
    probe_timeout=5.0,
    probe_poll_period=0.1,
    reconnect_delay=0.5,
    arm_factory=None,
    config_factory=None,
    report: Callable[[str], None] | None = None,
    monotonic=time.monotonic,
    sleep=time.sleep,
):
    """
    Probe with DEFAULT, disconnect, then reconnect with detected data.

    The first arm object is used only to query and save firmware metadata. It
    is always disconnected before a distinct, profile-specific arm object is
    created. The returned object is the second, formal control connection.
    """
    model = str(robot_model).lower()
    if model not in ("nero", "piper_l"):
        raise ValueError("robot_model must be nero or piper_l")
    if not isinstance(firmware_profiles, Mapping):
        raise TypeError("firmware_profiles must be a mapping")
    if "default" not in firmware_profiles:
        raise ValueError("firmware_profiles must contain default")
    if not timeout_is_valid(probe_timeout):
        raise ValueError("probe_timeout must be finite and greater than zero")
    if not period_is_valid(probe_poll_period):
        raise ValueError("probe_poll_period must be finite and nonnegative")
    if not period_is_valid(reconnect_delay):
        raise ValueError("reconnect_delay must be finite and nonnegative")

    if arm_factory is None or config_factory is None:
        from pyAgxArm import AgxArmFactory, create_agx_arm_config

        arm_factory = AgxArmFactory if arm_factory is None else arm_factory
        config_factory = (
            create_agx_arm_config
            if config_factory is None
            else config_factory
        )
    announce = report if report is not None else lambda message: None

    probe_config = config_factory(
        robot=arm_model,
        firmeware_version=firmware_profiles["default"],
        interface="socketcan",
        channel=can_interface,
    )
    probe_arm = arm_factory.create_arm(probe_config)
    probe_enable_attempted = False
    announce(
        f"firmware probe connection: {model} on {can_interface} "
        "with default profile"
    )
    try:
        probe_arm.connect()
        probe_enable_attempted = True
        announce(
            f"{model} firmware probe: sending one temporary enable request"
        )
        try:
            enable_result = probe_arm.enable()
            announce(
                f"{model} firmware probe enable request result: "
                f"{enable_result}"
            )
        except Exception as error:
            announce(
                f"{model} firmware probe enable request failed: "
                f"{type(error).__name__}: {error}"
            )
        firmware_info = _wait_for_firmware(
            probe_arm,
            float(probe_timeout),
            float(probe_poll_period),
            report=announce,
            monotonic=monotonic,
            sleep=sleep,
        )
        firmware_profile = firmware_profile_from_info(model, firmware_info)
        if firmware_profile not in firmware_profiles:
            raise FirmwareDetectionError(
                f"detected unsupported {model} profile {firmware_profile!r}"
            )
        announce(
            f"firmware probe data saved: {firmware_info}; "
            f"selected profile={firmware_profile}"
        )
    finally:
        if probe_enable_attempted:
            try:
                disable_result = probe_arm.disable()
                announce(
                    f"{model} firmware probe disable request result: "
                    f"{disable_result}"
                )
            except Exception as error:
                announce(
                    f"{model} firmware probe disable request failed: "
                    f"{type(error).__name__}: {error}"
                )
        probe_arm.disconnect()
        announce("firmware probe disconnected")

    reconnect_delay = float(reconnect_delay)
    if reconnect_delay > 0.0:
        announce(
            f"waiting {reconnect_delay:.3f} s before formal connection"
        )
        sleep(reconnect_delay)

    formal_config = config_factory(
        robot=arm_model,
        firmeware_version=firmware_profiles[firmware_profile],
        interface="socketcan",
        channel=can_interface,
    )
    formal_arm = arm_factory.create_arm(formal_config)
    try:
        formal_arm.connect()
    except Exception:
        formal_arm.disconnect()
        raise
    announce(
        f"formal control connection: {model} on {can_interface} "
        f"with detected profile {firmware_profile}"
    )
    return ArmConnection(
        formal_arm,
        firmware_info,
        firmware_profile,
        formal_config,
    )


def timeout_is_valid(value):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return False
    return numeric > 0.0 and numeric < float("inf")


def period_is_valid(value):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return False
    return numeric >= 0.0 and numeric < float("inf")
